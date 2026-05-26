
-- VTOL Tri-Engine Mode Management Script

-- THE THREE FLIGHT MODES WE NEED 
-- The pilot has TWO switches and ONE slider on the remote:
--   * VTOL switch    -> "I want to hover / take off vertically"
--   * SAFETY switch  -> "Limit how aggressively the plane responds"
--   * Safety throttle slider -> "If SAFETY is on, this controls HOW MUCH
--                                I'm willing to let the plane bank/pitch"
--
-- The combination of the two switches selects one of three modes:
--
--   Mode 1: MANUAL          (both switches OFF)
--           = the pilot is in raw control, no computer help
--           = your stick goes directly to the control surfaces
--           = this is "everything OFF, fly it like a 1940s plane"
--
--   Mode 2: VTOL ASSIST     (VTOL ON, SAFETY OFF)
--           = the plane hovers like a quadcopter
--           = motors tilt to vertical, rear motor spins up
--           = computer keeps it level; pilot just commands up/down/sideways
--
--   Mode 3: STABILIZED      (VTOL OFF, SAFETY ON)
--           = normal forward flight, but with safety limits
--           = computer won't let the plane bank/pitch beyond X degrees
--           = X is decided by where the pilot has the safety slider
--           = slider all the way down -> X is small (very gentle plane)
--           = slider all the way up   -> X is up to 20 degrees (normal)
--
-- The fourth combination (both ON) is not in the spec; we default to
-- Mode 2 (VTOL) because being in hover is safer than being in forward
-- flight if the pilot is confused.
--
-- HOW THIS SCRIPT MAPS TO ArduPilot
--   Spec Mode 1 (MANUAL)     -> ArduPilot mode "MANUAL"  (number 0)
--   Spec Mode 2 (VTOL ASSIST)-> ArduPilot mode "QHOVER"  (number 18)
--   Spec Mode 3 (STABILIZED) -> ArduPilot mode "FBWA"    (number 5) fbwa = fly by wire
--                               + we change the bank/pitch limit parameters
--                               on the fly to match the safety slider

-- QHOVER is the VTOL equivalent: aircraft holds its attitude (level), pilot
-- commands climb / descent / horizontal motion
-- SAFETY: WHAT IF THE GYRO BREAKS?
-- --------------------------------
-- The gyro tells the flight controller which way is up. If it dies or
-- gives garbage data, the auto-stabilization would chase phantom errors
-- and crash the plane. Spec section 4 says: if the gyro fails, drop ALL
-- stabilization and give the pilot direct control (Mode 1 MANUAL).
-- TRICKY DETAIL: at startup, AHRS is unhealthy for the first ~10 seconds
-- while the math inside (called an EKF, Extended Kalman Filter) figures
-- out which way is up. That is NOT a fault. So we wait until AHRS has
-- been healthy at least once before we trust an "unhealthy" report.


local MODE_MANUAL = 0    -- raw stick passthrough
local MODE_FBWA   = 5    -- stabilized fixed-wing with bank/pitch limits
local MODE_QHOVER = 18   -- stabilized VTOL hover

-- Which RC channels we read from the remote control.
-- RC channels are numbered 1..16 in ArduPilot. Channels 1..4 are reserved

local CH_SAFETY_THR = 6  -- continuous slider (1000..2000 microseconds)
local CH_VTOL_SW    = 7  -- two-position switch
local CH_SAFETY_SW  = 8  -- two-position switch

-- RC channel values are in microseconds (us), the radio servo-pulse standard.
-- 1000 us = stick / switch at minimum
-- 2000 us = stick / switch at maximum
-- 1500 us = center (mid-stick or switch flip point)
local PWM_MID = 1500
local PWM_MIN = 1000
local PWM_MAX = 2000

-- The biggest bank/pitch angle (degrees) we will EVER let the plane reach
-- in Mode 3 (only when safety slider is all the way up). Spec default = 20.
-- You could raise this for a more aerobatic aircraft, but 20 deg is gentle.
local MAX_ATTITUDE_LIMIT_DEG = 20

-- The smallest bank/pitch limit we will write. We can't write 0 because
-- ArduPilot's pre-flight check refuses to arm if the limit is below ~5 deg
-- (it thinks the plane is mis-configured). So 5 deg is our practical floor:
-- with the safety slider at zero, the pilot can still bank up to 5 deg,
-- but no more.
local MIN_LIMIT_DEG = 5

-- Update period in milliseconds. 100 ms = 10 Hz = ten checks per second.
local UPDATE_PERIOD_MS = 100



-- STATE 
--  we remember the last mode we asked for and only call
-- set_mode when it actually changes.
local last_requested_mode = -1

-- Same idea for the bank/pitch limit. Writing the same value to a
-- parameter every 100 ms is wasteful (and on real hardware it would wear
-- out the flash memory). So we only write when the integer value changes.
local last_limit_deg = -1

-- Tracking for the gyro/AHRS health check (see comment block in update()
-- for why this two-step logic exists).
local ahrs_ever_healthy   = false   -- has the gyro ever been OK since boot?
local ahrs_unhealthy_warned = false -- have we already told the pilot it broke?


-- HELPER FUNCTIONS

-- Read the PWM (in microseconds) of a given RC channel.
-- Returns nil if the channel is not active (e.g. the radio isn't bound,
-- or in simulation before the first RC frame arrives).
local function get_rc(ch)
    return rc:get_pwm(ch)
end

-- Treat a two-position switch as ON if its PWM is above 1500 us.
-- If we can't read the channel at all, assume OFF. Choosing OFF as the
-- default is the "fail safe" choice for both of our switches: with both
-- OFF, the plane goes to MANUAL, which is at least predictable.
local function is_switch_on(pwm)
    if pwm == nil then return false end
    return pwm > PWM_MID
end

-- Convert a slider's PWM (1000..2000 us) to a number between 0.0 and 1.0.
-- Clamped: values outside the range still produce 0.0 or 1.0.
-- Nil channel -> 0.0 (most restrictive, since this controls a SAFETY limit).
local function pwm_to_unit(pwm)
    if pwm == nil then return 0.0 end
    local val = (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)
    if val < 0 then val = 0 end
    if val > 1 then val = 1 end
    return val
end

-- Ask ArduPilot to change the flight mode. If we're already in that mode,
-- do nothing (avoids spamming). Also broadcasts a text message to the
-- ground station (Mission Planner's "Messages" tab) so the pilot can see
-- when and why the mode changed.
--
-- The "6" passed to gcs:send_text is the severity level. MAVLink defines:
--   0 = EMERGENCY  (red, urgent)
--   3 = ERROR      (red)
--   4 = WARNING    (yellow)
--   6 = INFO       (white/grey, normal status messages)
local function request_mode(mode)
    if mode ~= last_requested_mode then
        vehicle:set_mode(mode)
        last_requested_mode = mode
        gcs:send_text(6, string.format("VTOL script: mode -> %d", mode))
    end
end

-- Write the three parameters that limit how far the plane can bank and
-- pitch in FBWA mode. We write all three together so the envelope is
-- always symmetric (pitch up = pitch down = roll).
--
-- We use param:set_and_save (not just param:set) because in ArduPilot 4.5+
-- the in-memory "set" alone doesn't always reach the attitude controller.
-- set_and_save also writes to non-volatile storage so the value survives
-- a reboot. Cost: one flash write per change. Because we round to whole
-- degrees, sliding the slider end-to-end is only ~20 writes total.
local function set_attitude_limits(limit_deg)
    -- Clamp to MIN_LIMIT_DEG so ArduPilot's pre-arm check doesn't complain
    local effective = math.max(MIN_LIMIT_DEG, limit_deg)
    local rounded   = math.floor(effective + 0.5)   -- round to nearest int
    if rounded ~= last_limit_deg then
        param:set_and_save("ROLL_LIMIT_DEG",    rounded)
        param:set_and_save("PTCH_LIM_MAX_DEG",  rounded)
        param:set_and_save("PTCH_LIM_MIN_DEG", -rounded)
        last_limit_deg = rounded
    end
end


-- MAIN LOOP

--
-- ArduPilot's Lua scheduler calls update() every UPDATE_PERIOD_MS (100 ms).
-- Each call does three things in order:
--
--   1) Check that the gyro/AHRS is healthy. If not, force MANUAL and stop.
--   2) Read the two switches and the slider.
--   3) Based on the switch combination, command the appropriate mode and,
--      in Mode 3, update the bank/pitch limits to match the slider.
--
-- The order matters: the safety check (1) happens BEFORE anything else,
-- so that a broken gyro overrides whatever the pilot is asking for.
-- =============================================================================
function update()

    -- --- 1) GYRO / AHRS HEALTH CHECK ----------------------------------------
    --
    -- This block answers: "Is the attitude estimator giving us good data?"
    -- If yes, carry on. If no, distinguish between:
    --    (a) "still booting up" - normal, just wait silently
    --    (b) "was working, now broken" - real fault, force MANUAL
    --
    -- The ahrs_ever_healthy flag is what tells (a) from (b).

    local ahrs_ok = ahrs:healthy()

    if ahrs_ok then
        -- All good. Remember that AHRS has worked at least once, and reset
        -- the "we already warned" flag so a future fault can warn again.
        ahrs_ever_healthy   = true
        ahrs_unhealthy_warned = false
    else
        if ahrs_ever_healthy then
            -- It worked before, now it doesn't -> real sensor failure.
            -- Force MANUAL so the pilot has direct control.
            request_mode(MODE_MANUAL)
            -- Tell the pilot, but only once per fault episode (no spam).
            if not ahrs_unhealthy_warned then
                gcs:send_text(3, "VTOL script: AHRS UNHEALTHY -> MANUAL fallback")
                ahrs_unhealthy_warned = true
            end
        end
        -- Whether we're booting up or in a real fault, don't run the rest
        -- of the logic this cycle. Just schedule the next call and exit.
        return update, UPDATE_PERIOD_MS
    end


    -- --- 2) READ THE SWITCHES AND SLIDER ------------------------------------
    --
    -- Note: get_rc may return nil if the channel isn't active. The helper
    -- functions is_switch_on and pwm_to_unit handle nil gracefully.

    local vtol_pwm   = get_rc(CH_VTOL_SW)
    local safety_pwm = get_rc(CH_SAFETY_SW)
    local thr_pwm    = get_rc(CH_SAFETY_THR)

    local vtol_on   = is_switch_on(vtol_pwm)
    local safety_on = is_switch_on(safety_pwm)


    -- --- 3) PICK THE FLIGHT MODE FROM THE SWITCH COMBINATION ----------------

    if vtol_on and not safety_on then
        -- ====== Mode 2: VTOL ASSIST ===========================================
        -- Hover with computer-assisted stabilization. ArduPilot's QHOVER
        -- does all the hard work: motors tilt to vertical, rear motor
        -- engages, attitude is held level, sticks command climb / drift.
        request_mode(MODE_QHOVER)

    elseif not vtol_on and safety_on then
        -- ====== Mode 3: STABILIZED FIXED-WING + safety-throttle limit ========
        -- Standard fixed-wing FBWA mode, but we cap how far the pilot can
        -- bank or pitch based on where the safety slider is.

        request_mode(MODE_FBWA)

        -- Linear interpolation:
        --   slider at 0   (0 %)   -> limit = 0 deg (we clamp to 5 deg)
        --   slider at 0.5 (50 %)  -> limit = 10 deg
        --   slider at 1.0 (100 %) -> limit = 20 deg (the spec default)
        local thr_unit  = pwm_to_unit(thr_pwm)
        local limit_deg = thr_unit * MAX_ATTITUDE_LIMIT_DEG
        set_attitude_limits(limit_deg)

    elseif vtol_on and safety_on then
        -- ====== Undefined combination: default to VTOL =======================
        -- The spec doesn't say what to do when both switches are on. We
        -- pick QHOVER because in the air, hovering is recoverable; falling
        -- out of a misconfigured forward-flight mode is not.
        request_mode(MODE_QHOVER)

    else
        -- ====== Mode 1: MANUAL ==============================================
        -- Both switches OFF. Hand the plane entirely to the pilot, no
        -- stabilization. ArduPilot ignores the bank/pitch limits in
        -- MANUAL anyway, but we restore them to default values so that if
        -- the pilot flips into Mode 3 they get a sensible starting point.
        request_mode(MODE_MANUAL)
        set_attitude_limits(MAX_ATTITUDE_LIMIT_DEG)
    end

    -- Schedule ourselves to run again in UPDATE_PERIOD_MS milliseconds.
    -- This is how ArduPilot Lua scripts work: you return both your
    -- function and the next call delay.
    return update, UPDATE_PERIOD_MS
end


-- -----------------------------------------------------------------------------
-- SCRIPT STARTUP
-- -----------------------------------------------------------------------------

-- One-shot startup message so we can see in Mission Planner that the
-- script actually loaded (debugging life-saver).
gcs:send_text(6, "VTOL Tri-Engine mode script started")

-- Schedule the FIRST call of update() 1 second from now, so ArduPilot's
-- own boot sequence (sensor init, parameter load, RC bind) is finished
-- before we start touching things.
return update, 1000
