
--  vtol lua script


local MODE_MANUAL = 0    
local MODE_FBWA   = 5    
local MODE_QHOVER = 18   

local CH_SAFETY_THR = 6  
local CH_VTOL_SW    = 7  
local CH_SAFETY_SW  = 8  

local PWM_MID = 1500
local PWM_MIN = 1000
local PWM_MAX = 2000

local MAX_ATTITUDE_LIMIT_DEG = 20
local MIN_LIMIT_DEG = 5
local UPDATE_PERIOD_MS = 100

-- state
local last_requested_mode = -1
local last_limit_deg = -1
local ahrs_ever_healthy   = false  
local ahrs_unhealthy_warned = false 


--some helper functions

local function get_rc(ch)
    return rc:get_pwm(ch)
end

local function is_switch_on(pwm)
    if pwm == nil then return false end
    return pwm > PWM_MID
end

local function pwm_to_unit(pwm)
    if pwm == nil then return 0.0 end
    local val = (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)
    if val < 0 then val = 0 end
    if val > 1 then val = 1 end
    return val
end


local function request_mode(mode)
    if mode ~= last_requested_mode then
        vehicle:set_mode(mode)
        last_requested_mode = mode
        gcs:send_text(6, string.format("VTOL script: mode -> %d", mode))
    end
end


local function set_attitude_limits(limit_deg)
    local effective = math.max(MIN_LIMIT_DEG, limit_deg)
    local rounded   = math.floor(effective + 0.5)   
    if rounded ~= last_limit_deg then
        param:set_and_save("ROLL_LIMIT_DEG",    rounded)
        param:set_and_save("PTCH_LIM_MAX_DEG",  rounded)
        param:set_and_save("PTCH_LIM_MIN_DEG", -rounded)
        last_limit_deg = rounded
    end
end



function update()

    local ahrs_ok = ahrs:healthy()

    if ahrs_ok then
        ahrs_ever_healthy   = true
        ahrs_unhealthy_warned = false
    else
        if ahrs_ever_healthy then
            request_mode(MODE_MANUAL)
            if not ahrs_unhealthy_warned then
                gcs:send_text(3, "VTOL script: AHRS UNHEALTHY -> MANUAL fallback")
                ahrs_unhealthy_warned = true
            end
        end
        return update, UPDATE_PERIOD_MS
    end


    local vtol_pwm   = get_rc(CH_VTOL_SW)
    local safety_pwm = get_rc(CH_SAFETY_SW)
    local thr_pwm    = get_rc(CH_SAFETY_THR)

    local vtol_on   = is_switch_on(vtol_pwm)
    local safety_on = is_switch_on(safety_pwm)

    if vtol_on and not safety_on then
        request_mode(MODE_QHOVER)

    elseif not vtol_on and safety_on then
        request_mode(MODE_FBWA)
        local thr_unit  = pwm_to_unit(thr_pwm)
        local limit_deg = thr_unit * MAX_ATTITUDE_LIMIT_DEG
        set_attitude_limits(limit_deg)

    elseif vtol_on and safety_on then
        request_mode(MODE_QHOVER)

    else
        request_mode(MODE_MANUAL)
        set_attitude_limits(MAX_ATTITUDE_LIMIT_DEG)
    end
    return update, UPDATE_PERIOD_MS
end



gcs:send_text(6, "VTOL Tri-Engine mode script started")
return update, 1000
