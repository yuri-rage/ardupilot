--[[----------------------------------------------------------------------------

copter-circle-drag ArduPilot Lua script

Estimates copter drag coefficients using circle mode.

Set RCx_OPTION=300 for a 3 position switch

CAUTION: This script is capable of engaging and disengaging autonomous control
of a vehicle.  Use this script AT YOUR OWN RISK.

-- Yuri -- May 2023

LICENSE - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

-- TODO: use custom param set instead of constant vars
local COPTER_MASS          = 2.9    -- kg
local AUX_FUNCTION         = 300    -- aux switch function number
local RUN_INTERVAL_MS      = 100

local DRAG_ORIENTATION = { [0] = 'Y', [2] = 'X' }

local COPTER_MODE_CIRCLE   = 7
local MAV_SEVERITY_NOTICE  = 5
local MAV_SEVERITY_INFO    = 6

local roll_rc = rc:get_channel(param:get('RCMAP_ROLL') or 1)
local pitch_rc = rc:get_channel(param:get('RCMAP_PITCH') or 2)
local throttle_rc = rc:get_channel(param:get('RCMAP_THROTTLE') or 3)

local last_heading = 0.0
local cumulative_heading_change = 0.0
local throttle_samples = {}

-- return average of table values
local function average(tbl)
    if not tbl then return 0.0 end
    local sum = 0.0
    for _, val in ipairs(tbl) do
        sum = sum + val
    end
    return sum / #tbl
end

-- return heading in degrees
local function get_heading(yaw)
    yaw = yaw or ahrs:get_yaw()
    local hdg = math.deg(yaw)
    if hdg < 0 then return hdg + 360 end
    return hdg
end

-- return bearing difference in degrees
local function heading_diff(hdg1_deg, hdg2_deg)
    --https://stackoverflow.com/questions/5024375/getting-the-difference-between-two-headings
    return (hdg2_deg - hdg1_deg + 540.0) % 360.0 - 180.0
end

local function sticks_neutral()
    return math.abs(roll_rc:norm_input_dz()) +
           math.abs(pitch_rc:norm_input_dz()) +
           math.abs(throttle_rc:norm_input_dz()) == 0
end

local function init_vars()
    last_heading = get_heading()
    cumulative_heading_change = 0.0
    throttle_samples = {}
end

function measure_thrust()
    if not vehicle:get_likely_flying() then
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Must be flying to tune')
        return await_aux_sw_reset, RUN_INTERVAL_MS
    end
    if vehicle:get_mode() ~= COPTER_MODE_CIRCLE then
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Must be in CIRCLE mode to tune')
        return await_aux_sw_reset, RUN_INTERVAL_MS
    end

    if not sticks_neutral() then
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Neutralize RC sticks and restart tuning')
        return await_aux_sw_reset, RUN_INTERVAL_MS
    end

    local aux_pos = rc:get_aux_cached(AUX_FUNCTION) or 0
    if aux_pos == 0 then
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Circle drag tuning canceled')
        return standby, RUN_INTERVAL_MS
    end
    if aux_pos == 2 then
        return save_params, RUN_INTERVAL_MS
    end

    throttle_samples[#throttle_samples + 1] = motors:get_throttle()
    local heading = get_heading()
    cumulative_heading_change = cumulative_heading_change + heading_diff(last_heading, heading)
    last_heading = heading

    if cumulative_heading_change >= 360.0 then
        local average_throttle = average(throttle_samples)
        local hover_throttle = param:get('MOT_THST_HOVER')
        local drag_force = COPTER_MASS * (average_throttle - hover_throttle) * 9.81
        gcs:send_text(MAV_SEVERITY_NOTICE, ('%f'):format(hover_throttle))
        local drag_direction = param:get('CIRCLE_OPTIONS') & 2
        gcs:send_text(MAV_SEVERITY_NOTICE, ('Avg thr: %f   Hov thr: %f'):format(average_throttle, hover_throttle))
        gcs:send_text(MAV_SEVERITY_NOTICE, ('%s drag force: %f'):format(DRAG_ORIENTATION[drag_direction], drag_force))
        init_vars()
        -- TODO: average results from multiple laps?
        -- TODO: determine how to derive coefficients from calculated force
        -- TODO: automatically set CIRCLE_OPTIONS to other orientation and wait ~180 to stabilize
    end

    return measure_thrust, RUN_INTERVAL_MS
end

function save_params()
    -- TODO: sanity check and save params
    gcs:send_text(MAV_SEVERITY_NOTICE, 'Save function stub')
    return await_aux_sw_reset, RUN_INTERVAL_MS
end

function await_aux_sw_reset()
    if rc:get_aux_cached(AUX_FUNCTION) == 0 then return standby, RUN_INTERVAL_MS end
    return await_aux_sw_reset, RUN_INTERVAL_MS
end

function standby()
    local aux_pos = rc:get_aux_cached(AUX_FUNCTION) or 0
    if aux_pos == 1 then
        init_vars()
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Starting circle drag tuning')
        return measure_thrust, RUN_INTERVAL_MS
    end
    return standby, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, 'Circle drag script loaded')

return standby()
