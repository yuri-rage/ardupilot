--[[----------------------------------------------------------------------------

Low Speed Failsafe ArduPilot Lua script

Detects low speed during auto missions and triggers failsafe actions.

Edit the 'USER EDITABLE GLOBALS' to configure the failsafe script.

Edit the 'FAILSAFE_SEQUENCE' table to define failsafe actions.

License - GNU GPLv3 https://www.gnu.org/licenses/gpl-3.0.en.html
------------------------------------------------------------------------------]]

-- MAVLINK/AUTOPILOT 'CONSTANTS'
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local ROVER_MODE = { MANUAL = 0, HOLD = 4, LOITER = 5, AUTO = 10 }

-- USER EDITABLE GLOBALS
local SCRIPT_NAME = "Low Speed"
local UPDATE_MS = 1000                  -- (ms) how often to run this script
local LOW_SPEED_THRESHOLD_M_S = 0.25    -- (m/s) speed threshold to trigger failsafe
local LOW_SPEED_TIMEOUT_MS = 10000      -- (ms) timeout to trigger failsafe
local INITIAL_FS_MODE = ROVER_MODE.HOLD -- (mode) initial failsafe mode (nil to skip)
local RESUME_ON_MODE_CHANGE = true      -- (boolean) clear failsafe on mode change
local TRIGGER_ON_MC = true              -- (boolean) trigger failsafe on mission complete

-- ACTION TYPES - USE BELOW TO DEFINE FAILSAFE ACTIONS
local ACTIONS = { disarm = 0, set_mode = 1, set_relay = 2, set_servo = 3 }

--[[
    FAILSAFE SEQUENCE - ACTIONS TO TAKE DURING FAILSAFE

    `time` is the time in seconds to wait before taking the action

    `ACTIONS.disarm` does as it says

    `ACTIONS.set_mode` expects a `state` (mode number)
    -- NOTE: INITIAL_FS_MODE is always set immediately if defined

    `ACTIONS.set_relay` expects an `index` (relay number) and `state` (0 or 1 for off or on)

    `ACTIONS.set_servo` expects an `index` (channel number) and `state` (pwm value)

    `message` can be omitted ]]

local FAILSAFE_SEQUENCE = {
--  { time = (s), action = ACTIONS.action, index = (int), state = (int), message = "message" }
    { time = 5,  action = ACTIONS.disarm },
    { time = 10, action = ACTIONS.set_relay, index = 2, state = 0,    message = "PTO relay off" },
    { time = 12, action = ACTIONS.set_servo, index = 4, state = 1235, message = "Engine idle" },
    { time = 20, action = ACTIONS.set_relay, index = 1, state = 0,    message = "Ignition off" },
}

-- LOCAL VARIABLES
local last_above_threshold_ms = uint32_t(0)
local failsafe_trigger_ms = uint32_t(0)
local next_step = 1

local function handle_action(action)
    if action.message then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s FAILSAFE: %s", SCRIPT_NAME, action.message))
    end

    if action.action == ACTIONS.disarm then
        arming:disarm()
        return
    end

    if action.action == ACTIONS.set_mode then
        if action.state then
            vehicle:set_mode(action.state)
        else
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Mode action requires a mode", SCRIPT_NAME))
        end
        return
    end

    if action.action == ACTIONS.set_relay then
        if not action.index or not action.state then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Relay action requires an index and state", SCRIPT_NAME))
            return
        end
        if action.state and action.state == 0 then
            relay:off(action.index)
        else
            relay:on(action.index)
        end
        return
    end

    if action.action == ACTIONS.set_servo then
        if not action.index or not action.state then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Servo action requires an index and state", SCRIPT_NAME))
            return
        end
        SRV_Channels:set_output_pwm_chan(action.index, action.state)
        return
    end

    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Unknown action: %d", SCRIPT_NAME, action.action))
end

function handle_failsafe_actions()
    if INITIAL_FS_MODE and RESUME_ON_MODE_CHANGE and vehicle:get_mode() ~= INITIAL_FS_MODE then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Mode change, resuming.", SCRIPT_NAME))
        return standby, UPDATE_MS
    end

    for i = next_step, #FAILSAFE_SEQUENCE do
        if millis() - failsafe_trigger_ms > FAILSAFE_SEQUENCE[i].time * 1000 then
            handle_action(FAILSAFE_SEQUENCE[i])
            next_step = i + 1
        end
    end

    if next_step > #FAILSAFE_SEQUENCE then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s FAILSAFE: Actions complete", SCRIPT_NAME))
        return standby, UPDATE_MS
    end

    return handle_failsafe_actions, UPDATE_MS
end

function trigger_failsafe()
    if INITIAL_FS_MODE then
        vehicle:set_mode(INITIAL_FS_MODE)
    end
    next_step = 1
    failsafe_trigger_ms = millis()
    return handle_failsafe_actions, 0
end

function low_speed_watch()
    if not arming:is_armed() then
        return standby, UPDATE_MS
    end

    if TRIGGER_ON_MC and mission:state() == mission.MISSION_COMPLETE then
        return trigger_failsafe, 0
    end

    if mission:state() ~= mission.MISSION_RUNNING then
        return standby, UPDATE_MS
    end

    if ahrs:groundspeed_vector():length() > LOW_SPEED_THRESHOLD_M_S then
        last_above_threshold_ms = millis()
    end

    if millis() - last_above_threshold_ms > LOW_SPEED_TIMEOUT_MS then
        gcs:send_text(MAV_SEVERITY.EMERGENCY, string.format("%s: FAILSAFE!", SCRIPT_NAME))
        return trigger_failsafe, 0
    end

    return low_speed_watch, UPDATE_MS
end

function standby()
    if arming:is_armed() and mission:state() == mission.MISSION_RUNNING then
        last_above_threshold_ms = millis()
        return low_speed_watch, UPDATE_MS
    end
    return standby, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("%s Failsafe script loaded", SCRIPT_NAME))

return standby()
