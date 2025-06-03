local RUN_INTERVAL_MS = 200

-- TODO: make these parameters and poll them upon start of an auto mission
local LOOKAHEAD_DISTANCE_M = 4
local SCAN_RESOLUTION_M = 1
local SCAN_ANGLE_DEG = 10
local FENCE_MARGIN = param:get("FENCE_MARGIN") or 2
local WP_RADIUS = param:get("WP_RADIUS") or 2

local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local ROVER_MODE = { HOLD = 4, AUTO = 10, GUIDED = 15 }

local resume_target = Location()
local cur_wp_loc = Location()
local cur_nav_idx = 0

local function wrap_360(deg)
    return (deg + 360) % 360
end

local function heading_diff(hdg1, hdg2)
    local diff = math.abs(wrap_360(hdg1 - hdg2))
    return math.min(diff, 360 - diff)
end

local function is_breach_imminent(bearing, max_dist)
    local loc = assert(ahrs:get_location(), "OA: Failed to get AHRS location")

    for _ = SCAN_RESOLUTION_M, max_dist, SCAN_RESOLUTION_M do
        loc:offset_bearing(bearing, SCAN_RESOLUTION_M)
        if not fence:check_destination_within_fence(loc) then
            return true
        end
    end
    return false
end

local function get_resume_target(loc, bearing)
    local resume_loc = loc:copy()
    resume_loc:offset_bearing(bearing, LOOKAHEAD_DISTANCE_M)
    repeat
        resume_loc:offset_bearing(bearing, SCAN_RESOLUTION_M)
    until fence:check_destination_within_fence(resume_loc)
    return resume_loc
end

-- this is raytracing in a very rudimentary way, not unlike BendyRuler
-- it does not do any shortest path determination
-- but it should discover the initial shortest vector to clear the fence
local function get_clean_target(loc, dist, current_hdg, desired_hdg)
    local target_right = Location()
    local heading_diff_right = 360

    for scan_angle_right = 0, 180, SCAN_ANGLE_DEG do
        target_right = loc:copy()
        local target_bearing = wrap_360(desired_hdg + scan_angle_right)
        target_right:offset_bearing(target_bearing, dist)
        if fence:check_destination_within_fence(target_right) then
            heading_diff_right = heading_diff(target_bearing, current_hdg)
            break
        end
    end

    for scan_angle_left = 0, 180, SCAN_ANGLE_DEG do
        local target_left = loc:copy()
        local target_bearing = wrap_360(desired_hdg - scan_angle_left)
        target_left:offset_bearing(target_bearing, dist)
        if fence:check_destination_within_fence(target_left) then
            local heading_diff_left = heading_diff(target_bearing, current_hdg)
            if heading_diff_left < heading_diff_right then
                return target_left
            end
            return target_right
        end
    end

    return target_right -- we shouldn't get here, but we should return something
end

function run_oa_nav()
    if vehicle:get_mode() ~= ROVER_MODE.GUIDED then
        gcs:send_text(MAV_SEVERITY.NOTICE, "OA: Canceled - switched out of guided mode")
        return run_normal_nav, RUN_INTERVAL_MS
    end

    local loc = assert(ahrs:get_location(), "OA: Failed to get AHRS location")
    local desired_hdg = math.deg(loc:get_bearing(resume_target))
    local current_hdg = math.deg(ahrs:get_yaw())

    if loc:get_distance(resume_target) < WP_RADIUS then
        gcs:send_text(MAV_SEVERITY.NOTICE, "OA: Resuming mission")
        vehicle:set_mode(ROVER_MODE.AUTO)
        return run_normal_nav, RUN_INTERVAL_MS
    end

    local guided_target = get_clean_target(loc, FENCE_MARGIN, current_hdg, desired_hdg)
    if guided_target then
        vehicle:set_target_location(guided_target)
    end

    return run_oa_nav, RUN_INTERVAL_MS
end

function run_normal_nav()
    if mission:state() ~= mission.MISSION_RUNNING then
        return run_normal_nav, RUN_INTERVAL_MS
    end

    local idx = mission:get_current_nav_index()
    if cur_nav_idx ~= idx then
        cur_nav_idx = idx
        local cur_wp = mission:get_item(cur_nav_idx)
        if cur_wp then
            cur_wp_loc:lat(cur_wp:x())
            cur_wp_loc:lng(cur_wp:y())
        end
    end

    local loc = assert(ahrs:get_location(), "OA: Failed to get AHRS location")
    local desired_hdg = math.deg(loc:get_bearing(cur_wp_loc))
    local lookahead_dist = math.min(LOOKAHEAD_DISTANCE_M, loc:get_distance(cur_wp_loc))
    if is_breach_imminent(desired_hdg, lookahead_dist) then
        gcs:send_text(MAV_SEVERITY.NOTICE, "OA: Avoidance active")
        resume_target = get_resume_target(loc, desired_hdg)
        vehicle:set_mode(ROVER_MODE.GUIDED)
        return run_oa_nav, RUN_INTERVAL_MS
    end

    return run_normal_nav, RUN_INTERVAL_MS
end

gcs:send_text(6, "Rover Fence OA Loaded")

return run_normal_nav()
