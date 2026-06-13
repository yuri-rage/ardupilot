/*
    Extends AUTO mode by implementing the Stanley tracking controller described in:
    https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf

    When ATC_STAN_USE is enabled, lateral control (steering) is replaced by
    Stanley geometric path guidance:

    - Ackermann vehicles command steering angle directly (feed-forward).
    - Skid-steered vehicles convert steering angle to a target turn rate and
      utilize the steering rate controller.

    Longitudinal control (speed/throttle) remains managed by the position and
    speed controllers.
*/

#include "Rover.h"

#if MODE_AUTO_STANLEY_ENABLED

bool ModeAuto::_enter_auto_stanley()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Auto: Using Stanley controller.");
    return true;
}

// run Stanley controller
void Mode::run_stanley_control()
{
    float speed = 0.0f;
    attitude_control.get_forward_speed(speed);
    const float speed_abs = fabsf(speed);

    const float k = attitude_control.get_stan_k();
    const float v0 = attitude_control.get_stan_v0();
    const float kd = attitude_control.get_stan_kd();
    const float yaw_rad = ahrs.get_yaw();

    // Path heading from the current segment (origin → destination)
    float path_heading_rad = 0.0f;
    const Vector2f dest_from_origin = g2.wp_nav.get_oa_origin().get_distance_NE(g2.wp_nav.get_oa_destination());
    if (dest_from_origin.length() >= 1.0e-6f) {
        path_heading_rad = dest_from_origin.angle();
    } else {
        path_heading_rad = yaw_rad;
    }

    float crosstrack_error = 0.0f;
    const float current_seg_len = dest_from_origin.length();
    const float wheelbase = attitude_control.get_wheelbase_len();
    if (current_seg_len >= 1.0e-6f) {
        crosstrack_error = g2.wp_nav.crosstrack_error_m();

        // project cross-track error from the AHRS/rear axle reference to the front axle center.
        if (is_positive(wheelbase)) {
            Vector2f path_dir = dest_from_origin;
            path_dir.normalize();
            const Vector2f yaw_dir{cosf(yaw_rad), sinf(yaw_rad)};
            crosstrack_error += wheelbase * (yaw_dir % path_dir);

            // Scale crosstrack error for segments shorter than vehicle wheelbase
            if (current_seg_len < wheelbase) {
                crosstrack_error *= (current_seg_len / wheelbase);
            }
        }
    }

    // blend path_heading toward the next segment so the turn begins early
    // rather than after the waypoint is overshot; same blend factor is used
    // below to fade the crosstrack correction so it does not fight the turn.
    float corner_blend = 0.0f;
    float turn_radius = 0.0f;
    const float wp_radius = g2.wp_nav.get_radius();
    float next_seg_heading = 0.0f;
    const Location &next_dest = g2.wp_nav.get_next_destination();
    if (next_dest.initialised()) {
        const Vector2f next_from_dest = g2.wp_nav.get_oa_destination().get_distance_NE(next_dest);
        const float next_seg_len = next_from_dest.length();
        if (next_seg_len >= 1.0e-6f) {
            next_seg_heading = next_from_dest.angle();
            const float corner_angle_rad = fabsf(wrap_PI(next_seg_heading - path_heading_rad));

            // physical minimum turn radius:
            //   skid-steer can pivot in place → 0
            //   Ackermann is limited by wheelbase / tan(max_steer_angle)
            float turn_radius_min = 0.0f;
            if (!g2.motors.have_skid_steering()) {
                const float steer_angle_max_rad = attitude_control.get_steer_angle_max_rad();
                if (is_positive(steer_angle_max_rad) && is_positive(wheelbase)) {
                    turn_radius_min = wheelbase / tanf(steer_angle_max_rad);
                }
            }

            // turn radius capped by r_max so geometric corner anticipation stays bounded
            turn_radius = turn_radius_min;
            const float lat_accel_max = attitude_control.get_turn_lat_accel_max();
            if (is_positive(lat_accel_max) && speed_abs > 0.1f) {
                turn_radius = MAX(turn_radius_min, sq(speed_abs) / lat_accel_max);
            }

            const float denom = 1.0f / cosf(corner_angle_rad * 0.5f) - 1.0f;
            if (denom > 1.0e-6f) {
                const float r_max = (wp_radius * 0.9f) / denom;
                turn_radius = MIN(turn_radius, r_max);
            }
            turn_radius = MAX(turn_radius, turn_radius_min);

            // distance from current position at which the turn should begin based on achievable radius
            float tangent_dist = turn_radius * tanf(corner_angle_rad * 0.5f);

            // cap tangent distance to 50% of the shortest leg length
            const float max_t = 0.5f * MIN(current_seg_len, next_seg_len);
            tangent_dist = MIN(tangent_dist, max_t);

            const float blend_start = MAX(tangent_dist, wp_radius);

            Location current_loc;
            if (AP::ahrs().get_location(current_loc)) {
                const float prop = current_loc.line_path_proportion(g2.wp_nav.get_oa_origin(), g2.wp_nav.get_oa_destination());
                const float along_track_rem = (1.0f - prop) * current_seg_len;
                if (along_track_rem < blend_start && is_positive(blend_start)) {
                    // blend linearly: 0 at blend_start distance, 0.5 at waypoint (apex of corner arc)
                    corner_blend = 0.5f * constrain_float(1.0f - along_track_rem / blend_start, 0.0f, 1.0f);
                    path_heading_rad = wrap_PI(path_heading_rad + corner_blend * wrap_PI(next_seg_heading - path_heading_rad));
                }
            }
        }
    }

    const float heading_error = wrap_PI(path_heading_rad - yaw_rad);

    // crosstrack correction fades as the vehicle enters the corner blend zone
    // at full blend, crosstrack reaches zero, so it cannot fight the turn toward the next segment
    const float cross_track_term = atanf(k * crosstrack_error * (1.0f - corner_blend) / (speed_abs + v0));
    float delta = heading_error + cross_track_term;

    // trajectory yaw rate during a blended corner turn
    float yaw_rate_traj = 0.0f;
    if (corner_blend > 0.0f && turn_radius >= 0.1f) {
        const float turn_sign = (wrap_PI(next_seg_heading - path_heading_rad) >= 0.0f) ? 1.0f : -1.0f;
        yaw_rate_traj = corner_blend * (speed_abs / turn_radius) * turn_sign;
    }

    // active yaw rate damping
    const float yaw_rate_meas = ahrs.get_yaw_rate_earth();
    const float damping = kd * (yaw_rate_meas - yaw_rate_traj);
    delta -= damping;

    float steer_angle_max_rad = attitude_control.get_steer_angle_max_rad();
    if (is_zero(steer_angle_max_rad)) {
        steer_angle_max_rad = radians(45.0f);
    }

    if (g2.motors.have_skid_steering()) {
        // skid steered vehicle: convert steering angle to desired turn rate (yaw rate).
        // get_steering_out_rate() applies the lateral-acceleration (G) limit,
        // so no need to overconstrain here.
        delta = constrain_float(delta, -steer_angle_max_rad, steer_angle_max_rad);

        float wb = wheelbase;
        if (is_zero(wb)) {
            wb = 1.0f;
        }
        // kinematic conversion: r = v * tan(delta) / L
        // to handle stationary or near-zero speed, enforce a minimum speed scale
        float speed_scale = MAX(speed_abs, 0.5f);
        const float desired_turn_rate_rads = (speed_scale * tanf(delta)) / wb;

        // pass desired turn rate to rate controller (G-limiting applied inside)
        calc_steering_from_turn_rate(desired_turn_rate_rads);
    } else {
        // Ackermann steered vehicle: apply G-limit and convert desired steering angle directly to servo output
        const float lat_accel_max = attitude_control.get_turn_lat_accel_max();
        if (lat_accel_max > 0.0f && speed_abs > 0.1f) {
            float wb = wheelbase;
            if (is_zero(wb)) {
                wb = 1.0f;
            }
            const float delta_max_g = atanf((lat_accel_max * wb) / sq(speed_abs));
            steer_angle_max_rad = MIN(steer_angle_max_rad, delta_max_g);
        }

        delta = constrain_float(delta, -steer_angle_max_rad, steer_angle_max_rad);
        const float steering_out = attitude_control.get_steering_out_angle(delta, rover.G_Dt);
        set_steering(steering_out * 4500.0f);
    }
}

#endif
