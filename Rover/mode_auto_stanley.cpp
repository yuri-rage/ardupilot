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

#if AP_ROVER_STANLEY_ENABLED

#define STANLEY_TIMEOUT_MS 200

// run Stanley controller
void Mode::run_stanley_control()
{
    float speed = 0.0f;
    attitude_control.get_forward_speed(speed);
    const float speed_abs = fabsf(speed);

    const float k = attitude_control.get_stan_k();
    const float ki = attitude_control.get_stan_ki();
    const float v0 = attitude_control.get_stan_v0();
    const float kd = attitude_control.get_stan_kd();
    const float yaw_rad = ahrs.get_yaw();

    // Path heading from the current segment (origin -> destination)
    float path_heading_rad = 0.0f;
    const Location &oa_orig = g2.wp_nav.get_oa_origin();
    const Location &oa_dest = g2.wp_nav.get_oa_destination();
    const Vector2f dest_from_origin = oa_orig.get_distance_NE(oa_dest);
    const float current_seg_len = dest_from_origin.length();
    if (current_seg_len >= 1.0e-6f) {
        path_heading_rad = dest_from_origin.angle();
    } else {
        path_heading_rad = yaw_rad;
    }

    float raw_crosstrack_error = 0.0f;
    float crosstrack_error = 0.0f;
    const float wheelbase = attitude_control.get_wheelbase_len();
    if (current_seg_len >= 1.0e-6f) {
        // positive when vehicle reference point is to the left of the path
        raw_crosstrack_error = g2.wp_nav.crosstrack_error_m();
        crosstrack_error = raw_crosstrack_error;

        // project cross-track error from the navigation reference point (AHRS/rear axle) to the front steering axle center
        if (is_positive(wheelbase)) {
            const float eff_wb = MIN(wheelbase, current_seg_len);
            const Vector2f path_dir = dest_from_origin / current_seg_len;
            const Vector2f yaw_dir{cosf(yaw_rad), sinf(yaw_rad)};
            crosstrack_error += eff_wb * (yaw_dir % path_dir);
        }
    }

    float delta_ff = 0.0f;
    float yaw_rate_traj = 0.0f;
    bool in_fillet = false;

    Location current_loc;
    if (current_seg_len >= 1.0e-3f && AP::ahrs().get_location(current_loc)) {
        const bool oa_active = !oa_orig.same_latlon_as(g2.wp_nav.get_origin()) ||
                               !oa_dest.same_latlon_as(g2.wp_nav.get_destination());

        // Evaluate inscribed circular corner fillet for a 3-point turn (p1 -> p2 -> p3)
        auto apply_corner_fillet = [&](const Location &p1, const Location &p2, const Location &p3, bool is_entry) -> bool {
            float r = 0.0f;
            float tangent_dist = 0.0f;
            float turn_angle = 0.0f;
            if (!g2.wp_nav.calc_corner_fillet(p1, p2, p3, r, tangent_dist, turn_angle)) {
                return false;
            }

            const Vector2f seg1 = p1.get_distance_NE(p2);
            const Vector2f seg2 = p2.get_distance_NE(p3);
            const float seg1_len = seg1.length();
            const float seg2_len = seg2.length();
            if (seg1_len < 1.0e-3f || seg2_len < 1.0e-3f) {
                return false;
            }

            const Vector2f u1 = seg1 / seg1_len;
            const Vector2f u2 = seg2 / seg2_len;
            const Vector2f veh_from_p2 = p2.get_distance_NE(current_loc);
            if (is_entry) {
                const float along_rem = -veh_from_p2.dot(u1);
                if (along_rem > tangent_dist) {
                    return false;
                }
            } else {
                const float along_travel = veh_from_p2.dot(u2);
                if (along_travel > tangent_dist) {
                    return false;
                }
            }

            const float hdg1 = seg1.angle();
            const float theta = constrain_float(fabsf(turn_angle), radians(1.0f), radians(178.0f));
            const float half_theta = theta * 0.5f;
            const float turn_sign = (turn_angle >= 0.0f) ? 1.0f : -1.0f;
            const Vector2f n1{-u1.y * turn_sign, u1.x * turn_sign};
            const Vector2f t1_pos = -u1 * tangent_dist;
            const Vector2f arc_center = t1_pos + n1 * r;
            const Vector2f r_vec = veh_from_p2 - arc_center;

            // Measure arc angle relative to the corner bisector (half_theta) so the atan2f
            // branch cut at +/-pi is >90 deg away from the arc endpoints for all corners up to 178 deg
            const float r_cos = r_vec.dot(-n1);
            const float r_sin = r_vec.dot(u1);
            const float cos_half = cosf(half_theta);
            const float sin_half = sinf(half_theta);
            const float beta = atan2f(r_sin * cos_half - r_cos * sin_half,
                                      r_cos * cos_half + r_sin * sin_half);
            const float alpha = constrain_float(beta + half_theta, 0.0f, theta);

            path_heading_rad = wrap_PI(hdg1 + turn_sign * alpha);
            raw_crosstrack_error = turn_sign * (r_vec.length() - r);
            crosstrack_error = raw_crosstrack_error;
            if (is_positive(wheelbase)) {
                const float eff_wb = MIN(wheelbase, current_seg_len);
                const Vector2f path_dir{cosf(path_heading_rad), sinf(path_heading_rad)};
                const Vector2f yaw_dir{cosf(yaw_rad), sinf(yaw_rad)};
                crosstrack_error += eff_wb * (yaw_dir % path_dir);
            }

            const float arc_progress = alpha / theta;
            const float curv_scale = sinf(M_PI * arc_progress);
            const float wb_ff = is_positive(wheelbase) ? wheelbase : 1.0f;
            delta_ff = curv_scale * turn_sign * atanf(wb_ff / r);
            yaw_rate_traj = curv_scale * turn_sign * (speed_abs / r);
            return true;
        };

        if (!oa_active) {
            const Location &prev_orig = g2.wp_nav.get_prev_origin();
            if (prev_orig.initialised()) {
                in_fillet = apply_corner_fillet(prev_orig, oa_orig, oa_dest, false);
            }
            const Location &next_dest = g2.wp_nav.get_next_destination();
            if (!in_fillet && g2.wp_nav.is_fast_waypoint() && next_dest.initialised()) {
                in_fillet = apply_corner_fillet(oa_orig, oa_dest, next_dest, true);
            }
        }
    }

    float steer_angle_max_rad = attitude_control.get_steer_angle_max_rad();
    if (!is_positive(steer_angle_max_rad)) {
        steer_angle_max_rad = radians(45.0f);
    }

    // bounded ground-frame (NE) cross-track integral term to compensate for steady-state crab angle from side slopes,
    // cross-winds, or cross-currents. Integrating the unprojected reference-point error in the ground frame and
    // projecting onto the current path's left normal scales by cos(turn_angle) across corners and reverses sign after
    // 180-deg mowing headland turns. Held across both corner fillets and pivot turns; reset only on mode entry/exit
    // or when KI <= 0.
    const uint32_t now = AP_HAL::millis();
    const bool dt_valid = (_stanley_last_ms != 0) && ((now - _stanley_last_ms) <= STANLEY_TIMEOUT_MS);
    _stanley_last_ms = now;

    float xtrack_int = 0.0f;
    if (!is_positive(ki)) {
        _stanley_xtrack_int_ne.zero();
    } else if (current_seg_len >= 1.0e-6f) {
        const Vector2f path_left_ne{sinf(path_heading_rad), -cosf(path_heading_rad)};
        const float int_proj = _stanley_xtrack_int_ne.dot(path_left_ne);
        if (dt_valid && !in_fillet && (current_seg_len >= 1.0e-3f) &&
            (speed_abs > attitude_control.get_stop_speed())) {
            const bool can_integrate = (raw_crosstrack_error * int_proj < 0.0f) ||
                                       (is_positive(raw_crosstrack_error) && !attitude_control.steering_limit_right()) ||
                                       (is_negative(raw_crosstrack_error) && !attitude_control.steering_limit_left());
            if (can_integrate) {
                const float dt = constrain_float(rover.G_Dt, 0.0f, 1.0f);
                const float int_max_rad = MIN(radians(15.0f), 0.5f * steer_angle_max_rad);
                _stanley_xtrack_int_ne += path_left_ne * (ki * raw_crosstrack_error * dt);
                _stanley_xtrack_int_ne.limit_length(int_max_rad);
            }
        }
        xtrack_int = _stanley_xtrack_int_ne.dot(path_left_ne);
    }

    const float heading_error = wrap_PI(path_heading_rad - yaw_rad);
    const float cross_track_term = atanf(k * crosstrack_error / (speed_abs + v0));
    float delta = delta_ff + heading_error + cross_track_term + xtrack_int;

    // active yaw rate damping
    const float yaw_rate_meas = ahrs.get_yaw_rate_earth();
    const float damping_term = -kd * (yaw_rate_meas - yaw_rate_traj);
    delta += damping_term;

    if (g2.motors.have_skid_steering()) {
        // skid steered vehicle: convert steering angle to desired turn rate (yaw rate).
        // get_steering_out_rate() applies the lateral-acceleration (G) limit,
        // so no need to overconstrain here.
        delta = constrain_float(delta, -steer_angle_max_rad, steer_angle_max_rad);

        float wb = wheelbase;
        if (!is_positive(wb)) {
            wb = 1.0f;
        }
        // kinematic conversion: r = v * tan(delta) / L
        // to handle stationary or near-zero speed, enforce a minimum speed scale
        const float speed_scale = MAX(speed_abs, 0.5f);
        const float desired_turn_rate_rads = (speed_scale * tanf(delta)) / wb;

        // pass desired turn rate to rate controller (G-limiting applied inside)
        calc_steering_from_turn_rate(desired_turn_rate_rads);
    } else {
        // Ackermann steered vehicle: apply G-limit and convert desired steering angle directly to servo output
        const float lat_accel_max = attitude_control.get_turn_lat_accel_max();
        if (is_positive(lat_accel_max) && speed_abs > 0.1f) {
            float wb = wheelbase;
            if (!is_positive(wb)) {
                wb = 1.0f;
            }
            const float delta_max_g = atanf((lat_accel_max * wb) / sq(speed_abs));
            steer_angle_max_rad = MIN(steer_angle_max_rad, delta_max_g);
        }

        delta = constrain_float(delta, -steer_angle_max_rad, steer_angle_max_rad);
        calc_steering_from_angle(delta);
    }

#if HAL_LOGGING_ENABLED
    if (rover.should_log(MASK_LOG_STEERING)) {
        // @LoggerMessage: STAN
        // @Description: Stanley path controller state
        // @Field: TimeUS: Time since system startup
        // @Field: XTrack: Reference-point cross-track error (positive when left of path)
        // @Field: XTrackFA: Front-axle projected cross-track error
        // @Field: HdgErr: Path heading error
        // @Field: XTerm: Proportional cross-track steering term
        // @Field: ITerm: Integral cross-track steering term
        // @Field: DTerm: Yaw-rate damping steering term (added to Delta)
        // @Field: FF: Corner fillet curvature feed-forward steering term
        // @Field: Delta: Total commanded steering angle after angle and G limits
        AP::logger().WriteStreaming(
            "STAN",
            "TimeUS,XTrack,XTrackFA,HdgErr,XTerm,ITerm,DTerm,FF,Delta",
            "smmrrrrrr",
            "F00000000",
            "Qffffffff",
            AP_HAL::micros64(),
            raw_crosstrack_error,
            crosstrack_error,
            heading_error,
            cross_track_term,
            xtrack_int,
            damping_term,
            delta_ff,
            delta);
    }
#endif
}

#endif // AP_ROVER_STANLEY_ENABLED
