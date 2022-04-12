// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"


/*****************************************
* Throttle slew limit
*****************************************/
void Rover::throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate && last_throttle != 0) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}

/*
  check for triggering of start of auto mode
 */
bool Rover::auto_check_trigger(void)
{
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false; 
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }
 
    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;            
    }

    if (!is_zero(g.auto_kickstart)) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), (double)xaccel);
            auto_triggered = true;
            return true;            
        }
    }

    return false;   
}

/*
  work out if we are going to use pivot steering
 */
bool Rover::use_pivot_steering(void)
{
    if (control_mode >= AUTO && g.skid_steer_out && g.pivot_turn_angle != 0) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (abs(bearing_error) > g.pivot_turn_angle) {
            return true;
        }
    }
    return false;
}


/*
  test if we are loitering AND should be stopped at a waypoint
*/
bool Rover::in_stationary_loiter()
{
    // Confirm we are in AUTO mode and need to loiter for a time period
    if ((loiter_start_time > 0) && (control_mode == AUTO)) {
        // Check if active loiter is enabled AND we are outside the waypoint loiter radius
        // then the vehicle still needs to move so return false
        if (active_loiter && (wp_distance > g.waypoint_radius)) {
            return false;
        }
        return true;
    }

    return false;
}


/*
  calculate the throtte for auto-throttle modes
 */
void Rover::calc_throttle(float target_speed)
{
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!auto_check_trigger() || in_stationary_loiter()) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;  

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0, 1);
    float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0f - steer_rate*speed_turn_reduction;
    
    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }
    
    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed; 
    
    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle, -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min, g.throttle_max);
    }

    if (!in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0, 1);
        int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        channel_throttle->servo_out = constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min);

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }
    
    if (use_pivot_steering()) {
        channel_throttle->servo_out = 0;
    }
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

void Rover::calc_lateral_acceleration()
{
    switch (control_mode) {
    case AUTO:
        // If we have reached the waypoint previously navigate
        // back to it from our current position
        if (previously_reached_wp && (loiter_duration > 0)) {
            nav_controller->update_waypoint(current_loc, next_WP);
        } else {
            nav_controller->update_waypoint(prev_WP, next_WP);
        }
        break;

    case RTL:
    case GUIDED:
    case STEERING:
        nav_controller->update_waypoint(current_loc, next_WP);
        break;
    default:
        return;
    }

	// Calculate the required turn of the wheels

    // negative error = left turn
	// positive error = right turn
    lateral_acceleration = nav_controller->lateral_acceleration();
    if (use_pivot_steering()) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g*GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g*GRAVITY_MSS;
        }
    }
}

/*
  calculate steering angle given lateral_acceleration
 */
void Rover::calc_nav_steer(void)
{
    // check to see if the rover is loitering
    if (in_stationary_loiter()) {
        channel_steer->servo_out = 0;
        return;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g*GRAVITY_MSS, g.turn_max_g*GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out_lat_accel(lateral_acceleration);
}

/*
  calculate manual steering with offsets
  ToDo: define parameters in g for correct scaling
 */
void Rover::calc_scaled_manual_steering(void)
{    
    int16_t ch1_in;
    int16_t ch3_in;
    int16_t tmp_sail;
    int16_t tmp_rudder;
    
    ch1_in = RC_Channel::rc_channel(0)->read();
    ch3_in = RC_Channel::rc_channel(2)->read();

    tmp_rudder = g.rudder_mid+((8*(ch1_in-1500))/4);
    if (tmp_rudder < g.rudder_min) tmp_rudder = g.rudder_min;
    if (tmp_rudder > g.rudder_max) tmp_rudder = g.rudder_max;
    tmp_sail = g.sail_mid+((8*(ch3_in-1500))/4);
    if (tmp_sail < g.sail_min) tmp_sail = g.sail_min;
    if (tmp_sail > g.sail_max) tmp_sail = g.sail_max;
   
    //RC_Channel::rc_channel(0)->servo_out = tmp_rudder;
    //RC_Channel::rc_channel(2)->servo_out = tmp_sail;
    RC_Channel::rc_channel(0)->radio_out = tmp_rudder;
    RC_Channel::rc_channel(2)->radio_out = tmp_sail;

}

/*
  get apparent
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_apparent(void)
{
    int32_t apparent = RC_Channel::rc_channel(5)->read();
    
    apparent = ((((apparent - g.wind_mid) * 3600) / (g.wind_max - g.wind_min)) + 3600) % 3600;

    return apparent;
}

/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_apparent_diff(int32_t apparent)
{
    int32_t result = (apparent - g.target_apparent);

    if (result > 1800) {
        result = - (3600 - result);
    }
    
    return result;
}

/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_apparent_rudder(void)
{
    int32_t result = 0;
    int32_t diff = get_apparent_diff(get_apparent());
    diff = diff / 2;

    if ((diff > 30) || (diff < -30)) {
        result = diff;
    }

    if (diff > 400) {
        result = 400;
    }

    if (diff < -400) {
        result = -400;
    }

    result = g.rudder_mid + result;

    if (result < g.rudder_min) {
        result = g.rudder_min;
    }

    if (result > g.rudder_max) {
        result = g.rudder_max;
    }
    
    return result;
}

/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_apparent_sail(void)
{   
    return g.sail_angle;
}

int32_t Rover::approximate_apparent_boat(int32_t apparent, int32_t sail)
{
    return -((((sail-g.sail_mid)*(60000 / (g.sail_max - g.sail_mid)))/100)-apparent);
}

int32_t Rover::approximate_sail(int32_t apparent_boat)
{
    int32_t result = 0;
    // int32_t a = 0;
    // int32_t b = 0;
    // int32_t c = 0;
    if (apparent_boat < 1800) {
        // a = ((int32_t)apparent_boat * 100);
        // b = (180000 / (int32_t)(g.sail_max - g.sail_mid));
        // c = (((int32_t)apparent_boat * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
        result = (int32_t)g.sail_mid - (((int32_t)apparent_boat * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
    } else {
        // a = ((3600-(int32_t)apparent_boat) * 100);
        // b = (180000 / (int32_t)(g.sail_max - g.sail_mid));
        // c = (((3600-(int32_t)apparent_boat) * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
        result = (int32_t)g.sail_mid + (((3600-(int32_t)apparent_boat) * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
    }
    //gcs_send_text_fmt(PSTR("AS: AB %.1f a %.1f b %.1f c %.1f R %.1f"), (double)apparent_boat, (double)a, (double)b, (double)c, (double)result);
    return result;
}

int32_t Rover::get_course_sail(int32_t apparent, int32_t sail)
{
    int32_t result = sail;
    int32_t apparent_boat = approximate_apparent_boat(apparent,sail);
    if ((apparent_boat < 1700) || (apparent_boat > 1900)) {
        result = approximate_sail(apparent_boat);
    } else {
        if (sail < g.sail_mid) {
            result = g.sail_min;
        } else {
            result = g.sail_max;
        }
    }

    if (result > sail) {
        if ((result - sail) < g.sail_gap) {
            result = sail;
        } 
        if ((result - sail) > g.sail_step) {
            result = sail + g.sail_step;
        } 
    } else {
        if ((result - sail) > -g.sail_gap) {
            result = sail;
        }
        if ((result - sail) < -g.sail_step) {
            result = sail - g.sail_step;
        }
    }
      
    //gcs_send_text_fmt(PSTR("CS: A %.1f AB %.1f S %.1f R %.1f"), (double)apparent, (double)apparent_boat, (double)sail, (double)result);
    
    if (result > g.sail_max) result = g.sail_max;
    if (result < g.sail_min) result = g.sail_min;    
 
    return result;
}

/*
  get apparent
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_course(void)
{
    return (int32_t) (((ahrs.yaw_sensor / 10) + 1800) % 3600);
}

/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_course_diff(int32_t course)
{
    int32_t result = (course - g.target_course);

    if (result > 1800) {
        result = - (3600 - result);
    }
    
    return result;
}

/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_course_rudder(void)
{
    int32_t result = 0;
    int32_t diff = get_course_diff(get_course());
    diff = diff / 2;

    if ((diff > 30) || (diff < -30)) {
        result = diff;
    }

    if (diff > 400) {
        result = 400;
    }

    if (diff < -400) {
        result = -400;
    }
    
    return g.rudder_mid - result;
}


/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    static int16_t last_throttle;
    static int16_t ch1_in;
    static int16_t ch2_in;    
    static int16_t ch3_in;
    int16_t internal_control_mode = 0;
    
    // support a separate steering channel
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, channel_steer->pwm_to_angle_dz(0));

	if (control_mode == MANUAL || control_mode == LEARNING) {
        // boat
        ch1_in = RC_Channel::rc_channel(0)->read();
        ch2_in = RC_Channel::rc_channel(1)->read();
        ch3_in = RC_Channel::rc_channel(2)->read();
        //        RC_Channel::rc_channel(0)->radio_out = 1300+((7*(ch1_in-1500))/4);
        //        RC_Channel::rc_channel(2)->radio_out = 1300+((7*(ch3_in-1500))/4);

        // check if channel 2 is 'pulled' --> manual override
        if (ch2_in > 1700) {
            internal_control_mode = 0;
        } else {
            internal_control_mode = g.target_mode;
        }

        //gcs_send_text_fmt(PSTR("CS: C %.1f C %.1f C %.1f C %.1f"), (double)apparent, (double)apparent_boat, (double)sail, (double)result);
        //gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), (double)g.rudder_min);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "R: %d", g.rudder_min);
        
        if (internal_control_mode == 1) {
            // gcs_send_text_fmt(PSTR("RC6 %d"), RC_Channel::rc_channel(5)->read());
            // apparent = RC_Channel::rc_channel(5)->read() - g.wind_mid;
            // //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP1: %d", apparent);
            // //gcs_send_text_fmt(PSTR("AP1 %d"), apparent);
            // apparent = (((apparent * 3600) / (g.wind_max-g.wind_min)) + 7200) % 3600;
            // diff = g.target_apparent - apparent;
            // //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "DIFF1: %d", diff);
            // //gcs_send_text_fmt(PSTR("DIFF1 %d"), diff);            
            // if (diff < -1800) diff = 3600+diff;
            // if (diff > 1800) diff = 3600-diff;
            // g.rudder_min = g.rudder_mid+(diff/10);
            // //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "DIFF1: %d", g.rudder_min);
            // //gcs_send_text_fmt(PSTR("WD %d W1 %d W2 %d W3 %d AP %d DF %d RD %d"), RC_Channel::rc_channel(7)->read(), g.wind_min, g.wind_mid, g.wind_max, apparent, diff, g.rudder_min);            
            RC_Channel::rc_channel(0)->radio_out = get_apparent_rudder();
            RC_Channel::rc_channel(2)->radio_out = g.sail_angle;
        } else if (internal_control_mode == 2) {
            RC_Channel::rc_channel(0)->radio_out = get_course_rudder();
            RC_Channel::rc_channel(2)->radio_out = get_course_sail(get_apparent(),last_sail);
            //RC_Channel::rc_channel(0)->servo_out = get_course_rudder();
            //RC_Channel::rc_channel(2)->servo_out = get_course_sail();
        } else if (internal_control_mode == 3) {
            RC_Channel::rc_channel(0)->radio_out = RC_Channel::rc_channel(0)->read();
            RC_Channel::rc_channel(2)->radio_out = RC_Channel::rc_channel(2)->read();
            //RC_Channel::rc_channel(0)->servo_out = 1500;
            //RC_Channel::rc_channel(2)->servo_out = 1500;
        } else {
            calc_scaled_manual_steering();
            //            RC_Channel::rc_channel(0)->radio_out = g.rudder_mid+((7*(ch1_in-1500))/4);
            //            RC_Channel::rc_channel(2)->radio_out = g.sail_mid+((7*(ch3_in-1500))/4);
        }
        last_rudder = RC_Channel::rc_channel(0)->radio_out; 
        last_sail = RC_Channel::rc_channel(2)->radio_out;  
        //ahrs.roll
        //RC_Channel::rc_channel(0)->radio_out = ch1_in;
        //RC_Channel::rc_channel(2)->radio_out = ch3_in;        
        
        /*        
        // do a direct pass through of radio values
        channel_steer->radio_out       = channel_steer->read();
        channel_throttle->radio_out    = channel_throttle->read();
        */
        
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;                        
        }
        
	} else {       
        channel_steer->calc_pwm();
        if (in_reverse) {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                          -g.throttle_max,
                                                          -g.throttle_min);
        } else {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                          g.throttle_min.get(), 
                                                          g.throttle_max.get());
        }

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) && control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
    }

    // record last throttle before we apply skid steering
    last_throttle = channel_throttle->radio_out;

    if (g.skid_steer_out) {
        // convert the two radio_out values to skid steering values
        /*
          mixing rule:
          steering = motor1 - motor2
          throttle = 0.5*(motor1 + motor2)
          motor1 = throttle + 0.5*steering
          motor2 = throttle - 0.5*steering
        */          
        float steering_scaled = channel_steer->norm_output();
        float throttle_scaled = channel_throttle->norm_output();
        float motor1 = throttle_scaled + 0.5f*steering_scaled;
        float motor2 = throttle_scaled - 0.5f*steering_scaled;
        channel_steer->servo_out = 4500*motor1;
        channel_throttle->servo_out = 100*motor2;
        channel_steer->calc_pwm();
        channel_throttle->calc_pwm();
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    channel_steer->output(); 
    channel_throttle->output();
    RC_Channel_aux::output_ch_all();
#endif
}


