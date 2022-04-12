// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*
  calculate the apparent wind direction relative to sail and boat 
  */
void Rover::update_wind(void)
{
}

/*
  calculate the sail and rudder servo angles
  */
void Rover::update_servos(void)
{
    static int16_t last_throttle;
    static int16_t ch1_in;
    static int16_t ch2_in;    
    static int16_t ch3_in;
    static int32_t apparent;
    static int32_t diff;            

    // support a separate steering channel
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, channel_steer->pwm_to_angle_dz(0));

	if (control_mode == MANUAL || control_mode == LEARNING) {
        // boat
        ch1_in = RC_Channel::rc_channel(0)->read();
        ch3_in = RC_Channel::rc_channel(2)->read();
        //        RC_Channel::rc_channel(0)->radio_out = 1300+((7*(ch1_in-1500))/4);
        //        RC_Channel::rc_channel(2)->radio_out = 1300+((7*(ch3_in-1500))/4);
        if (g.target_mode == 1) {
            gcs_send_text_fmt(PSTR("RC6 %d"), RC_Channel::rc_channel(5)->read());
            apparent = RC_Channel::rc_channel(5)->read() - g.wind_mid;
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP1: %d", apparent);
            //gcs_send_text_fmt(PSTR("AP1 %d"), apparent);
            apparent = (((apparent * 3600) / (g.wind_max-g.wind_min)) + 7200) % 3600;
            diff = g.target_apparent - apparent;
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "DIFF1: %d", diff);
            //gcs_send_text_fmt(PSTR("DIFF1 %d"), diff);            
            if (diff < -1800) diff = 3600+diff;
            if (diff > 1800) diff = 3600-diff;
            g.rudder_min = g.rudder_mid+(diff/10);
            //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "DIFF1: %d", g.rudder_min);
            //gcs_send_text_fmt(PSTR("WD %d W1 %d W2 %d W3 %d AP %d DF %d RD %d"), RC_Channel::rc_channel(7)->read(), g.wind_min, g.wind_mid, g.wind_max, apparent, diff, g.rudder_min);            
            RC_Channel::rc_channel(0)->radio_out = g.rudder_mid+(diff/10);
            RC_Channel::rc_channel(2)->radio_out = g.sail_mid+((7*(ch3_in-1500))/4);
        } else {        
            RC_Channel::rc_channel(0)->radio_out = g.rudder_mid+((7*(ch1_in-1500))/4);
            RC_Channel::rc_channel(2)->radio_out = g.sail_mid+((7*(ch3_in-1500))/4);
        }
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

}


