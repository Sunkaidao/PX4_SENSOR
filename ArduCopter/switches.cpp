#include "Copter.h"

#define CONTROL_SWITCH_DEBOUNCE_TIME_MS  200

//Documentation of Aux Switch Flags:
struct {
    uint8_t CH6_flag    : 2; // 0, 1    // ch6 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH7_flag    : 2; // 2, 3    // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH8_flag    : 2; // 4, 5    // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH9_flag    : 2; // 6, 7    // ch9 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH10_flag   : 2; // 8, 9    // ch10 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH11_flag   : 2; // 10,11   // ch11 aux switch : 0 is low or false, 1 is center or true, 2 is high
    uint8_t CH12_flag   : 2; // 12,13   // ch12 aux switch : 0 is low or false, 1 is center or true, 2 is high
} aux_con;

void Copter::read_control_switch()
{
    uint32_t tnow_ms = millis();

    // calculate position of flight mode switch
    //	modified by ZhangYong 20171220
    
    int8_t switch_position;
    /*uint16_t rc5_in = RC_Channels::rc_channel(CH_5)->get_radio_in();
    if      (rc5_in < 1231) switch_position = 0;
    else if (rc5_in < 1361) switch_position = 1;
    else if (rc5_in < 1491) switch_position = 2;
    else if (rc5_in < 1621) switch_position = 3;
    else if (rc5_in < 1750) switch_position = 4;
    else switch_position = 5;
	*/
	//	modified end
	switch_position = readSwitch();
	

    // store time that switch last moved
    if (control_switch_state.last_switch_position != switch_position) {
        control_switch_state.last_edge_time_ms = tnow_ms;
    }

    // debounce switch
    bool control_switch_changed = control_switch_state.debounced_switch_position != switch_position;
    bool sufficient_time_elapsed = tnow_ms - control_switch_state.last_edge_time_ms > CONTROL_SWITCH_DEBOUNCE_TIME_MS;
    bool failsafe_disengaged = !failsafe.radio && failsafe.radio_counter == 0;

    if (control_switch_changed && sufficient_time_elapsed && failsafe_disengaged) {
        // set flight mode and simple mode setting
        if (set_mode((control_mode_t)flight_modes[switch_position].get(), MODE_REASON_TX_COMMAND)) {
            // play a tone
            if (control_switch_state.debounced_switch_position != -1) {
                // alert user to mode change failure (except if autopilot is just starting up)
                if (ap.initialised) {
                    AP_Notify::events.user_mode_change = 1;
                }
            }

            if (!check_if_auxsw_mode_used(AUXSW_SIMPLE_MODE) && !check_if_auxsw_mode_used(AUXSW_SUPERSIMPLE_MODE)) {
                // if none of the Aux Switches are set to Simple or Super Simple Mode then
                // set Simple Mode using stored parameters from EEPROM
                if (BIT_IS_SET(g.super_simple, switch_position)) {
                    set_simple_mode(2);
                } else {
                    set_simple_mode(BIT_IS_SET(g.simple_modes, switch_position));
                }
            }

        } else if (control_switch_state.last_switch_position != -1) {
            // alert user to mode change failure
            AP_Notify::events.user_mode_change_failed = 1;
        }

        // set the debounced switch position
        control_switch_state.debounced_switch_position = switch_position;
    }

    control_switch_state.last_switch_position = switch_position;
}

// check_if_auxsw_mode_used - Check to see if any of the Aux Switches are set to a given mode.
bool Copter::check_if_auxsw_mode_used(uint8_t auxsw_mode_check)
{
	//	modified by ZhangYong 20171115
    //bool ret = g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check 
     //           || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;
	//	modified end
#if PROJECTGKXN == ENABLED 

	bool ret = g.radio_tuning == auxsw_mode_check || g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check 
                || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;
#else
	bool ret = g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check 
                || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;
	//	modified end
#endif

    return ret;
}

// check_duplicate_auxsw - Check to see if any Aux Switch Functions are duplicated
bool Copter::check_duplicate_auxsw(void)
{
    uint8_t auxsw_option_counts[AUXSW_SWITCH_MAX] = {};
    auxsw_option_counts[g.ch7_option]++;
    auxsw_option_counts[g.ch8_option]++;
    auxsw_option_counts[g.ch9_option]++;
    auxsw_option_counts[g.ch10_option]++;
    auxsw_option_counts[g.ch11_option]++;
    auxsw_option_counts[g.ch12_option]++;

    for (uint8_t i=0; i<sizeof(auxsw_option_counts); i++) {
        if (i == AUXSW_DO_NOTHING) {
            continue;
        }
        if (auxsw_option_counts[i] > 1) {
            return true;
        }
    }
   return false;
}

//	added by ZhangYong 20171220
uint8_t Copter::readSwitch()
{
	int8_t switch_position;
    uint16_t rc5_in = RC_Channels::rc_channel(CH_5)->get_radio_in();
    if      (rc5_in < 1231) switch_position = 0;
    else if (rc5_in < 1361) switch_position = 1;
    else if (rc5_in < 1491) switch_position = 2;
    else if (rc5_in < 1621) switch_position = 3;
    else if (rc5_in < 1750) switch_position = 4;
    else switch_position = 5;

	return switch_position;
}
//	added end

void Copter::reset_control_switch()
{
    control_switch_state.last_switch_position = control_switch_state.debounced_switch_position = -1;
    read_control_switch();
}

// read_3pos_switch
uint8_t Copter::read_3pos_switch(uint8_t chan)
{
    uint16_t radio_in = RC_Channels::rc_channel(chan)->get_radio_in();
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// can't take reference to a bitfield member, thus a #define:
#define read_aux_switch(chan, flag, option)                           \
    do {                                                            \
        switch_position = read_3pos_switch(chan); \
        if (flag != switch_position) {                              \
            flag = switch_position;                                 \
            do_aux_switch_function(option, flag);                   \
        }                                                           \
    } while (false)

// read_aux_switches - checks aux switch positions and invokes configured actions
void Copter::read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

	//	added by ZhangYong 20171115
	read_aux_switch(CH_6, aux_con.CH6_flag, g.radio_tuning);
	//	added end

    read_aux_switch(CH_7, aux_con.CH7_flag, g.ch7_option);
    read_aux_switch(CH_8, aux_con.CH8_flag, g.ch8_option);
    read_aux_switch(CH_9, aux_con.CH9_flag, g.ch9_option);
    read_aux_switch(CH_10, aux_con.CH10_flag, g.ch10_option);
    read_aux_switch(CH_11, aux_con.CH11_flag, g.ch11_option);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    read_aux_switch(CH_12, aux_con.CH12_flag, g.ch12_option);
#endif
}

#undef read_aux_switch

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
void Copter::init_aux_switches()
{
    // set the CH7 ~ CH12 flags
	//	added by ZhangYong 20171115
	aux_con.CH6_flag = read_3pos_switch(CH_6);
	//	added end

	
    aux_con.CH7_flag = read_3pos_switch(CH_7);
    aux_con.CH8_flag = read_3pos_switch(CH_8);
    aux_con.CH10_flag = read_3pos_switch(CH_10);
    aux_con.CH11_flag = read_3pos_switch(CH_11);

    // ch9, ch12 only supported on some boards
    aux_con.CH9_flag = read_3pos_switch(CH_9);
    aux_con.CH12_flag = read_3pos_switch(CH_12);

    // initialise functions assigned to switches
    //	added by ZhangYong 20171115
    init_aux_switch_function(g.radio_tuning, aux_con.CH6_flag);
	//	added end
	
    init_aux_switch_function(g.ch7_option, aux_con.CH7_flag);
    init_aux_switch_function(g.ch8_option, aux_con.CH8_flag);
    init_aux_switch_function(g.ch10_option, aux_con.CH10_flag);
    init_aux_switch_function(g.ch11_option, aux_con.CH11_flag);

    // ch9, ch12 only supported on some boards
    init_aux_switch_function(g.ch9_option, aux_con.CH9_flag);
    init_aux_switch_function(g.ch12_option, aux_con.CH12_flag);
}

// init_aux_switch_function - initialize aux functions
void Copter::init_aux_switch_function(int8_t ch_option, uint8_t ch_flag)
{    
    // init channel options
    switch(ch_option) {
        case AUXSW_SIMPLE_MODE:
        case AUXSW_RANGEFINDER:
        case AUXSW_FENCE:
        case AUXSW_SUPERSIMPLE_MODE:
        case AUXSW_ACRO_TRAINER:
        case AUXSW_GRIPPER:
        case AUXSW_SPRAYER:
        case AUXSW_PARACHUTE_ENABLE:
        case AUXSW_PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUXSW_RETRACT_MOUNT:
        case AUXSW_MISSION_RESET:
        case AUXSW_ATTCON_FEEDFWD:
        case AUXSW_ATTCON_ACCEL_LIM:
        case AUXSW_MOTOR_ESTOP:
        case AUXSW_MOTOR_INTERLOCK:
        case AUXSW_AVOID_ADSB:
        case AUXSW_PRECISION_LOITER:
        case AUXSW_AVOID_PROXIMITY:
		case AUXSW_DIR_AB:
            do_aux_switch_function(ch_option, ch_flag);
            break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
void Copter::do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{

    switch(ch_function) 
	{
        case AUXSW_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(FLIP, MODE_REASON_TX_COMMAND);
            }
            break;

        case AUXSW_SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUXSW_SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            set_simple_mode(ch_flag);
            break;

        case AUXSW_RTL:
            if (ch_flag == AUX_SWITCH_HIGH) {
                // engage RTL (if not possible we remain in current flight mode)
                set_mode(RTL, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in RTL
                if (control_mode == RTL) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_SAVE_TRIM:
            if ((ch_flag == AUX_SWITCH_HIGH) && (control_mode <= ACRO) && (channel_throttle->get_control_in() == 0)) {
                save_trim();
            }
            break;

        case AUXSW_SAVE_WP:
            // save waypoint when switch is brought high
            if (ch_flag == AUX_SWITCH_HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (control_mode == AUTO || !motors->armed()) {
                    return;
                }

                // do not allow saving the first waypoint with zero throttle
                if ((mission.num_commands() == 0) && (channel_throttle->get_control_in() == 0)) {
                    return;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if (mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.options = 0;
                    cmd.p1 = 0;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = MAX(current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if (mission.add_cmd(cmd)) {
                        // log event
                        Log_Write_Event(DATA_SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = current_loc;

                // if throttle is above zero, create waypoint command
                if (channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                } else {
                    // with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if (mission.add_cmd(cmd)) {
                    // log event
                    Log_Write_Event(DATA_SAVEWP_ADD_WP);
                }
            }
            break;

        case AUXSW_CAMERA_TRIGGER:
#if CAMERA == ENABLED
            if (ch_flag == AUX_SWITCH_HIGH) {
                camera.take_picture();
            }
#endif
            break;

        case AUXSW_RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == AUX_SWITCH_HIGH) && rangefinder.has_orientation(ROTATION_PITCH_270)) {
                rangefinder_state.enabled = true;
            } else {
                rangefinder_state.enabled = false;
            }
#endif
            break;

        case AUXSW_FENCE:
#if AC_FENCE == ENABLED
            // enable or disable the fence
            if (ch_flag == AUX_SWITCH_HIGH) {
                fence.enable(true);
                Log_Write_Event(DATA_FENCE_ENABLE);
            } else {
                fence.enable(false);
                Log_Write_Event(DATA_FENCE_DISABLE);
            }
#endif
            break;

        case AUXSW_ACRO_TRAINER:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g.acro_trainer = ACRO_TRAINER_DISABLED;
                    Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    g.acro_trainer = ACRO_TRAINER_LEVELING;
                    Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case AUX_SWITCH_HIGH:
                    g.acro_trainer = ACRO_TRAINER_LIMITED;
                    Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
            break;

        case AUXSW_GRIPPER:
#if GRIPPER_ENABLED == ENABLED
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g2.gripper.release();
                    Log_Write_Event(DATA_GRIPPER_RELEASE);
                    break;
                case AUX_SWITCH_HIGH:
                    g2.gripper.grab();
                    Log_Write_Event(DATA_GRIPPER_GRAB);
                    break;
            }
#endif
            break;

        case AUXSW_SPRAYER:
#if SPRAYER == ENABLED
            sprayer.run(ch_flag == AUX_SWITCH_HIGH);
            // if we are disarmed the pilot must want to test the pump
            //if(1 == sprayer.get_vpvs_enable())
            //{
            	sprayer.test_pump((ch_flag == AUX_SWITCH_HIGH) && !motors->armed());
            //}
			//else
			//{
			//	sprayer.test_pump(ch_flag == AUX_SWITCH_HIGH);
			//}
            
#endif
            break;

        case AUXSW_AUTO:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(AUTO, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in AUTO
                if (control_mode == AUTO) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_AUTOTUNE:
#if AUTOTUNE_ENABLED == ENABLED
            // turn on auto tuner
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                case AUX_SWITCH_MIDDLE:
                    // restore flight mode based on flight mode switch position
                    if (control_mode == AUTOTUNE) {
                        reset_control_switch();
                    }
                    break;
                case AUX_SWITCH_HIGH:
                    // start an autotuning session
                    set_mode(AUTOTUNE, MODE_REASON_TX_COMMAND);
                    break;
            }
#endif
            break;

        case AUXSW_LAND:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(LAND, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in LAND
                if (control_mode == LAND) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_PARACHUTE_ENABLE:
#if PARACHUTE == ENABLED
            // Parachute enable/disable
            parachute.enabled(ch_flag == AUX_SWITCH_HIGH);
#endif
            break;

        case AUXSW_PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (ch_flag == AUX_SWITCH_HIGH) {
                parachute_manual_release();
            }
#endif
            break;

        case AUXSW_PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case AUX_SWITCH_LOW:
                    parachute.enabled(false);
                    Log_Write_Event(DATA_PARACHUTE_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    parachute.enabled(true);
                    Log_Write_Event(DATA_PARACHUTE_ENABLED);
                    break;
                case AUX_SWITCH_HIGH:
                    parachute.enabled(true);
                    parachute_manual_release();
                    break;
            }
#endif
            break;

        case AUXSW_MISSION_RESET:
            if (ch_flag == AUX_SWITCH_HIGH) {
                mission.reset();
            }
            break;

        case AUXSW_ATTCON_FEEDFWD:
            // enable or disable feed forward
            attitude_control->bf_feedforward(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            attitude_control->accel_limiting(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_RETRACT_MOUNT:
#if MOUNT == ENABLE
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
                    break;
                case AUX_SWITCH_LOW:
                    camera_mount.set_mode_to_default();
                    break;
            }
#endif
            break;

        case AUXSW_RELAY:
            ServoRelayEvents.do_set_relay(0, ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_RELAY2:
            ServoRelayEvents.do_set_relay(1, ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_RELAY3:
            ServoRelayEvents.do_set_relay(2, ch_flag == AUX_SWITCH_HIGH);
            break;

	   case AUXSW_RELAY4:
            ServoRelayEvents.do_set_relay(3, ch_flag == AUX_SWITCH_HIGH);
            break;

       case AUXSW_LANDING_GEAR:
#if LG_ENABLE == ENABLED 	   	
            switch (ch_flag) {
                case AUX_SWITCH_LOW:
                    //landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
					landinggear.set_position(AP_LandingGear::LandingGear_Deploy, \
															inertial_nav.get_altitude(), \
															motors->armed());
                    break;
                case AUX_SWITCH_HIGH:
                    //landinggear.set_position(AP_LandingGear::LandingGear_Retract);
					landinggear.set_position(AP_LandingGear::LandingGear_Retract, \
															inertial_nav.get_altitude(), \
															motors->armed());
                    break;
            }
#endif			
            break;

        case AUXSW_LOST_COPTER_SOUND:
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    AP_Notify::flags.vehicle_lost = true;
                    break;
                case AUX_SWITCH_LOW:
                    AP_Notify::flags.vehicle_lost = false;
                    break;
            }
            break;

        case AUXSW_MOTOR_ESTOP:
            // Turn on Emergency Stop logic when channel is high
            set_motor_emergency_stop(ch_flag == AUX_SWITCH_HIGH);
            break;

        case AUXSW_MOTOR_INTERLOCK:
            // Turn on when above LOW, because channel will also be used for speed
            // control signal in tradheli
            ap.motor_interlock_switch = (ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUXSW_BRAKE:
            // brake flight mode
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(BRAKE, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in BRAKE
                if (control_mode == BRAKE) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_THROW:
            // throw flight mode
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(THROW, MODE_REASON_TX_COMMAND);
            } else {
                // return to flight mode switch's flight mode if we are currently in throw mode
                if (control_mode == THROW) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_AVOID_ADSB:
            // enable or disable AP_Avoidance
            if (ch_flag == AUX_SWITCH_HIGH) {
                avoidance_adsb.enable();
                Log_Write_Event(DATA_AVOIDANCE_ADSB_ENABLE);
            } else {
                avoidance_adsb.disable();
                Log_Write_Event(DATA_AVOIDANCE_ADSB_DISABLE);
            }
            break;

        case AUXSW_PRECISION_LOITER:
#if PRECISION_LANDING == ENABLED
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    set_precision_loiter_enabled(true);
                    break;
                case AUX_SWITCH_LOW:
                    set_precision_loiter_enabled(false);
                    break;
            }
#endif
            break;

        case AUXSW_AVOID_PROXIMITY:
#if PROXIMITY_ENABLED == ENABLED && AC_AVOID_ENABLED == ENABLED
            switch (ch_flag) {
                case AUX_SWITCH_HIGH:
                    avoid.proximity_avoidance_enable(true);
                    Log_Write_Event(DATA_AVOIDANCE_PROXIMITY_ENABLE);
                    break;
                case AUX_SWITCH_LOW:
                    avoid.proximity_avoidance_enable(false);
                    Log_Write_Event(DATA_AVOIDANCE_PROXIMITY_DISABLE);
                    break;
            }
#endif
            break;
        case AUXSW_ARMDISARM:
            // arm or disarm the vehicle
            switch (ch_flag) {
            case AUX_SWITCH_HIGH:
                init_arm_motors(false);
                break;
            case AUX_SWITCH_LOW:
                init_disarm_motors();
                break;
            }
            break;


		case AUXSW_FS_PLD:
			if (ch_flag == AUX_SWITCH_HIGH) 
			{
//				printf("AUXSW_FS_PLD ON\n");
				set_failsafe_payload(FAILSAFE_PLD_TYPE_FM, true);
			}
			else if(ch_flag == AUX_SWITCH_LOW)
			{
//				printf("AUXSW_FS_PLD OFF\n");
				set_failsafe_payload(FAILSAFE_PLD_TYPE_FM, false);
			}
			break;

		case AUXSW_AUTO_HEIGHT:
			
			if (ch_flag == AUX_SWITCH_HIGH) 
			{
				height_replace_switch = 1;
				//	modified by ZhangYong for current_loc.alt differs from inertial_nav.get_altitude()
				height_replace_alt_home = inertial_nav.get_altitude();
		
				if(rangefinder_alt_ok())
				{
					height_replace_alt_terrain = rangefinder_state.alt_cm;
			 	}
					
				set_mode(AUTO, MODE_REASON_TX_COMMAND);
			}
			else if(ch_flag == AUX_SWITCH_LOW)
			{
				height_replace_switch = 0;
				height_replace_alt_home = 0;
				height_replace_alt_terrain = 0;
					
				if (control_mode == AUTO) 
				{
					reset_control_switch();
				}
			}
			break;
			
#if PROJECTFB == ENABLE
		case AUXSW_SWITCH_TURBINE:
		if (ch_flag == AUX_SWITCH_HIGH) 
		{
//			turbine = 2;

			motors->set_turbine(2);
			
//			printf("HIGH %d\n", turbine);		
		}
		else if(ch_flag == AUX_SWITCH_MIDDLE)
		{

//			turbine = 1;

			motors->set_turbine(1);
			
//			printf("MID %d\n", turbine);	
		}
		else
		{
//			turbine = 0;

			motors->set_turbine(0);
			
//			printf("LOW %d\n", turbine);	
			
		}
		break;	
#endif

#if CHARGINGSTATION == ENABLED			
        //baiyang added in 20170414
        case AUXSW_FLIGHT:
            if (ch_flag == AUX_SWITCH_HIGH) 
                chargingStation.fly();
            else if(ch_flag == AUX_SWITCH_LOW)
            	chargingStation.landed();
            else
            	chargingStation.reset_flight_status();
            break;
        case AUXSW_BLASTOFF:
            if (ch_flag == AUX_SWITCH_LOW)
            		chargingStation.set_blastoff_flag();
            else if(ch_flag == AUX_SWITCH_HIGH)
            		chargingStation.start_communication();
            else if(ch_flag == AUX_SWITCH_MIDDLE)
            		chargingStation.reset();
            break;
        //added end

        //baiyang added in 20170612
        case AUXSW_DO_TAKEOFF:
            if (ch_flag == AUX_SWITCH_HIGH) 
            	chargingStation.do_takeoff();
            break;
        //added end
#endif

#if ABMODE == ENABLED
		case AUXSW_RECORD_AB:
            switch (ch_flag) 
			{
	            case AUX_SWITCH_HIGH:
	                rf_abmode.abmode_set_pos_b();
	                break;
	            case AUX_SWITCH_LOW:
	                rf_abmode.abmode_set_pos_a();
					//	added by zhangyong for spraying operation 20180823
					//	sprayer.run(true);
					//	sprayer.test_pump(!motors->armed());
					//	added end
					
	                break;
            }
            break;
#endif
	
#if ABMODE == ENABLED
		case AUXSW_MODE_DIR_AB:
            switch (ch_flag) 
			{
	            case AUX_SWITCH_HIGH:
	                set_mode(ABMODE_RF, MODE_REASON_TX_COMMAND);
	                break;
	            case AUX_SWITCH_LOW:
	                rf_abmode.invert_direction(SEMIAUTO);
	                break;
	            }
            break;
#endif
	
#if ABMODE == ENABLED
		case AUXSW_DIR_AB:
            switch (ch_flag) 
			{
	            case AUX_SWITCH_HIGH:
	                rf_abmode.invert_direction(MANUAL,ISLIFT);
	                break;
	            case AUX_SWITCH_LOW:
	                rf_abmode.invert_direction(MANUAL,ISRIGHT);
	                break;
	        }
            break;
#endif

#if ABMODE == ENABLED
		case AUXSW_SET_ABMODE:
            switch (ch_flag) 
			{
	            case AUX_SWITCH_HIGH:
	                set_mode(ABMODE_RF, MODE_REASON_TX_COMMAND);
					break;
			    case AUX_SWITCH_MIDDLE:
			    case AUX_SWITCH_LOW:
					if (control_mode == ABMODE_RF) {
                    	reset_control_switch();
                	}
	                break;
	        }
            break;
#endif
		default:
			break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
void Copter::save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)channel_roll->get_control_in()/100.0f);
    float pitch_trim = ToRad((float)channel_pitch->get_control_in()/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    Log_Write_Event(DATA_SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void Copter::auto_trim()
{
    if (auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)channel_roll->get_control_in() / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)channel_pitch->get_control_in() / 4000.0f);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if (auto_trim_counter == 0) {
            AP_Notify::flags.save_trim = false;
        }
    }
}

