/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Copter.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_aux_switches,     10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_proximity,     100,     50),
    SCHED_TASK(update_beacon,        400,     50),
    SCHED_TASK(update_visual_odom,   400,     50),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(run_nav_updates,       50,    100),
    SCHED_TASK(update_throttle_hover,100,     90),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(compass_accumulate,   100,    100),
    SCHED_TASK(barometer_accumulate,  50,     90),
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
    SCHED_TASK(fourhundred_hz_logging,400,    50),
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(gpsglitch_check,       10,     50),
#if LG_ENABLE == ENABLED 
    SCHED_TASK(landinggear_update,    10,     75),
#endif
	SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
    SCHED_TASK(update_trigger,        50,     75),
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(perf_update,           0.1,    75),
    SCHED_TASK(read_receiver_rssi,    10,     75),
    SCHED_TASK(rpm_update,            10,    200),
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK(gripper_update,        10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
#if BUTTON_ENABLED == ENABLED
    SCHED_TASK(button_update,          5,    100),
#endif
    SCHED_TASK(stats_update,           1,    100),
};


void Copter::setup() 
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();
}

/*
  try to accumulate a baro reading
 */
void Copter::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Copter::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

/*
  update AP_Stats
 */
void Copter::stats_update(void)
{
    g2.stats.update();
}

void Copter::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}


// Main loop - 400hz
void Copter::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();
    
    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
}

// update_mount - update camera mount position
// should be run at 50hz
void Copter::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

// update camera trigger
void Copter::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.update_trigger();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }


}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Copter::fourhundred_hz_logging()
{
//	modifified by ZhangYong to store data as less as possible
//	if (should_log(MASK_LOG_ATTITUDE_FAST)) {
//        Log_Write_Attitude();
//    }
//	modifified end
    
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{

    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {

        DataFlash.Log_Write_RCIN();
        if (rssi.enabled()) {
            DataFlash.Log_Write_RSSI(rssi);
        }

		//	added by ZhangYong for land detector log 20170703
//		land_dt_log.override = hal.rcin->get_override_valid();
//		land_dt_log.active =  failsafe.rc_override_active;
//		DataFlash.Log_Write_LD(&land_dt_log);
		//	added end
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
		//	modified by ZhangYong 20170515
//        attitude_control->control_monitor_log();
		//	modified end
        Log_Write_Proximity();
        Log_Write_Beacon();
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif


#if BCBMONITOR == ENABLED
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBMonitor,0)))
	{
		printf("%8d, A:%3d, B:%3d, B1V:%4.1f, C:%4.1f, P:%6.1f, B2V:%4.1f, C:%4.1f, P:%6.1f\n", \
																				(AP_HAL::millis()), \
																				(bcbmonitor.voltageA() / 100), \
																				(bcbmonitor.voltageB() / 100), \
																				battery.voltage(), \
																				battery.current_amps(), \
																				(battery.voltage() * battery.current_amps()), \
																				battery.voltage(1), \
																				battery.current_amps(1), \
																				(battery.voltage(1) * battery.current_amps(1)));
	}	
#endif

//	added by ZhangYong 20170809
//	printf("%d: %4.2f\n", battery.num_instances(), battery.voltage(1));
//	added end

	

#if BCBPMBUS == ENABLE
	update_bcbpmbus();
#endif


}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs().send_message(MSG_SERVO_OUTPUT_RAW);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
	//	modified by ZhangYong 20170915
	//if (should_log(MASK_LOG_ATTITUDE_FAST)) {
    //    Log_Write_EKF_POS();
    //}

    // log IMU data if we're not already logging at the higher rate
    //if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
    //    DataFlash.Log_Write_IMU(ins);
    //}
	//	modified end
    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif

#if PRECISION_LANDING == ENABLED
    // log output
    Log_Write_Precland();
#endif
}

void Copter::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED

	//	update vpvs timely
	//	added by ZhangYong 20171011
	/*if(sprayer.get_enabled())
	{
		if(sprayer.get_running())
		{
			sprayer.test_pump(!motors->armed());
		}
	}*/
	//	added end



    sprayer.update(control_mode, wp_distance);


#if PROJECTGKXN == ENABLED 
	Log_Write_Sprayer(sprayer, wp_distance, flowmeter.get_warning(), flowmeter.get_packet_cnt(),flowmeter.get_volume(),flowmeter.get_high(), newbroadcast.get_view_flight_area());
#else
	Log_Write_Sprayer(sprayer, wp_distance, 1, 0);
#endif
#endif



    update_events();

	//	//	shielded by ZhangYong to meet 20170627 20170705
    // update ch6 in flight tuning
    //	tuning();
    //	shielded end

#if PROJECTGKXN == ENABLED
	flowmeter.update(serial_manager);


	/*	to do zhangyong 20160920
	//	printf("%d, %d, %d, %d\n", !ap.usb_connected, \
	//								failsafe.payload, \
	//								flowmeter.exhausted(), \
	//								motors.armed());
	*/
	//	to set payload failsafe true
	if(\
		(!ap.usb_connected) && \
		(0 == get_failsafe_payload(FAILSAFE_PLD_TYPE_FM)) && \
		flowmeter.exhausted() && \
		motors->armed()\
		)
	{
		//printf("here!\n");
		set_failsafe_payload(FAILSAFE_PLD_TYPE_FM, true);
	//		sprayer.enable(false);

		sprayer.run(false);
		sprayer.test_pump(false);

	}

	if(\
		(!ap.usb_connected) && \
		(0 == motors->armed()) && \
		(1 == get_failsafe_payload(FAILSAFE_PLD_TYPE_FM))\
	)
	{
		set_failsafe_payload(FAILSAFE_PLD_TYPE_FM, false);
	
	}

	
#endif	


	Log_Write_land_detector(&log_land_detector);

//#if BCBMONITOR == ENABLE
//bcbmonitor.read();
//#endif

//#if BCBPMBUS == ENABLE
//	update_bcbpmbus();
//#endif
	
}



// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
	static uint8_t lcl_uint8 = 0;

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    check_usb_mux();

    // log terrain data
    terrain_logging();

    adsb.set_is_flying(!ap.land_complete);
    
    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();


		//		added by ZhangYong 20161110
#if FXTX_AUTH == ENABLED
	//	modidied by zhangyong to make sure gps fix is durable
	if((false == curr_gps_week_ms.time_week_settled) &&\
		(gps.status() >=	AP_GPS::GPS_OK_FIX_3D) && \
		(gps.get_hdop() <= g.gps_hdop_good) && \
		(gps.num_sats() >= ahrs.get_gps_minsats()))
	//	modified end			
	{
		//	added for debug 20161110
		
		//	added end
		curr_gps_week_ms.time_week_settled = true;
		curr_gps_week_ms.time_week = gps.time_week();
		curr_gps_week_ms.time_week_ms = gps.time_week_ms();

		//	added by zhangyong for debug
//		printf("week %d\n", gps.time_week());
//		printf("week_ms %d\n", gps.time_week_ms());

	
	}	

//	printf("1. auth_state_ms = %d\n", auth_state_ms);
//	printf("1. auth_state_timeout_cnt = %d\n", auth_state_timeout_cnt);

	//	added by zhangyong to make clear the auth state 20180612
	if(auth_state_up_auth == auth_state_ms)
	{	
		
		
		if(auth_state_timeout_cnt >= 2)
		{
			auth_state_timeout_cnt = 0;
			auth_result_ms = auth_result_failed;
			auth_state_ms = auth_state_initialize;
			AP_Notify::events.tune_next = auth_result_ms + 1;
		}

		auth_state_timeout_cnt++;
		
	}

//	printf("2. auth_state_ms = %d\n", auth_state_ms);
//	printf("2. auth_state_timeout_cnt = %d\n", auth_state_timeout_cnt);
	//	

//	printf("A. %d, %d, %d, &%d\n", gps.status(), curr_gps_week_ms.time_week, gps.time_week_ms(), &curr_gps_week_ms);
#endif

#if PROJECTGKXN == ENABLED
/*	if(motors->armed())
	{
		local_flight_time_sec++;
		
	}//	added end
*/#endif


//		added by ZhangYong
#if PJTPASSOSD == ENABLED
	
	PassOSD_data_status lcl_data, *lcl_data_ptr;
	lcl_data_ptr = &lcl_data;

	//	fill the contend of data
	lcl_data_ptr->data.voltage_battery = battery.voltage();
	lcl_data_ptr->data.current_total_mah = battery.current_total_mah();
	lcl_data_ptr->data.fix_type = gps.status();
	lcl_data_ptr->data.num_sats = gps.num_sats();
	lcl_data_ptr->data.control_mode = control_mode;
	lcl_data_ptr->data.velocity = inertial_nav.get_velocity_xy();
	lcl_data_ptr->data.climbrate = inertial_nav.get_velocity_z();
	lcl_data_ptr->data.altitude = inertial_nav.get_altitude();
	lcl_data_ptr->data.flight_time_sec = 0x09;
	lcl_data_ptr->data.yaw = ahrs.yaw;
	
/*	lcl_data_ptr->data.voltage_battery = 1;
	lcl_data_ptr->data.current_total_mah = 2;
	lcl_data_ptr->data.fix_type = 3;
	lcl_data_ptr->data.num_sats = 4;
	lcl_data_ptr->data.control_mode = 5;
	lcl_data_ptr->data.velocity = 6;
	lcl_data_ptr->data.climbrate = 7;
	lcl_data_ptr->data.altitude = 8;
	lcl_data_ptr->data.flight_time_sec = 9;
	lcl_data_ptr->data.yaw = 10;
*/	


	passosd.send_message(lcl_data_ptr);
#endif
//		added end

	/*
	//	added by ZhangYong 20170915 for duration test
//	printf("%d\n", duration_cnt++);

	//	added by ZhangYong 20171013 debug
	//printf("one_hz_loop %d %4.2f\n", channel_throttle->get_control_in(), motors->get_spin_arm());
	//	added end
	*/

	/*printf("one_hz_loop %d %4.2f\n", channel_throttle->get_control_in(), \
									get_pilot_desired_climb_rate(channel_throttle->get_rc_control_in()));
	*/
	
//	printf("rp %d, cp %d gcs %d\n", copter.ap.rc_receiver_present, copter.fs_mk.control_present, copter.fs_mk.gcs_control);

//	printf("one_hz_loop rc_valid %d\n",  hal.rcin->get_rc_valid());

	//	added by ZhangYong for loging purpose
	
	
	DataFlash.Log_Write_Mtr(motors->get_mtr_log());

	DataFlash.Log_Write_Control(failsafe.rc_override_active, \
								hal.rcin->get_override_valid(), \
								hal.rcin->get_rc_valid(), \
								hal.rcin->get_rc_rc3_radio_in());

	//	added by ZhangYong 20180116 to log GK proximity
	//DataFlash.Log_Write_GKProx(AP_Proximity &proximity);	
	//	adde end

	//	added by ZhangYong for test
	//	...to do
	
	/*Location homeLocation;

	homeLocation.lat = 0;
	homeLocation.lng = 0;
	homeLocation.alt = 0;
	
	printf("home, lat:%d, lng:%d, alt:%d\n", ahrs.get_home().lat, ahrs.get_home().lng, ahrs.get_home().alt);
	printf("origal, %d, lat:%d, lng:%d, alt:%d\n", ahrs.get_origin(homeLocation), homeLocation.lat, homeLocation.lng, homeLocation.alt);
	*/
	//	added end


	//printf("alt %4.2f baro %d\n", inertial_nav.get_altitude(), copter.baro_alt);

//	gcs().send_text(MAV_SEVERITY_CRITICAL, "PMBUS develop");

//	printf("one_hz_loop: ap.pre_arm_check %d, motor->armed() %d\n", ap.pre_arm_check, motors->armed());

	
	//printf("streamRates[para] = %d\n", gcs().chan(1).get_streamRates(8));

	//	added by zhangyong 20180713
	//	printf("compass_checks %d\n", compass.get_external(compass.get_primary()));
		//	added end


	//	added by zhangyong 20180728
	
	//	added end

///	printf("%d\n", lcl_uint8);
#if LOGGING_ENABLED == ENABLED

//	printf("%d %d %d %d\n", lcl_uint8, motors->armed(), DataFlash.get_num_logs(), DataFlash.get_num_logs_max());

	if((29 == (lcl_uint8++) % 30) && \
		(!motors->armed()) && \
		(DataFlash.get_num_logs() >= DataFlash.get_num_logs_max() - 10)\
		)
	{
//	 	printf("A\n");
		gcs().send_text(MAV_SEVERITY_WARNING,"Warning: log numbers exceed max");
	}
#endif
//sunkaidao added in 180829
#if AIRCHECK== ENABLED
  gassensor.update(serial_manager,DataFlash);
#endif


}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }
    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

#if CAMERA == ENABLED
        camera.update(control_mode);
#endif
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and rangefinder altitude at 10hz
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN_CALLBACKS(&copter);
