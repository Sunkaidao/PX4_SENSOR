/// @file	AC_Sprayer.h
/// @brief	Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        - set CH7_OPT or CH8_OPT parameter to 15 to allow turning the sprayer on/off from one of these channels
        - set RC10_FUNCTION to 22 to enable the servo output controlling the pump speed on servo-out 10
        - set RC11_FUNCTION to 23 to enable the servo output controlling the spinner on servo-out 11
        - ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        - set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        - set the SPRAY_PUMP_RATE to the value the pump servo should move to when the vehicle is travelling 1m/s expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        - set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        - set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/
#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
//	added by ZhangYong 20170717
//#include <../././ArduCopter/defines.h>


//	added end
#define AC_SPRAYER_DEFAULT_ENABLE			0
#define AC_SPRAYER_DEFAULT_PUMP_RATE        10.0f   ///< default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_PUMP_MIN         0       ///< default minimum pump speed expressed as a percentage from 0 to 100
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    ///< default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     ///< we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     ///< delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump

//	modified by ZhangYong to meet custom demand 20170717
//#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   300    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump
//	modified end


#define AC_SPRAYER_DEFAULT_SPEED_OFF_MAX    120     // we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_UNSPRAY_DISTANCE	550
#define AC_SPRAYER_DEFAULT_VPVS_ENABLE		1


/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class AC_Sprayer {

public:

    /// Constructor
    AC_Sprayer(const AP_InertialNav* inav);

    /// run - allow or disallow spraying to occur
    void run(bool true_false);


	//	modified by zhangYong 20170703
    /// running - returns true if spraying is currently permitted
    //bool running() const { return _flags.running; }

	
    /// spraying - returns true if spraying is actually happening
    //bool spraying() const { return _flags.spraying; }
	uint8_t get_running() const { return _flags.running; }

	uint8_t get_spraying() {return _flags.spraying; }

	//	modified end


	//	added by zhangYong 20170703
	uint8_t get_testing() {return _flags.testing; }

	int8_t get_enabled() {return _enabled.get();}
	//	added end

    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    //	modified by ZhangYong 20170717
    void test_pump(bool true_false);
	//void test_pump(bool true_false) { _flags.testing = true_false; }
	//	modified end

	//	added by zhangyong 20170717
	int8_t get_vpvs_enable() {return _vpvs_enable.get(); }
	//	added end

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

	void set_short_edge(uint32_t wp_dist);

	uint8_t get_short_edge() {return _flags.short_edge; }

    /// update - adjusts servo positions based on speed and requested quantity
    void update(int8_t ctl_mode, uint32_t wp_dist);

	uint16_t get_actual_pump_rate() {return actual_pump_rate;}

	uint32_t get_speed_over_min_time() {return _speed_over_min_time;}

	uint32_t get_speed_under_min_time() {return _speed_under_min_time;}

	//baiyang added in 20180119
    //units:cm
    int16_t get_unspray_dist() {return _unspray_dist.get();}
    //added end

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_InertialNav* const _inav;      ///< pointers to other objects we depend upon

    // parameters
    AP_Int8         _enabled;               ///< top level enable/disable control
    AP_Float        _pump_pct_1ms;          ///< desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Int8         _pump_min_pct;          ///< minimum pump rate (expressed as a percentage from 0 to 100)
    AP_Int16        _spinner_pwm;           ///< pwm rate of spinner
    AP_Float        _speed_min;             ///< minimum speed in cm/s above which the sprayer will be started
	//	added by zhangyong 20161128
	AP_Float		_speed_max; 			// maximum speed in cm/s below which the sprayer will be stoped
	AP_Int16		_unspray_dist;
	AP_Int8 		_vpvs_enable;				// top level enable/disable control
	//	added end

	int16_t 		actual_pump_rate;

    /// flag bitmask
    struct sprayer_flags_type {
        uint8_t spraying    : 1;            ///< 1 if we are currently spraying
        uint8_t testing     : 1;            ///< 1 if we are testing the sprayer and should output a minimum value
        uint8_t running     : 1;            ///< 1 if we are permitted to run sprayer
        uint8_t short_edge	: 1;			///< 1 if we are ing short edge, then spray should not allowed
    } _flags;

    // internal variables
    uint32_t        _speed_over_min_time;   ///< time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  ///< time at which we fell below speed minimum

	

    void stop_spraying();
};
