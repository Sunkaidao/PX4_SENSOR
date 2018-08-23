#include <AP_HAL/AP_HAL.h>
#include "AC_Sprayer.h"

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 0, AC_Sprayer, _enabled, AC_SPRAYER_DEFAULT_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

	// @Param: PSTOF_DELAY
    // @DisplayName: Pump shut off delay
    // @Description: 
    // @Units: ms millisecond
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("VPVS",   5, AC_Sprayer, _vpvs_enable, AC_SPRAYER_DEFAULT_VPVS_ENABLE),

	
	// @Param: PSTOF_DELAY
	// @DisplayName: Pump shut off delay
	// @Description: 
	// @Units: ms millisecond
	// @Range: 0 1000
	// @User: Standard
	AP_GROUPINFO("SPEED_MAX",   6, AC_Sprayer, _speed_max, AC_SPRAYER_DEFAULT_SPEED_OFF_MAX),

	
	// @Param: UNSPY_DIST
	// @DisplayName: unsprayer distance
	// @Description: 
	// @Units: cm
	// @Range: 0 5000
	// @User: Standard
	AP_GROUPINFO("UNSPY_DIST",	7, AC_Sprayer, _unspray_dist, AC_SPRAYER_DEFAULT_UNSPRAY_DISTANCE),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(const AP_InertialNav* inav) :
    _inav(inav),
    _speed_over_min_time(0),
    _speed_under_min_time(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }

	// check for silly parameter values
    if (_pump_min_pct < 0 || _pump_min_pct > 100) {
        _pump_min_pct.set_and_save(10);
    }
	
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

	
	void AC_Sprayer::test_pump(bool true_false) 
	{
		if(true_false == _flags.testing)
			return;
		
		_flags.testing = true_false; 
	
	}	

void AC_Sprayer::run(const bool true_false)
{
    // return immediately if no change
    if (true_false == _flags.running) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = true_false && _enabled;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);

    _flags.spraying = false;
}


void AC_Sprayer::set_short_edge(uint32_t wp_dist)
{
	uint32_t unspray_distance = _unspray_dist.get();
	
	if(wp_dist > unspray_distance)
		_flags.short_edge = false;
	else
		_flags.short_edge = true;

}


/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void
AC_Sprayer::update(int8_t ctl_mode, uint32_t wp_dist)
{
	//	added by ZhangYong 20170717
	uint32_t unspray_distance;
	float lcl_pump_rate;
	//int16_t pump_pct;
	actual_pump_rate = 0;
	//	added end


    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !get_running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    const Vector3f &velocity = _inav->get_velocity();
    float ground_speed = norm(velocity.x,velocity.y);

    // get the current time
    const uint32_t now = AP_HAL::millis();

    bool should_be_spraying = _flags.spraying;

	//	added by ZhangYong 20170717	
	unspray_distance = _unspray_dist.get();

	if(unspray_distance > 5000)
	{
		unspray_distance = AC_SPRAYER_DEFAULT_UNSPRAY_DISTANCE;
	}
	//
	
	// check our speed vs the minimum
	//	modfied by ZhangYong 20170717
	/*
    if (ground_speed >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    should_be_spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    }else{
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }*/
    //	modified by ZhangYong
    if(1 == _vpvs_enable)
    {

	if(false == should_be_spraying)
	{
    	if (ground_speed >= _speed_min) 
		{
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) 
			{
                _speed_over_min_time = now;
            }
			else
			{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) 
				{
					//	AUTO = 3
					//	ABMODE_RF = 21

					if(3 == ctl_mode || 21 == ctl_mode)					
					{
						if(wp_dist != 0)
						{
							if(wp_dist > unspray_distance)
							{
								should_be_spraying = true;
                    			_speed_over_min_time = 0;
						
							}
						}
					}
					else
					{
                    	should_be_spraying = true;
                    	_speed_over_min_time = 0;
					}
					
                }
            }

			
    	}	
        // reset the speed under timer

//		printf("1. %d %d vs %d\n", should_be_spraying, now, _speed_over_min_time);
		
        _speed_under_min_time = 0;
    }
	else
	{
        // we are under the min speed.  If we are spraying
        if (ground_speed <= _speed_max) 
		{
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }
			else
			{
				//	add by Zhangyong to satisfy the custom GKXN for equal unsprayed area 20161128
				//if(_pstof_delay <= 0)
				//{	
				//	pump_shutoff_delay = 0;
				//}
				//e/lse if(_pstof_delay >= 10000)
				//{
				//	pump_shutoff_delay = 10000;
				
//}

				//pump_shutoff_delay = _pstof_delay;
				//	added end
			
                // check if we've been over the speed long enough to engage the sprayer
                //	modified by Zhangyong g to satisfy the custom GKXN for equal unsprayed area 20161128
                //if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) 
				//	modified ended
				if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY)
				{
					///	auto 3
					//	ABMODE_RF = 21
					//if((ctl_mode == 3) && (1 =  _vpvs_enable))

					if(3 == ctl_mode || 21 == ctl_mode)					
					{	
						if(0 != wp_dist)
						{
							if(wp_dist < unspray_distance)
							{
								should_be_spraying = false;
                    			_speed_under_min_time = 0;
							
							}
						
						}
					
					}
					else
					{
                    	should_be_spraying = false;
                    	_speed_under_min_time = 0;
					}
					
               	}
            }
        }

//		printf("2. %d %d vs %d\n", should_be_spraying, now, _speed_under_min_time);
		
        // reset the speed over timer
        _speed_over_min_time = 0;
    }
    	}

	else
	{
		if(_flags.running)
		{
			should_be_spraying = true;
		}
		else
			should_be_spraying = false;
	}

    // if testing pump output speed as if traveling at 1m/s
    if (_flags.testing) {
       
        should_be_spraying = true;
    }

//	printf("3. %d %d vs %d\n", should_be_spraying, _flags.testing, ground_speed);
	

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
		//	modified by ZhangYong 20170717
		/*
        float pos = ground_speed * _pump_pct_1ms;
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range
        */
        if(_flags.testing)
        {
        	lcl_pump_rate = 10000;
		}
		else
		{
			if(1 == _vpvs_enable)
			{
				
				lcl_pump_rate = float(_pump_min_pct.get()) * 100 + (ground_speed * _pump_pct_1ms.get() / 10 );
				//printf("sprayer, %4.2f %4.2f\n", ground_speed, _pump_pct_1ms.get());
			}else
				lcl_pump_rate = 100 *_pump_pct_1ms.get();
				
		}

		constrain_float(lcl_pump_rate, -32767, 32767);		
		
		
//		pump_pct = 

		actual_pump_rate = MIN((int16_t)lcl_pump_rate, 10000);
		
		//	added by ZhangYong 20170602
		//	if we want to test the pump and spray on ground, we hope spray and pump act full speed
		//	shieleded by zhangyong to meet custmoer requirement 20180607
		/*if (_flags.testing) 
		{
			actual_pump_rate = 10000;
		}*/

		//printf("4. %d %d\n", actual_pump_rate, _spinner_pwm);
		
		//	added end
		//	modified by ZhangYong
		//	SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
		//	modified end
		//	
//		printf("actual_pump_rate %d\n", actual_pump_rate);
        SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, actual_pump_rate, 0, 10000);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
		
        _flags.spraying = true;
    }
	else
	{
        stop_spraying();
    }
}
