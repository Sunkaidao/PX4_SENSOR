#include"AP_ABMode.h"
#include "./../ArduCopter/Copter.h"
#include <AP_Param/AP_Param.h>

#if ABMODE == ENABLED


#define NO  0
#define YES 1

#define INVALID -3

#define A_TO_B      0
#define B_TO_A      1

//0: A to B,1: B to A
#define AB_SEQUENCE A_TO_B

#define ENABLE_DEFAULT 1
#define WIDTH_DEFAULT 3
#define RELAY_DEFAULT 0
#define ST_TIME_DEFAULT 1
#define RELAY_MD_DEFAULT 1
#define RGB_TIME_DEFAULT 1.5f

const AP_Param::GroupInfo AP_ABMode::var_info[] = {
	// @Param: WIDTH
	// @DisplayName: AB width between routes
	// @Parameter Type: float
	// @unit: Meter
	//AP_GROUPINFO("WIDTH", 1, AP_ABMode, width, WIDTH_DEFAULT),

	// @Param: RELAY
	// @DisplayName: Whether to go break point
	// @Parameter Type: int8
	//AP_GROUPINFO("RELAY", 2, AP_ABMode, relay_spray, RELAY_DEFAULT),

	// @Param: ST_TIME
	// @DisplayName: After reaching the target point, the time to pause
	// @Parameter Type: float
	// @unit: second
	AP_GROUPINFO("ST_TIME", 3, AP_ABMode, stop_time, ST_TIME_DEFAULT),

	// @Param: ST_TIME
	// @DisplayName: After reaching the target point, the time to pause
	// @Parameter Type: float
	// @unit: second
	//AP_GROUPINFO("RELAY_MD", 4, AP_ABMode, relay_spray_mode, RELAY_MD_DEFAULT),

	AP_GROUPINFO("LAT_BP1", 5, AP_ABMode, relay.lat_bp1, 0),
	AP_GROUPINFO("LNG_BP1", 6, AP_ABMode, relay.lng_bp1, 0),
	AP_GROUPINFO("LAT_BP2", 7, AP_ABMode, relay.lat_bp2, 0),
	AP_GROUPINFO("LNG_BP2", 8, AP_ABMode, relay.lng_bp2, 0),
	AP_GROUPINFO("LAT_BK", 9, AP_ABMode, relay.lat_break, 0),
	AP_GROUPINFO("LNG_BK", 10, AP_ABMode, relay.lng_break, 0),
	AP_GROUPINFO("ORDER", 11, AP_ABMode, relay.order, 0),
	AP_GROUPINFO("YAW", 12, AP_ABMode, relay.yaw, 0),
	AP_GROUPINFO("INDEX", 13, AP_ABMode, relay.index, 0),
	AP_GROUPINFO("ALT_BK", 14, AP_ABMode, relay.alt_break, 0),
	AP_GROUPINFO("DIR", 19, AP_ABMode, relay.direction, 0),
	
	// @Param: ST_TIME
	// @DisplayName: After reaching the target point, the time to pause
	// @Parameter Type: float
	// @unit: second
	AP_GROUPINFO("RGB_TIME", 20, AP_ABMode, rgb_time, RGB_TIME_DEFAULT),
	
    AP_GROUPEND
};




AP_ABMode::AP_ABMode()
{
	AP_Param::setup_object_defaults(this, var_info);	
  	_initialised = false;
	ab_mode.direction = ISRIGHT; 		// the right is default
}

AP_ABMode::~AP_ABMode()
{}

bool AP_ABMode::init()
{
	index = 0;
	step = 0;
	order = 0;
	step_first = 0;
	ab_mode.is_record_a = NO;
	ab_mode.is_record_b = NO;
	ab_mode.is_start = NO;
	ab_mode.is_calc_wp = NO;
	ab_mode.is_first_start = YES;
	alt_break = 0;
	rgb_timer = 0;
	rgb_flag = 0;
	ab_mode.yaw = 0;
    width = (copter.sprayer.get_unspray_dist()-50)/100.0f;
	
	memset( & ab_mode.a_loc, 0, sizeof(ab_mode.a_loc));
	memset( & ab_mode.b_loc, 0, sizeof(ab_mode.b_loc));
	memset( & ab_mode.a_pos, 0, sizeof(ab_mode.a_pos));
	memset( & ab_mode.b_pos, 0, sizeof(ab_mode.b_pos));
	memset( & target_cmd, 0, sizeof(target_cmd));
	memset( & p_1, 0, sizeof(p_1));
	memset( & p_2, 0, sizeof(p_2));
	mark_wp_mavlink_index(INVALID);
	set_break_mode(1);
	
	_initialised = true; 


	return true;
}

bool AP_ABMode::abmode_set_pos_a(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	
	
	Location home =  copter.ahrs.get_home();
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed()\
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{   
		ab_mode.a_loc.lng = copter.inertial_nav.get_longitude();
		ab_mode.a_loc.lat = copter.inertial_nav.get_latitude();
		ab_mode.a_loc.alt = copter.inertial_nav.get_position().z;
		ab_mode.a_pos = location_3d_diff_NED(home, ab_mode.a_loc);

		ab_mode.is_record_a = YES;
		
		//A point -1
		copter.DataFlash.Log_Write_Target_WP(ab_mode.a_loc,-1,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-1);
		mark_wp_loc(ab_mode.a_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set A point success");

		trigger_buzzer_and_rgb(BUZZER_AND_RGB);


		//	added by zhangyong 20180824, begin to spraying 20180824
		if(!copter.sprayer.get_running())
		{
			copter.sprayer.run(true);
			copter.sprayer.test_pump(!copter.motors->armed());
		}
		//	added end

		return true;
	}

	return false;
}

bool AP_ABMode::abmode_set_pos_b(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	

	if(ab_mode.is_record_a == NO)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Not record A");
		return false;
	}

	Location home = copter.ahrs.get_home();
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed() \
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{
		ab_mode.b_loc.lng = copter.inertial_nav.get_longitude();
		ab_mode.b_loc.lat = copter.inertial_nav.get_latitude();
		ab_mode.b_loc.alt = copter.inertial_nav.get_position().z;
		ab_mode.b_pos = location_3d_diff_NED(home, ab_mode.b_loc);
		
		ab_mode.is_record_b = YES;
		
		//B point -2
		copter.DataFlash.Log_Write_Target_WP(ab_mode.b_loc,-2,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-2);
		mark_wp_loc(ab_mode.b_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set B point success");

		trigger_buzzer_and_rgb(BUZZER_AND_RGB);
		
		return true;
	}

	return false;
}

bool AP_ABMode::calc_two_wp(Vector3d &p1 , Vector3d &p2 , const double &d , const bool inside)
{
	Vector3d p1_next;
	Vector3d p2_next;
	
	if((index % 2) == 0)
	{
		p1_next = calThreePointCoord(p1 , p2 , d , inside);
		p2_next = calThreePointCoord(p2 , p1 , d , !inside);

				
		p1 = p1_next;
		p2 = p2_next;
	}

	if(order == 4)
	{
		order = 0;
	}

#if AB_SEQUENCE == B_TO_A

	switch(order)
	{
		case -1:
			target_wp = NED_diff_location_3d(p1, home_loc);
			break;
		case 0:
			target_wp = NED_diff_location_3d(p1, home_loc);
			break;
		case 1:
			target_wp = NED_diff_location_3d(p2, home_loc);
			break;
		case 2:
			target_wp = NED_diff_location_3d(p2, home_loc);
			break;
		case 3:
			target_wp = NED_diff_location_3d(p1, home_loc);
			break;
	}
#else
	//GKXN
	switch(order)
	{
		case -1:
			target_wp = NED_diff_location_3d(p2, home_loc);
			break;
		case 0:
			target_wp = NED_diff_location_3d(p2, home_loc);
			break;
		case 1:
			target_wp = NED_diff_location_3d(p1, home_loc);
			break;
		case 2:
			target_wp = NED_diff_location_3d(p1, home_loc);
			break;
		case 3:
			target_wp = NED_diff_location_3d(p2, home_loc);
			break;
	}
#endif

	update_index();
	update_order();
	
    return true;
}

//Calculate on long side
Vector3d AP_ABMode::calc_three_wp(const Vector3d &p1 , const Vector3d &p2)
{
	Vector3d temp;
	Vector3d current_m;
	
	current_m.x = (double)copter.inertial_nav.get_position().x/(double)100.0f;
	current_m.y = (double)copter.inertial_nav.get_position().y/(double)100.0f;
	current_m.z = 0;
	
	temp.x = (current_m.x*(p2.x-p1.x)+current_m.y*(p2.y-p1.y)+p1.x*(pow(p2.y-p1.y,2))/(p2.x-p1.x)-p1.y*(p2.y-p1.y))*(p2.x-p1.x)/(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));
	temp.y = (p2.y-p1.y)/(p2.x-p1.x)*(temp.x-p1.x)+p1.y;
	temp.z = 0;
	
    return temp;
}

Vector3d AP_ABMode::calc_three_wp_test()
{
	Vector3d temp;
	Vector3d current_cm;
	Vector3d p1;
	Vector3d p2;

	p1.x = 1.0f;
	p1.y = 1.0f;
	p1.z = 0;

	p2.x = 3.0f;
	p2.y = 5.0f;
	p2.z = 0;
	
	current_cm.x = 3.0f;
	current_cm.y = 3.0f;
	current_cm.z = 0;
	
	temp.x = (current_cm.x*(p2.x-p1.x)+current_cm.y*(p2.y-p1.y)+p1.x*(pow(p2.y-p1.y,2))/(p2.x-p1.x)-p1.y*(p2.y-p1.y))*(p2.x-p1.x)/(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));
	temp.y = (p2.y-p1.y)/(p2.x-p1.x)*(temp.x-p1.x)+p1.y;
	temp.z = 0;

	printf("temp x %3.4f\n",temp.x);
	printf("temp y %3.4f\n",temp.y);

	double _current_m = (double)300.0f/(double)100.0f;
	printf("_current_m %4.4f\n",_current_m);
    return temp;
}


void AP_ABMode::update_index()
{
	index++;
}

void AP_ABMode::update_order()
{
	order++;
}

bool AP_ABMode::abmode_start(void)
{
	home_loc = copter.ahrs.get_home();

	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}
	
	if (ab_mode.is_record_a && ab_mode.is_record_b)
	{
		index = 0;
		order = 0;
		p_1 = ab_mode.a_pos;
		p_2 = ab_mode.b_pos;

#if AB_SEQUENCE == B_TO_A
		target_wp = ab_mode.a_loc;
#else
		target_wp = ab_mode.b_loc;
#endif
		clear_break_mode();
		
		ab_mode.is_first_start = YES;
		ab_mode.is_calc_wp = YES;
       //ab_mode.is_start = YES;
	}
	else if (!ab_mode.is_record_a && !ab_mode.is_record_b)
	{
		if(break_mode == 1)                     //Return from the current location to the breakpoint
		{
			restore_spray(home_loc);

			if(order % 2 == 0)
			{
				target_wp.lat = relay.lat_break;
				target_wp.lng = relay.lng_break;
			}
			else
			{
#if AB_SEQUENCE == B_TO_A		
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					case 3:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
#else				
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					case 3:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
#endif
			}
			if(order%2 == 0)
			{
				index--;
				order--;
			}
		}
		else if(break_mode == 2)                //Return from the current position to the flight path
		{
			restore_spray(home_loc);
		
			if(order % 2 == 0)
			{
				target_wp = NED_diff_location_3d(calc_three_wp(p_1,p_2), home_loc);
			}
			else
			{
#if AB_SEQUENCE == B_TO_A			
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					case 3:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
#else
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					case 3:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
#endif
			}
			if(order%2 == 0)
			{
				index--;
				order--;
			}
		}
		else
		{
			copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
		}

		ab_mode.is_first_start = YES;
		ab_mode.is_calc_wp = YES;
       ab_mode.is_start = YES;
	}
	else
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need record B");
		return false;
	}

	return true;
}

bool AP_ABMode:: abmode_reset(void)
{	
	ab_mode.is_record_a = NO;
	ab_mode.is_record_b = NO;
	ab_mode.is_start = NO;
	ab_mode.is_calc_wp = NO;
	ab_mode.is_first_start = YES;
	alt_break = 0;  // unit:cm
	rgb_timer = 0;
	rgb_flag = 0;
	ab_mode.yaw = 0;

	memset( & p_1, 0, sizeof(p_1));
	memset( & p_2, 0, sizeof(p_2));
	memset( & target_wp, 0, sizeof(target_wp));
	memset( & target_cmd, 0, sizeof(target_cmd));

	return true;
}

void AP_ABMode:: record_break_point(void)
{

	relay.lng_break.set_and_save_ifchanged(copter.inertial_nav.get_longitude());
	relay.lat_break.set_and_save_ifchanged(copter.inertial_nav.get_latitude());
	if(ab_mode.is_first_start == YES)
		relay.alt_break.set_and_save_ifchanged((int32_t)copter.inertial_nav.get_position().z);
	else
		relay.alt_break.set_and_save_ifchanged(target_cmd.content.location.alt);
	
	relay.lat_bp1.set_and_save_ifchanged(NED_diff_location_3d(p_1, home_loc).lat);
	relay.lng_bp1.set_and_save_ifchanged(NED_diff_location_3d(p_1, home_loc).lng);

	relay.lat_bp2.set_and_save_ifchanged(NED_diff_location_3d(p_2, home_loc).lat);
	relay.lng_bp2.set_and_save_ifchanged(NED_diff_location_3d(p_2, home_loc).lng);
	
	relay.index.set_and_save_ifchanged(index); //Records really "index", in the abmode_start() to decide whether to back off
	relay.order.set_and_save_ifchanged(order); //Records really "order", in the abmode_start() to decide whether to back off

	relay.yaw.set_and_save((float)copter.ahrs.yaw_sensor/100.0f);    //degrees

	relay.direction.set_and_save_ifchanged(ab_mode.direction);       // 1:right		0:left

}

void AP_ABMode:: restore_spray(const Location& home)
{	
    Location temp1;
	Location temp2;
	
	temp1.lat = relay.lat_bp1;
	temp1.lng = relay.lng_bp1;

	temp2.lat = relay.lat_bp2;
	temp2.lng = relay.lng_bp2;
	
	p_1 = location_3d_diff_NED(home, temp1);
	p_2 = location_3d_diff_NED(home, temp2);

	alt_break = relay.alt_break;
	
	index = relay.index;
	order = relay.order;
	ab_mode.direction = relay.direction;
}

void AP_ABMode:: set_wp_cmd(uint8_t type,const Location &target, AP_Mission::Mission_Command &cmd,float yaw_degree) 
{
  	if (type == YAW) 
	{
	    cmd.id = MAV_CMD_CONDITION_YAW;
	    cmd.content.yaw.angle_deg = yaw_degree;    // target angle in degrees (0 = north, 90 = east)
	    cmd.content.yaw.turn_rate_dps = 0;         // turn rate in degrees / second (0 = use default)
	    cmd.content.yaw.relative_angle = 0;        // 0 = absolute angle, 1 = relative angle

	    if ((int32_t)(copter.ahrs.yaw_sensor - yaw_degree*100) < 18000 \
			&& (int32_t)(copter.ahrs.yaw_sensor - yaw_degree*100) > -18000)
	    {
			cmd.content.yaw.direction = 1;         // cw(Clockwise)
		}
		else
		{
			cmd.content.yaw.direction = -1; 	   // ccw(Counterclockwise)
		}
  	} 
	else if (type == POSITION) 
  	{
	    // Set the command to return to the specified height(comm_alt) above the
	    // station  Station coordinates(lat_station,lng_station)
	    cmd.id = MAV_CMD_NAV_WAYPOINT;
	    cmd.content.location = target;
		
		set_wp_alt_and_type(cmd.content.location);  
  	}
}

void AP_ABMode:: set_wp_alt_and_type(Location &cmd_location)
{
	int32_t temp_alt;
	cmd_location.options = 0;
	
	temp_alt = MAX(ab_mode.a_loc.alt,ab_mode.b_loc.alt);
	cmd_location.alt = MAX(alt_break,temp_alt);
	cmd_location.flags.relative_alt = 1;

}

void AP_ABMode:: adjust_yaw()
{	
	if(copter.control_mode == ABMODE_RF \
		&& break_mode !=0 )
	{
		ab_mode.yaw = relay.yaw;
		
		set_wp_cmd(YAW,target_wp, target_cmd,ab_mode.yaw);
		copter.do_yaw(target_cmd);
	}
	else if(copter.control_mode == ABMODE_RF \
		&& break_mode ==0)
	{
		ab_mode.yaw = wrap_360(degrees(atan2f((p_2.y-p_1.y),(p_2.x-p_1.x))));
		set_wp_cmd(YAW,target_wp, target_cmd,ab_mode.yaw);
		copter.do_yaw(target_cmd);
	}

}

void AP_ABMode::adjust_yaw_test()
{
	Vector3d p1,p2;
	Location home;
	Location a_loc;
	Location b_loc;

	home.lat = 225622460;
	home.lng = 1134840987;
	home.alt = 0;

	a_loc.lat = 225622473;
	a_loc.lng = 1134842446;
	a_loc.alt = 0;

	b_loc.lat = 225620318;
	b_loc.lng = 1134842470;
	b_loc.alt = 0;
	
	p1 = location_3d_diff_NED(home, a_loc);
	p2 = location_3d_diff_NED(home, b_loc);

/*	printf("p1.x %4.4f\n",p1.x);
	printf("p1.y %4.4f\n",p1.y);
	printf("p2.x %4.4f\n",p2.x);
	printf("p2.y %4.4f\n",p2.y);
	printf("yaw    %4.4f\n",wrap_360(degrees(atan2f((p2.y-p1.y),(p2.x-p1.x)))));
*/
}

//set direction,when ab mode isn't running
void AP_ABMode:: set_direction_from_rc_roll()
{
	bool direction_rc = ISRIGHT;

	if(ab_mode.is_start)
	{
		return;
	}

	if(ABMODE_RF != copter.control_mode)
	{
		return;
	}

	if(!ab_mode.is_record_a || !ab_mode.is_record_b)
	{
		return;
	}

	if(copter.channel_roll == nullptr)
	{
		return;
	}
	
	if(copter.channel_roll->get_control_in() == 0)
	{
		return;
	}
	else if(copter.channel_roll->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/4)
	{
		direction_rc = ISLIFT;
	}
	else if(copter.channel_roll->get_control_in() > ROLL_PITCH_YAW_INPUT_MAX/4)
	{
		direction_rc = ISRIGHT;
	}
	else
	{
		return;
	}

	if(ab_mode.direction != direction_rc)
	{
		//ab_mode.direction = direction_rc;
		invert_direction(MANUAL,direction_rc);
	}

	ab_mode.is_start = YES;
}

void AP_ABMode:: invert_direction(switch_type        type,int8_t direction)
{	
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,direction failure");
		return;
	}
	switch(type)
	{
		case SEMIAUTO:
			ab_mode.direction = !ab_mode.direction;
			if(ab_mode.direction == ISRIGHT)
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Right");
			else
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Lift");
			break;
		case MANUAL:
			ab_mode.direction = direction;
			if(ab_mode.direction == ISRIGHT)
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Right");
			else
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Lift");
			break;
	}
}

void AP_ABMode:: direction_debug()
{
	if(get_direction() == ISRIGHT)
	{
		printf("ab mode: Right\n");
	}
	else
	{
		printf("ab mode: Lift\n");
	}
}

/*
 * Param[in]:	p1 - waypoint A(Vector3d)
 * 				p2 - waypoint B(Vector3d)
 *				d - width in m
 *				inside - direction (left and right)
 * Notes: this function for calculate two waypoint(latitude and longtitude), then return
 */
Vector3d AP_ABMode:: calThreePointCoord(const Vector3d p1 , const Vector3d p2 , const double d , const bool inside)
{
    double x = 0 , y = 0;

    if(inside)
    {
        x = p1.x - (d * (p2.y - p1.y)) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
        y = p1.y + (d * (p2.x - p1.x)) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }
    else
    {
        x = p1.x + (d * (p2.y - p1.y)) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
        y = p1.y - (d * (p2.x - p1.x)) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }

    return Vector3d(x , y, 0);
}

/*
  return the distance in meters in North/East/Down plane as a N/E/D vector
  from loc1 to loc2
  It is necessary to force conversion into double type
  only use 2D
  unit：M
 */
Vector3d AP_ABMode::location_3d_diff_NED(const struct Location &home_loc1, const struct Location &loc2)
{
    return Vector3d((double)(loc2.lat - home_loc1.lat) * (double)LOCATION_SCALING_FACTOR,
                    (double)(loc2.lng - home_loc1.lng) * (double)LOCATION_SCALING_FACTOR * (double)longitude_scale(home_loc1),
                    0);
}

/*
  return the Location ,N/E/D vector plane as a North/East/Down
  from poit_m to home_loc1
  It is necessary to force conversion into double type
 */
Location AP_ABMode::NED_diff_location_3d(const Vector3d &poit_m, const struct Location &home_loc1)
{
    Location temp;
	
	//must clear temp.options,if not clear,temp.options = 89.This is a strange phenomenon
	temp.options = 0;
	
	temp.lat = (int32_t)(poit_m.x/(double)(LOCATION_SCALING_FACTOR)) + home_loc1.lat;
	temp.lng = (int32_t)(poit_m.y/((double)(LOCATION_SCALING_FACTOR) * (double)longitude_scale(home_loc1))) + home_loc1.lng;
	temp.alt = 0;
	
    return temp;
}

void AP_ABMode:: run()
{
	switch(ab_mode.is_first_start)
	{
		case YES:
			switch(step_first)
			{
				case 0:
					if(copter.control_mode == ABMODE_RF \
		   				&& target_wp.lat != 0 \
		   				&& target_wp.lng != 0)
					{
						adjust_yaw();
						timer = AP_HAL::millis64();
						
						step_first = 1;
		   			}
					else if(target_wp.lat == 0 || target_wp.lng == 0)
					{
						gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
					}
					break;
				case 1:
					if((AP_HAL::millis64()-timer) > stop_time*1500)
					{
						if(copter.control_mode == ABMODE_RF \
			   				&& target_wp.lat != 0 \
			   				&& target_wp.lng != 0)
						{
							set_wp_cmd(POSITION, target_wp, target_cmd);
							
							if(!copter.do_abmode(target_cmd))
							{
								gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: do_abmode failure");
							}
							//fisrt point 0
							copter.DataFlash.Log_Write_Target_WP(target_cmd.content.location,0,ab_mode.direction,ab_mode.yaw,home_loc);
							mark_wp_mavlink_index(0);
							mark_wp_loc(target_cmd.content.location);
							
							step = 0;
							step_first = 0;
							ab_mode.is_first_start = NO;
						}
						else if(target_wp.lat == 0 || target_wp.lng == 0)
						{
							gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
						}
						
					}
					break;
			}			
			break;
			
		case NO:
			
			switch(step)
			{
				case 0:
					if (copter.wp_nav->reached_wp_destination()) {
      					timer = AP_HAL::millis64();
						step = 1;

						if(!copter.sprayer.get_running())
						{
							copter.sprayer.run(true);
							copter.sprayer.test_pump(!copter.motors->armed());
						}
    				}
					break;
				case 1:
					if((AP_HAL::millis64()-timer) > stop_time*1000)
					{
						if(copter.control_mode == ABMODE_RF \
					   		&& target_wp.lat != 0 \
					   		&& target_wp.lng != 0)
						{		
							calc_two_wp(p_1 , p_2 , width, ab_mode.direction);
							
							set_wp_cmd(POSITION, target_wp, target_cmd);
							
							if(!copter.do_abmode(target_cmd))
							{
								gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: do_abmode failure");
							}
							copter.DataFlash.Log_Write_Target_WP(target_cmd.content.location,index,ab_mode.direction,ab_mode.yaw,home_loc);
							mark_wp_mavlink_index(index);
							mark_wp_loc(target_cmd.content.location);
							
							step = 0;
						}
						else if(target_wp.lat == 0 || target_wp.lng == 0)
						{
							gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
						}
					}
					
					break;
			}
			break;
	}
	   
}

void AP_ABMode::get_position_test()
{
	Location temp;
	temp.lat = copter.inertial_nav.get_latitude();
	temp.lng = copter.inertial_nav.get_longitude();

	printf("home_loc lat %d\n",copter.ahrs.get_home().lat);
	printf("home_loc lng %d\n",copter.ahrs.get_home().lng);

	printf("currnet_loc lat %d\n",copter.inertial_nav.get_latitude());
	printf("currnet_loc lng %d\n",copter.inertial_nav.get_longitude());

	printf("get_position x %4.4f\n",(double)copter.inertial_nav.get_position().x);
	printf("get_position y %4.4f\n",(double)copter.inertial_nav.get_position().y);

	printf("currnet_ned x %4.4f\n",location_3d_diff_NED(copter.ahrs.get_home(),temp).x);
	printf("currnet_ned y %4.4f\n",location_3d_diff_NED(copter.ahrs.get_home(),temp).y);
}

void AP_ABMode::trigger_buzzer_and_rgb(int8_t type)
{
	switch(type)
	{
		case 0:
			AP_Notify::events.user_mode_change = 1;
			break;
		case 1:
			AP_Notify::flags.esc_calibration = 1;
			rgb_flag = 1;
			rgb_timer = 0;
			break;
		case 2:
			AP_Notify::events.user_mode_change = 1;
			AP_Notify::flags.esc_calibration = 1;
			rgb_flag = 1;
			rgb_timer = 0;
			break;
	}
}

void AP_ABMode::update_rgb()
{
	if(rgb_flag == 1)
	{
		if(is_equal(rgb_timer,10*rgb_time))
		{
			AP_Notify::flags.esc_calibration = 0;
			rgb_flag = 0;
			rgb_timer = 0;
		}
		
		rgb_timer++;
	}
}

void AP_ABMode::update_spray_dist()
{
	width = (copter.sprayer.get_unspray_dist()-50)/100.0f;
}

void AP_ABMode::update()
{
	if (!_initialised ) 
	{
    	return;
  	}

	update_spray_dist();
	update_rgb();
	set_direction_from_rc_roll();
    
	if (!ab_mode.is_start) 
	{
    	return;
  	}

	run();

}
#endif

