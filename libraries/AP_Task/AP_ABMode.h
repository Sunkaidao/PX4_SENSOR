#pragma once

#include <stdio.h>
#include <math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_WPNav/AC_WPNav.h>
#include <DataFlash/DataFlash.h>
#include "AP_Mission/AP_Mission.h"
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_SerialManager/AP_SerialManager.h>


#if ABMODE == ENABLED


#define YAW          0
#define POSITION     1

#define ISRIGHT 1
#define ISLIFT  0

#define BUZZER 0
#define RGB    1
#define BUZZER_AND_RGB 2


typedef struct _abmode
{
	bool is_record_a;
	bool is_record_b;
	bool is_calc_wp;
	Location a_loc;
	Location b_loc;
	Vector3d a_pos;
	Vector3d b_pos;
	bool is_start;
	int8_t is_first_start;
	bool direction;		// 1:right		0:left
	float yaw;
}ABPOINT;

typedef struct _breakpoint
{
	AP_Int32 lat_break;
	AP_Int32 lng_break;
	AP_Int32 alt_break;
	AP_Int32 lat_bp1;
	AP_Int32 lng_bp1;
	AP_Int32 lat_bp2;
	AP_Int32 lng_bp2;
	AP_Int32 index;
	AP_Int8  order;
	AP_Float yaw;
	AP_Int8 direction;
}BREAKPOINT;

typedef struct 
{
	Location wp;
	int32_t wp_mavlink_index;
}mavlink_info;

enum switch_type
{
	SEMIAUTO = 0,
	MANUAL = 1,
};

class AP_ABMode
{
public:
	AP_ABMode() ;
	~AP_ABMode();
	bool init();
	void update();
	void run();

	bool calc_two_wp( Vector3d &p1 , Vector3d &p2 , const double &d , const bool inside);

	Vector3d calThreePointCoord(const Vector3d p1 , const Vector3d p2 , const double d , const bool inside);
	Vector3d calc_three_wp(const Vector3d &p1 , const Vector3d &p2);
	Vector3d calc_three_wp_test();
	void get_position_test();
	Vector3d location_3d_diff_NED(const Location &loc1, const Location &loc2);
	Location NED_diff_location_3d(const Vector3d &poit_m, const struct Location &home_loc1);

	bool abmode_set_pos_a(void);
	bool abmode_set_pos_b(void);
	
	bool abmode_start(void);
	bool abmode_reset(void);

	void set_p1_p2_NED(const Location &home);

	void set_target_wp();
	void set_wp_cmd(uint8_t type,const Location &target, AP_Mission::Mission_Command &cmd,float yaw_degree = 0);
	void set_wp_alt_and_type(Location &target_cmd);

	void record_break_point(void);
	void clear_break_point(void);

	void restore_spray(const Location& home);

	void adjust_yaw();
	void adjust_yaw_test();

	//Change the direction of AB
	void invert_direction(switch_type        type = MANUAL,int8_t direction = ISRIGHT);

	void set_direction_from_rc_roll();
	bool get_direction(){return ab_mode.direction;}
	void direction_debug();

	void update_index();
	void update_order();
	
	//It is used to differentiate the waypoints from the aircraft 
	void mark_wp_mavlink_index(int32_t _order) {ab_wp.wp_mavlink_index = _order;}
	int32_t get_wp_order(){return ab_wp.wp_mavlink_index;}

	void mark_wp_loc(const Location wp){ab_wp.wp = wp;}
	const struct Location& get_wp_loc(){return ab_wp.wp;}
	
	int8_t get_relay_spray_mode(){return relay_spray_mode;}
	AP_Mission::Mission_Command get_target_cmd(){return target_cmd;}
	bool get_abmode_direction(){return ab_mode.direction;}
	int32_t get_wp_mavlink_index(){return ab_wp.wp_mavlink_index;}
	int8_t read_aux_switch_ab();

	void set_break_mode(int8_t _mode){break_mode = _mode;}
	void clear_break_mode(){break_mode = 0;}

	void set_relay_spray(){relay_spray.set_and_save_ifchanged(1);}
	void clear_relay_spray(){relay_spray.set_and_save_ifchanged(0);}

	void send_message();
	void handle_message();

	void update_rgb();
	void trigger_buzzer_and_rgb(int8_t type);

	void update_spray_dist();
	
	static const struct AP_Param::GroupInfo     var_info[];
protected:
		
	bool _initialised;

private:
	struct AP_Mission::Mission_Command target_cmd = {};

	//AP_Float width;
	AP_Int8  relay_spray;
	AP_Float stop_time;
	AP_Int8 relay_spray_mode;  //1:Ground station command  -1: remote controler command
	AP_Float rgb_time;

	ABPOINT ab_mode;
	BREAKPOINT relay;
	mavlink_info ab_wp;

	int8_t order;
	int8_t break_mode;  // 0:AB flight   1: break point, 2: calculate point
	float_t rgb_timer;
	int8_t rgb_flag;
    float width;
	
	int32_t alt_break;
	int32_t index;
	uint64_t timer;
	uint8_t step;
	uint8_t step_first;
	Vector3d p_1;  //unit: m
	Vector3d p_2;  //unit: m
	Location target_wp;
	Location home_loc;
	//	added by zhangyong for
};

#endif
