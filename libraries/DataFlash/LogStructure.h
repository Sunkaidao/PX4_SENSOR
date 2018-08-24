#pragma once

//	added by zhangyong 20180125
#include <./../../ArduCopter/config.h>
//	added end


/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id
#define LOG_PACKET_HEADER_LEN 3 // bytes required for LOG_PACKET_HEADER

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char name[5];
    const char format[16];
    const char labels[64];
};

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char name[16];
    float value;
};

//baiyang modified in 20170621
//#if DGPS_HEADINGA == ENABLED

struct PACKED log_GPS_HEADINGA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
	  //baiyang add in 20161117
	  float    heading;
	  //added end 
	  //baiyang add in 20161128
	  int16_t  rtk_status;
	  //added end 
	  int8_t   heading_status;
};
//#endif

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    float    ground_speed;
    float    ground_course;
    float    vel_z;
    uint8_t  used;
};

struct PACKED log_GPA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t vdop;
    uint16_t hacc;
    uint16_t vacc;
    uint16_t sacc;
    uint8_t  have_vv;
    uint32_t sample_ms;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    uint32_t gyro_error, accel_error;
    float temperature;
    uint8_t gyro_health, accel_health;
    uint16_t gyro_rate, accel_rate;
};

struct PACKED log_IMUDT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float delta_time, delta_vel_dt, delta_ang_dt;
    float delta_ang_x, delta_ang_y, delta_ang_z;
    float delta_vel_x, delta_vel_y, delta_vel_z;
};

struct PACKED log_Vibe {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vibe_x, vibe_y, vibe_z;
    uint32_t clipping_0, clipping_1, clipping_2;
};

struct PACKED log_Gimbal1 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float delta_time;
    float delta_angles_x;
    float delta_angles_y;
    float delta_angles_z;
    float delta_velocity_x;
    float delta_velocity_y;
    float delta_velocity_z;
    float joint_angles_x;
    float joint_angles_y;
    float joint_angles_z;
};

struct PACKED log_Gimbal2 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  est_sta;
    float est_x;
    float est_y;
    float est_z;
    float rate_x;
    float rate_y;
    float rate_z;
    float target_x;
    float target_y;
    float target_z;
};

struct PACKED log_Gimbal3 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t rl_torque_cmd;
    int16_t el_torque_cmd;
    int16_t az_torque_cmd;
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
    uint16_t chan10;
    uint16_t chan11;
    uint16_t chan12;
    uint16_t chan13;
    uint16_t chan14;
};

struct PACKED log_RSSI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float RXRSSI;
};

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   altitude;
    float   pressure;
    int16_t temperature;
    float   climbrate;
    uint32_t sample_time_ms;
    float   drift_offset;
    float   ground_temp;
};

struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
    float q1, q2, q3, q4;
};

struct PACKED log_POS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
    int32_t lng;
    float alt;
    float rel_home_alt;
    float rel_origin_alt;
};

//	added by ZhangYong
struct PACKED log_SPRAYER {
    LOG_PACKET_HEADER;
    uint64_t timestamp;
	int8_t enabled;
    uint8_t running;
	uint8_t sprayering;
	uint8_t testing;
	uint32_t speed_over_min_time;
	uint32_t speed_under_min_time;
	uint16_t actual_pump_rate;
	uint32_t wp_dist;
	uint8_t short_edge;
	uint8_t fm_warn;
	uint8_t pck_cnt;
	double fm_vol;
	double fm_h;
	uint16_t flight_area;
};



struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Vcc;
    float Vservo;
    uint16_t flags;
};

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posD_dot;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
    int32_t originHgt;
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int8_t Ratio;
    int8_t AZ1bias;
    int8_t AZ2bias;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct PACKED log_NKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int8_t AZbias;
    int16_t scaleX;
    int16_t scaleY;
    int16_t scaleZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    uint8_t index;
};

struct PACKED log_NKF2a {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t accBiasX;
    int16_t accBiasY;
    int16_t accBiasZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    uint8_t index;
};

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovVT;
};

struct PACKED log_NKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovYaw;
    int16_t innovVT;
};

struct PACKED log_EKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarMX;
    int16_t sqrtvarMY;
    int16_t sqrtvarMZ;
    int16_t sqrtvarVT;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint16_t solution;
    uint16_t gps;
};

struct PACKED log_NKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarM;
    int16_t sqrtvarVT;
    float   tiltErr;
    int8_t  offsetNorth;
    int8_t  offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint16_t solution;
    uint16_t gps;
    int8_t primary;
};

struct PACKED log_EKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
};

struct PACKED log_NKF5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t normInnov;
    int16_t FIX;
    int16_t FIY;
    int16_t AFI;
    int16_t HAGL;
    int16_t offset;
    int16_t RI;
    uint16_t meaRng;
    uint16_t errHAGL;
    float angErr;
    float velErr;
    float posErr;
};

struct PACKED log_Quaternion {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float q1;
    float q2;
    float q3;
    float q4;
};

struct PACKED log_RngBcnDebug {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t ID;             // beacon identifier
    int16_t rng;            // beacon range (cm)
    int16_t innov;          // beacon range innovation (cm)
    uint16_t sqrtInnovVar;  // sqrt of beacon range innovation variance (cm)
    uint16_t testRatio;     // beacon range innovation consistency test ratio *100
    int16_t beaconPosN;     // beacon north position (cm)
    int16_t beaconPosE;     // beacon east position (cm)
    int16_t beaconPosD;     // beacon down position (cm)
    int16_t offsetHigh;     // high estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t offsetLow;      // low estimate of vertical position offset of beacons rel to EKF origin (cm)
    int16_t posN;           // North position of receiver rel to EKF origin (cm)
    int16_t posE;           // East position of receiver rel to EKF origin (cm)
    int16_t posD;           // Down position of receiver rel to EKF origin (cm)
};

// visual odometry sensor data
struct PACKED log_VisualOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float time_delta;
    float angle_delta_x;
    float angle_delta_y;
    float angle_delta_z;
    float position_delta_x;
    float position_delta_y;
    float position_delta_z;
    float confidence;
};

struct PACKED log_ekfBodyOdomDebug {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float velInnovX;
    float velInnovY;
    float velInnovZ;
    float velInnovVarX;
    float velInnovVarY;
    float velInnovVarZ;
};

struct PACKED log_ekfStateVar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float v00;
    float v01;
    float v02;
    float v03;
    float v04;
    float v05;
    float v06;
    float v07;
    float v08;
    float v09;
    float v10;
    float v11;
};

struct PACKED log_Cmd {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t command_total;
    uint16_t sequence;
    uint16_t command;
    float param1;
    float param2;
    float param3;
    float param4;
    float latitude;
    float longitude;
    float altitude;
	uint8_t frame;
};

struct PACKED log_Radio {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t rssi;
    uint8_t remrssi;
    uint8_t txbuf;
    uint8_t noise;
    uint8_t remnoise;
    uint16_t rxerrors;
    uint16_t fixed;
};

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int32_t  altitude_rel;
    int32_t  altitude_gps;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

struct PACKED log_PID {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   desired;
    float   P;
    float   I;
    float   D;
    float   FF;
    float   AFF;
	//	added by ZhangYong for debug purpose
	float 	actual;
	float	error;
	float	scale;
//	added end	
};

//	added by ZhangYong 20180105 just for loging
/*
struct MTR_log
{
	float rc_roll_log;
	float rc_pitch_log;
	float rc_throttle_log;
	float rc_yaw_log;
	float thr_thrust_best_log;
	float yaw_allowed_log;
	float rpy_low_log;
	float rpy_high_log;
	float thr_thrust_best_rpy_log;
	float rpy_scale_log;
	uint8_t limit_roll_pitch;
	uint8_t limit_yaw;
	uint8_t limit_thr_lower;
	uint8_t limit_thr_upper;
};*/
//	added end



struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    voltage;
    float    voltage_resting;
    float    current_amps;
    float    current_total;
    int16_t  temperature; // degrees C * 100
    float    resistance;
	//	added by ZhangYong
	float	power;
	//	added end
};

struct PACKED log_Current_Cells {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    voltage;
    uint16_t cell_voltages[10];
};

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
    uint8_t  health;
    uint32_t SUS;
};

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t mode;
    uint8_t mode_num;
    uint8_t mode_reason;
};

/*
  rangefinder - support for 4 sensors
 */
struct PACKED log_RFND {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t dist1;
    uint8_t orient1;
    uint16_t dist2;
    uint8_t orient2;
	//	added by ZhangYong
	uint16_t dist3;
    uint8_t orient3;
	//	added end
	uint16_t error1;
	uint16_t notarget1;
	uint16_t unvaildnum1;
	uint8_t condition1;
};

#if CHARGINGSTATION == ENABLED
//baiyang added in 20170523
struct PACKED log_RTBS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t hit;
	uint8_t btsm;
	uint8_t lap;
	int8_t ask;
	uint8_t msg;
};
//added end
#endif

//#if ABMODE == ENABLED
//baiyang added in 20171102
struct PACKED log_TWP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t lat;
	int32_t lng;
	int32_t alt;
	uint8_t alt_type;
	int32_t idx;
	int8_t  dir;
	float yaw;
	int32_t home_lat;
	int32_t home_lng;
};
//added end
//#endif

//#if FXTX_AUTH == ENABLED
//	added by ZhangYong 20170731
struct PACKED log_Communication_drops {
    LOG_PACKET_HEADER;
    uint64_t timestamp;
	  int32_t  home_distances;
    float  communication_drops;
};

//	added end
//#endif

//	added by ZhangYong 20180104
struct PACKED log_PADCMD {
    LOG_PACKET_HEADER;
	uint64_t timestamp;
	uint32_t msgid_lg;
};
//	added end

//	added by sunkaidao 20180904
struct PACKED log_Sensor {
    LOG_PACKET_HEADER;
	uint64_t time_us;
	float CO;
	float SO2;
	float NO2;
	float O3;
	float X1;
	float X2;
	float CO2;
	float PM1_0;
	float PM2_5;
	float PM10;
	float TEMP;
	float HUM;
};
//	added end



	
/*
*/




struct PACKED log_MTR{
	LOG_PACKET_HEADER;
    uint64_t timestamp;
    float rc_roll;
	float rc_pitch;
	float rc_throttle;
	float rc_yaw;
	float thr_thrust_best;
	float yaw_allowed;
	float rpy_low;
	float rpy_high;
	float thr_thrust_best_rpy;
	float rpy_scale;
	uint8_t lmt_roll_pitch;
	uint8_t lmt_yaw;
	uint8_t lmt_thr_lower;
	uint8_t lmt_thr_upper;
};

struct PACKED log_Control {
	LOG_PACKET_HEADER;
	uint64_t timestamp;
	uint8_t rc_override_active_log;
	uint8_t rcin_override_valid_log;		
	uint8_t rc_valid_log;
	uint16_t rc3_radio_in;
};


struct PACKED log_GKProx {
	LOG_PACKET_HEADER;
	uint64_t timestamp;
    uint8_t   health   ;
    float   dist0     ;     
    float   dist180    ;    
    float   cmdan      ;     
    float closest_angle  ; 
    float  closest_dist ;
	float  valid_num;
	bool   R1_warn;
	bool   R3_warn;
	uint16_t unc_num;
	uint16_t error_num;
	int32_t t_angle;
};


/*
  terrain log structure
 */
struct PACKED log_TERRAIN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t status;
    int32_t lat;
    int32_t lng;
    uint16_t spacing;
    float terrain_height;
    float current_height;
    uint16_t pending;
    uint16_t loaded;
};

/*
  UBlox logging
 */
struct PACKED log_Ubx1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t noisePerMS;
    uint8_t  jamInd;
    uint8_t  aPower;
    uint16_t agcCnt;
};

struct PACKED log_Ubx2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
};

struct PACKED log_GPS_RAW {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t iTOW;
    int16_t week;
    uint8_t numSV;
    uint8_t sv;
    double cpMes;
    double prMes;
    float doMes;
    int8_t mesQI;
    int8_t cno;
    uint8_t lli;
};

struct PACKED log_GPS_RAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
};

struct PACKED log_GPS_RAWS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double prMes;
    double cpMes;
    float doMes;
    uint8_t gnssId;
    uint8_t svId;
    uint8_t freqId;
    uint16_t locktime;
    uint8_t cno;
    uint8_t prStdev;
    uint8_t cpStdev;
    uint8_t doStdev;
    uint8_t trkStat;
};

struct PACKED log_GPS_SBF_EVENT {  
	LOG_PACKET_HEADER; 
	uint64_t time_us;
	uint32_t TOW;
	uint16_t WNc;
	uint8_t Mode;
	uint8_t Error;
	double Latitude;
	double Longitude;
	double Height;
	float Undulation;
	float Vn;
	float Ve;
	float Vu;
	float COG;
};

struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;     
    int16_t rpm;
    int16_t voltage;
    int16_t current;
    int16_t temperature;
};

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
    float   rawpressure;
    float   offset;
    bool    use;
};

struct PACKED log_ACCEL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float AccX, AccY, AccZ;
};

struct PACKED log_GYRO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t sample_us;
    float GyrX, GyrY, GyrZ;
};

struct PACKED log_DF_MAV_Stats {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint32_t seqno;
    uint32_t dropped;
    uint32_t retries;
    uint32_t resends;
    uint8_t internal_errors; // uint8_t - wishful thinking?
    uint8_t state_free_avg;
    uint8_t state_free_min;
    uint8_t state_free_max;
    uint8_t state_pending_avg;
    uint8_t state_pending_min;
    uint8_t state_pending_max;
    uint8_t state_sent_avg;
    uint8_t state_sent_min;
    uint8_t state_sent_max;
    // uint8_t state_retry_avg;
    // uint8_t state_retry_min;
    // uint8_t state_retry_max;
};

struct PACKED log_ORGN {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t origin_type;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct PACKED log_RPM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float rpm1;
    float rpm2;
};

struct PACKED log_Rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   control_roll;
    float   roll;
    float   roll_out;
    float   control_pitch;
    float   pitch;
    float   pitch_out;
    float   control_yaw;
    float   yaw;
    float   yaw_out;
    float   control_accel;
    float   accel;
    float   accel_out;
};

// #if SBP_HW_LOGGING

struct PACKED log_SbpLLH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t tow;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
    uint8_t  n_sats;
    uint8_t  flags;
};

struct PACKED log_SbpHealth {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t crc_error_counter;
    uint32_t last_injected_data_ms;
    uint32_t last_iar_num_hypotheses;
};

struct PACKED log_SbpRAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[48];
};

struct PACKED log_SbpRAWM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t msg_type;
    uint16_t sender_id;
    uint8_t index;
    uint8_t pages;
    uint8_t msg_len;
    uint8_t res;
    uint8_t data[104];
};

struct PACKED log_SbpEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t wn;
    uint32_t tow;
    int32_t ns_residual;
    uint8_t level;
    uint8_t quality;
};

struct PACKED log_Rally {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t total;
    uint8_t sequence;
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
};

struct PACKED log_AOA_SSA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float AOA;
    float SSA;
};

struct PACKED log_Beacon {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    float posx;
    float posy;
    float posz;
};


struct PACKED log_Land_Detector {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t land_complete;
	uint8_t land_complete_maybe;
	uint8_t motors_limit_throttle_lower;
	uint8_t att_is_throttle_mix_min;
	float 	land_accel_ef_filter_length;
	float  	descent_rate_low;
	uint8_t rangefinder_check;
};


//	added by ZhangYong 20170731
/*
struct PACKED log_Communication_drops {
    LOG_PACKET_HEADER;
    uint64_t timestamp;
	int32_t  home_distances;
    float  communication_drops;
};*/

struct PACKED log_BCBPMBus {
	LOG_PACKET_HEADER;
    uint64_t timestamp;						//	0

	uint8_t seq;
	
	uint8_t component_id;					//	1

	uint8_t status_byte;					//	2
	uint8_t status_word;					//	3
	uint8_t status_iout;					//	4
	uint8_t status_input;					//	5
	uint8_t status_temperature;				//	6
	uint8_t status_cml;						//	7
		
	uint16_t read_vin;						//	8
	uint16_t read_iin;						//	9
	uint16_t read_vout;						//	10
	uint16_t read_iout;						//	11
	uint16_t read_temperature;				//	12
	uint16_t read_pout;						//	13
};


typedef struct PACKED BCBPMBus_component_info_struct {
		uint8_t component_id;

		uint8_t status_byte;					//	10
		uint8_t status_word;					//	11
		uint8_t status_iout;					//	12
		uint8_t status_input;					//	13
		uint8_t status_temperature;				//	14
		uint8_t status_cml;						//	15
		
		uint16_t read_vin;
		uint16_t read_iin;
		uint16_t read_vout;
		uint16_t read_iout;
		uint16_t read_temperature;
		uint16_t read_pout;
}BCBPMBUS_COMPONENT_INFO;

//	added end


// proximity sensor logging
struct PACKED log_Proximity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    float dist0;
    float dist45;
    float dist90;
    float dist135;
    float dist180;
    float dist225;
    float dist270;
    float dist315;
    float distup;
    float closest_angle;
    float closest_dist;
};


// #endif // SBP_HW_LOGGING

#define ACC_LABELS "TimeUS,SampleUS,AccX,AccY,AccZ"
#define ACC_FMT "QQfff"

// see "struct sensor" in AP_Baro.h and "Log_Write_Baro":
#define BARO_LABELS "TimeUS,Alt,Press,Temp,CRt,SMS,Offset,GndTemp"
#define BARO_FMT   "QffcfIff"

// see "log_Communication_drops GPS_State" and "Log_Write_CD":
#define CD_LABELS "TMS,D2H,CD" 
#define CD_FMT  "Qif"

#define ESC_LABELS "TimeUS,RPM,Volt,Curr,Temp"
#define ESC_FMT   "Qcccc"

#define GPA_LABELS "TimeUS,VDop,HAcc,VAcc,SAcc,VV,SMS"
#define GPA_FMT   "QCCCCBI"

// see "struct GPS_State" and "Log_Write_GPS":
#define GPS_LABELS "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,U"
#define GPS_FMT   "QBIHBcLLefffB"

// see "struct log_RTBS" and "Log_Write_RTBS":
#define RTK_LABELS "TimeUS,HD,Rs,HDs"
#define RTK_FMT   "Qfhb"

// see "struct log_TWP" and "Log_Write_TWP":
#define TWP_LABELS "TimeUS,lat,lng,alt,a_type,index1,dir,yaw,Hlat,Hlng"
#define TWP_FMT   "QiiiBibfii"

#define GYR_LABELS "TimeUS,SampleUS,GyrX,GyrY,GyrZ"
#define GYR_FMT    "QQfff"

#define IMT_LABELS "TimeUS,DelT,DelvT,DelaT,DelAX,DelAY,DelAZ,DelVX,DelVY,DelVZ"
#define IMT_FMT    "Qfffffffff"

#define IMU_LABELS "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz"
#define IMU_FMT   "QffffffIIfBBHH"

#define MAG_LABELS "TimeUS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ,Health,S"
#define MAG_FMT   "QhhhhhhhhhBI"

//	modified by ZhangYong 20170629
//#define PID_LABELS "TimeUS,Des,P,I,D,FF,AFF"
//#define PID_FMT    "Qffffff"
//	modified end

#define PID_LABELS "TimeUS,Des,P,I,D,FF,AFF,At,Er,Sc"
#define PID_FMT    "Qfffffffff"


#define QUAT_LABELS "TimeUS,Q1,Q2,Q3,Q4"
#define QUAT_FMT    "Qffff"


//	modified by ZhangYong 20171101
//#define CURR_LABELS "TimeUS,Volt,VoltR,Curr,CurrTot,Temp,Res"
//#define CURR_FMT    "Qffffcf"
//	modified end


#define CURR_LABELS "TimeUS,Volt,VoltR,Curr,CurrTot,Temp,Res,Pwr"
#define CURR_FMT    "Qffffcff"

#define CURR_CELL_LABELS "TimeUS,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10"
#define CURR_CELL_FMT    "QfHHHHHHHHHH"

#define PMBUS_LABELS	"TMS,seq,id,sb,sw,sio,sin,st,sc,vi,ii,vo,io,t,p"
#define PMBUS_FMT	"QBBBBBBBBHHHHHH"



/*
// message types for common messages
enum LogMessages {
//	modified by ZhangYong for over
//	LOG_FORMAT_MSG = 128,
//	modified end
	LOG_FORMAT_MSG = 128,
    LOG_PARAMETER_MSG,
    LOG_GPS_MSG,
    LOG_GPS2_MSG,
    LOG_GPSB_MSG,
    LOG_IMU_MSG,
    LOG_MESSAGE_MSG,
    LOG_RCIN_MSG,
    LOG_RCOUT_MSG,
    LOG_RSSI_MSG,
    LOG_IMU2_MSG,
    LOG_BARO_MSG,
    LOG_POWR_MSG,
    LOG_AHR2_MSG,
    LOG_SIMSTATE_MSG,
    LOG_CMD_MSG,
    LOG_RADIO_MSG,
    LOG_ATRP_MSG,
    LOG_CAMERA_MSG,
    LOG_IMU3_MSG,
    LOG_TERRAIN_MSG,
    LOG_GPS_UBX1_MSG,
    LOG_GPS_UBX2_MSG,
    LOG_GPS2_UBX1_MSG,
    LOG_GPS2_UBX2_MSG,
    LOG_ESC1_MSG,
    LOG_ESC2_MSG,
    LOG_ESC3_MSG,
    LOG_ESC4_MSG,
    LOG_ESC5_MSG,
    LOG_ESC6_MSG,
    LOG_ESC7_MSG,
    LOG_ESC8_MSG,
    LOG_BAR2_MSG,
    LOG_ARSP_MSG,
    LOG_ATTITUDE_MSG,
    LOG_CURRENT_MSG,
    LOG_CURRENT2_MSG,
  shielded by ZhangYong 20180116 to save msg_id
    LOG_CURRENT_CELLS_MSG,
    LOG_CURRENT_CELLS2_MSG,
    LOG_COMPASS_MSG,
    LOG_COMPASS2_MSG,
    LOG_COMPASS3_MSG,
    LOG_MODE_MSG,
    LOG_GPS_RAW_MSG,
    LOG_GPS_RAWH_MSG,
    LOG_GPS_RAWS_MSG,
	LOG_GPS_SBF_EVENT_MSG,
    LOG_ACC1_MSG,
    LOG_ACC2_MSG,
    LOG_ACC3_MSG,
    LOG_GYR1_MSG,
    LOG_GYR2_MSG,
    LOG_GYR3_MSG,
    LOG_POS_MSG,
    LOG_PIDR_MSG,
    LOG_PIDP_MSG,
    LOG_PIDY_MSG,
    LOG_PIDA_MSG,
    LOG_PIDS_MSG,
    LOG_PIDL_MSG,
    LOG_VIBE_MSG,
    LOG_IMUDT_MSG,
    LOG_IMUDT2_MSG,
    LOG_IMUDT3_MSG,
    LOG_ORGN_MSG,
    LOG_RPM_MSG,
    LOG_GPA_MSG,
    LOG_GPA2_MSG,
    LOG_GPAB_MSG,
    LOG_RFND_MSG,
    LOG_BAR3_MSG,
    LOG_NKF1_MSG,
    LOG_NKF2_MSG,
    LOG_NKF3_MSG,
    LOG_NKF4_MSG,
    LOG_NKF5_MSG,
    LOG_NKF6_MSG,
    LOG_NKF7_MSG,
    LOG_NKF8_MSG,
    LOG_NKF9_MSG,
    LOG_NKF10_MSG,
    LOG_NKQ1_MSG,
    LOG_NKQ2_MSG,
    LOG_XKF1_MSG,
    LOG_XKF2_MSG,
    LOG_XKF3_MSG,
    LOG_XKF4_MSG,
    LOG_XKF5_MSG,
    LOG_XKF6_MSG,
    LOG_XKF7_MSG,
    LOG_XKF8_MSG,
    LOG_XKF9_MSG,
    LOG_XKF10_MSG,
    LOG_XKQ1_MSG,
    LOG_XKQ2_MSG,
    LOG_XKFD_MSG,
    LOG_XKV1_MSG,
    LOG_XKV2_MSG,
    LOG_DF_MAV_STATS,

    LOG_MSG_SBPHEALTH,
    LOG_MSG_SBPLLH,
    LOG_MSG_SBPBASELINE,
    LOG_MSG_SBPTRACKING1,
    LOG_MSG_SBPTRACKING2,
    LOG_MSG_SBPRAWH,
    LOG_MSG_SBPRAWM,
    LOG_MSG_SBPEVENT,
    LOG_TRIGGER_MSG,

    LOG_GIMBAL1_MSG,
    LOG_GIMBAL2_MSG,
    LOG_GIMBAL3_MSG,
  LOG_RATE_MSG,
    LOG_RALLY_MSG,
    LOG_VISUALODOM_MSG,
    LOG_AOA_SSA_MSG,
    LOG_BEACON_MSG,

    LOG_SPRAYER_MSG,
	
//#if BCBPMBUS == ENABLED	
	LOG_PMBUS0_MSG,
	LOG_PMBUS1_MSG,
	LOG_PMBUS2_MSG,
	LOG_PMBUS3_MSG,
//#endif	
    LOG_PROXIMITY_MSG,

// #if CHARGINGSTATION == ENABLED
    //baiyang added in 20170523
    LOG_RTBS_MSG,
    //added end
// #endif

//baiyang added in 20170829
// #if DGPS_HEADINGA == ENABLED
    LOG_GPS_HEADINGA_MSG,
    LOG_GPS2_HEADINGA_MSG,
// #endif
// added end
    
//baiyang added in 20170831
    LOG_CD_MSG,
//added end

//#if ABMODE == ENABLED
	LOG_TWP_MSG,
//#endif
	LOG_PADCMD_MSG,
	LOG_MTR_MSG,
	LOG_CONTROL_MSG,
	LOG_GKPROX_MSG
};*/


// message types for common messages
enum LogMessages {
//	modified by ZhangYong for over
//	LOG_FORMAT_MSG = 128,
//	modified end
	LOG_FORMAT_MSG = 128,
    LOG_PARAMETER_MSG,
    LOG_GPS_MSG,
    LOG_GPS2_MSG,
    LOG_GPSB_MSG,
    LOG_IMU_MSG,
    LOG_MESSAGE_MSG,
    LOG_RCIN_MSG,
    LOG_RCOUT_MSG,
    LOG_RSSI_MSG,
    LOG_IMU2_MSG,
    LOG_BARO_MSG,
    LOG_POWR_MSG,
    LOG_AHR2_MSG,
    LOG_SIMSTATE_MSG,
    LOG_CMD_MSG,
    LOG_RADIO_MSG,
    LOG_ATRP_MSG,
    LOG_CAMERA_MSG,
    LOG_IMU3_MSG,
    LOG_TERRAIN_MSG,
    LOG_GPS_UBX1_MSG,
    LOG_GPS_UBX2_MSG,
    LOG_GPS2_UBX1_MSG,
    LOG_GPS2_UBX2_MSG,
    LOG_ESC1_MSG,
    LOG_ESC2_MSG,
    LOG_ESC3_MSG,
    LOG_ESC4_MSG,
    LOG_ESC5_MSG,
    LOG_ESC6_MSG,
    LOG_ESC7_MSG,
    LOG_ESC8_MSG,
    LOG_BAR2_MSG,
    LOG_ARSP_MSG,
    LOG_ATTITUDE_MSG,
    LOG_CURRENT_MSG,
    LOG_CURRENT2_MSG,
/*  shielded by ZhangYong 20180116 to save msg_id
    LOG_CURRENT_CELLS_MSG,
    LOG_CURRENT_CELLS2_MSG,
*/    LOG_COMPASS_MSG,
    LOG_COMPASS2_MSG,
    LOG_COMPASS3_MSG,
    LOG_MODE_MSG,
    LOG_GPS_RAW_MSG,
    LOG_GPS_RAWH_MSG,
    LOG_GPS_RAWS_MSG,
	LOG_GPS_SBF_EVENT_MSG,
    LOG_ACC1_MSG,
    LOG_ACC2_MSG,
    LOG_ACC3_MSG,
    LOG_GYR1_MSG,
    LOG_GYR2_MSG,
    LOG_GYR3_MSG,
    LOG_POS_MSG,
    LOG_PIDR_MSG,
    LOG_PIDP_MSG,
    LOG_PIDY_MSG,
    LOG_PIDA_MSG,
    LOG_PIDS_MSG,
    LOG_PIDL_MSG,
    LOG_VIBE_MSG,
    LOG_IMUDT_MSG,
    LOG_IMUDT2_MSG,
    LOG_IMUDT3_MSG,
    LOG_ORGN_MSG,
    LOG_RPM_MSG,
    LOG_GPA_MSG,
    LOG_GPA2_MSG,
    LOG_GPAB_MSG,
    LOG_RFND_MSG,
    LOG_BAR3_MSG,
    LOG_NKF1_MSG,
    LOG_NKF2_MSG,
    LOG_NKF3_MSG,
    LOG_NKF4_MSG,
    LOG_NKF5_MSG,
    LOG_NKF6_MSG,
    LOG_NKF7_MSG,
    LOG_NKF8_MSG,
    LOG_NKF9_MSG,
    LOG_NKF10_MSG,
    LOG_NKQ1_MSG,
    LOG_NKQ2_MSG,
    LOG_XKF1_MSG,
    LOG_XKF2_MSG,
    LOG_XKF3_MSG,
    LOG_XKF4_MSG,
    LOG_XKF5_MSG,
    LOG_XKF6_MSG,
    LOG_XKF7_MSG,
    LOG_XKF8_MSG,
    LOG_XKF9_MSG,
    LOG_XKF10_MSG,
    LOG_XKQ1_MSG,
    LOG_XKQ2_MSG,
    LOG_XKFD_MSG,
    LOG_XKV1_MSG,
    LOG_XKV2_MSG,
    LOG_DF_MAV_STATS,

    LOG_MSG_SBPHEALTH,
    LOG_MSG_SBPLLH,
    LOG_MSG_SBPBASELINE,
    LOG_MSG_SBPTRACKING1,
    LOG_MSG_SBPTRACKING2,
    LOG_MSG_SBPRAWH,
    LOG_MSG_SBPRAWM,
    LOG_MSG_SBPEVENT,
    LOG_TRIGGER_MSG,

/*    LOG_GIMBAL1_MSG,
    LOG_GIMBAL2_MSG,
    LOG_GIMBAL3_MSG,
 */ LOG_RATE_MSG,
    LOG_RALLY_MSG,
    LOG_LD_MSG,
    LOG_AOA_SSA_MSG,
    LOG_BEACON_MSG,

    LOG_SPRAYER_MSG,
	
//#if BCBPMBUS == ENABLED	
	LOG_PMBUS0_MSG,
	LOG_PMBUS1_MSG,
	LOG_PMBUS2_MSG,
	LOG_PMBUS3_MSG,
//#endif	
    LOG_PROXIMITY_MSG,

// #if CHARGINGSTATION == ENABLED
    //baiyang added in 20170523
    LOG_RTBS_MSG,
    //added end
// #endif

//baiyang added in 20170829
// #if DGPS_HEADINGA == ENABLED
    LOG_GPS_HEADINGA_MSG,
    LOG_GPS2_HEADINGA_MSG,
// #endif
// added end
    
//baiyang added in 20170831
    LOG_CD_MSG,
//added end

//#if ABMODE == ENABLED
	LOG_TWP_MSG,
//#endif
	LOG_PADCMD_MSG,
	LOG_MTR_MSG,
	LOG_CONTROL_MSG,
	LOG_GKPROX_MSG,

//sunkaidao added in 180904
	LOG_SENSOR_MSG
//added end
};


/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
  q   : int64_t
  Q   : uint64_t
 */

//	modified by ZhangYong 20180105 log id over

/*
{ LOG_GIMBAL1_MSG, sizeof(log_Gimbal1), \
      "GMB1", "Iffffffffff", "TimeMS,dt,dax,day,daz,dvx,dvy,dvz,jx,jy,jz" }, \
    { LOG_GIMBAL2_MSG, sizeof(log_Gimbal2), \
      "GMB2", "IBfffffffff", "TimeMS,es,ex,ey,ez,rx,ry,rz,tx,ty,tz" }, \
    { LOG_GIMBAL3_MSG, sizeof(log_Gimbal3), \
      "GMB3", "Ihhh", "TimeMS,rl_torque_cmd,el_torque_cmd,az_torque_cmd" }, \

   { LOG_CURRENT_CELLS_MSG, sizeof(log_Current_Cells), \
      "BCL", CURR_CELL_FMT, CURR_CELL_LABELS }, \
    { LOG_CURRENT_CELLS2_MSG, sizeof(log_Current_Cells), \
      "BCL2", CURR_CELL_FMT, CURR_CELL_LABELS }, \   
*/

/*
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "QNf",        "TimeUS,Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  GPS_FMT, GPS_LABELS }, \
    { LOG_GPS2_MSG, sizeof(log_GPS), \
      "GPS2", GPS_FMT, GPS_LABELS }, \
    { LOG_GPS_HEADINGA_MSG, sizeof(log_GPS_HEADINGA), \
      "RTK",  RTK_FMT, RTK_LABELS }, \
    { LOG_GPS2_HEADINGA_MSG, sizeof(log_GPS_HEADINGA), \
      "RTK2", RTK_FMT, RTK_LABELS }, \
    { LOG_TWP_MSG, sizeof(log_TWP), \
      "TWP",  TWP_FMT, TWP_LABELS }, \
    { LOG_GPSB_MSG, sizeof(log_GPS), \
      "GPSB", GPS_FMT, GPS_LABELS }, \
    { LOG_GPA_MSG,  sizeof(log_GPA), \
      "GPA",  GPA_FMT, GPA_LABELS }, \
    { LOG_GPA2_MSG, sizeof(log_GPA), \
      "GPA2", GPA_FMT, GPA_LABELS }, \
    { LOG_GPAB_MSG, sizeof(log_GPA), \
      "GPAB", GPA_FMT, GPA_LABELS }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  IMU_FMT,     IMU_LABELS }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
	{ LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qf",     "TimeUS,RXRSSI" }, \
	{ LOG_SPRAYER_MSG, sizeof(log_SPRAYER), \
      "SPY",  "QbBBBIIhIBB",     "TimeMS,ab,run,spy,tt,ot,ut,pr,wp,fm,pc" }, \
    { LOG_CD_MSG, sizeof(log_Communication_drops), \
	 "CD", "Qif", "TMS,D2H,CD" }, \
	{ LOG_PADCMD_MSG, sizeof(log_PADCMD), \
	 "PAD", "QI", "TMS,MID" }, \
	{ LOG_MTR_MSG, sizeof(log_MTR), \
     "MTR",  "QffffffffffBBBB",     "TimeMS,r,p,t,y,thr,ya,rpyl,rpyh,thrb,sc,r,p,tl,tu" }, \
    { LOG_CONTROL_MSG, sizeof(log_Control), \
      "CTR",  "QBBBH",     "TimeMS,ov,oa,rv,rc3" }, \
    { LOG_GKPROX_MSG, sizeof(log_GKProx), \
      "GKPX",  "QB",     "TimeMS,t" }, \
	{ LOG_PMBUS0_MSG, sizeof(log_BCBPMBus), \
	 "PM0", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS1_MSG, sizeof(log_BCBPMBus), \
	 "PM1", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS2_MSG, sizeof(log_BCBPMBus), \
	 "PM2", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS3_MSG, sizeof(log_BCBPMBus), \
	 "PM3", PMBUS_FMT, PMBUS_LABELS }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  BARO_FMT, BARO_LABELS }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffH","TimeUS,Vcc,VServo,Flags" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHfffffff","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), \
      "ARSP",  "QffcffB",  "TimeUS,Airspeed,DiffPress,Temp,RawPress,Offset,U" }, \
    { LOG_CURRENT_MSG, sizeof(log_Current), \
      "BAT", CURR_FMT,CURR_LABELS }, \
    { LOG_CURRENT2_MSG, sizeof(log_Current), \
      "BAT2", CURR_FMT,CURR_LABELS }, \
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "QccccCCCC", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", MAG_FMT,    MAG_LABELS }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QCBCBCB", "TimeUS,Dt1,Ot1,Dt2,Ot2,Dt3,Ot3" }, \
    { LOG_DF_MAV_STATS, sizeof(log_DF_MAV_Stats), \
      "DMS", "IIIIIBBBBBBBBBB",         "TimeMS,N,Dp,RT,RS,Er,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx" }, \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
      "BCN", "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ" }, \
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity), \
      "PRX", "QBfffffffffff", "TimeUS,Health,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis" }

	#define LOG_EXTRA_STRUCTURES \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  IMU_FMT,     IMU_LABELS }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  IMU_FMT,     IMU_LABELS }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
      "POS","QLLfff","TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4" }, \
    { LOG_NKF1_MSG, sizeof(log_EKF1), \
      "NKF1","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_NKF5_MSG, sizeof(log_NKF5), \
      "NKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos" }, \
    { LOG_NKF6_MSG, sizeof(log_EKF1), \
      "NKF6","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_NKF7_MSG, sizeof(log_NKF2), \
      "NKF7","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF8_MSG, sizeof(log_NKF3), \
      "NKF8","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF9_MSG, sizeof(log_NKF4), \
      "NKF9","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_NKF10_MSG, sizeof(log_RngBcnDebug), \
      "NKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD" }, \
    { LOG_NKQ1_MSG, sizeof(log_Quaternion), "NKQ1", QUAT_FMT, QUAT_LABELS }, \
    { LOG_NKQ2_MSG, sizeof(log_Quaternion), "NKQ2", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKF1_MSG, sizeof(log_EKF1), \
      "XKF1","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_XKF2_MSG, sizeof(log_NKF2a), \
      "XKF2","QccccchhhhhhB","TimeUS,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_XKF3_MSG, sizeof(log_NKF3), \
      "XKF3","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_XKF4_MSG, sizeof(log_NKF4), \
      "XKF4","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_XKF5_MSG, sizeof(log_NKF5), \
      "XKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos" }, \
    { LOG_XKF6_MSG, sizeof(log_EKF1), \
      "XKF6","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_XKF7_MSG, sizeof(log_NKF2a), \
      "XKF7","QccccchhhhhhB","TimeUS,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_XKF8_MSG, sizeof(log_NKF3), \
      "XKF8","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_XKF9_MSG, sizeof(log_NKF4), \
      "XKF9","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_XKF10_MSG, sizeof(log_RngBcnDebug), \
      "XKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD" }, \
    { LOG_XKQ1_MSG, sizeof(log_Quaternion), "XKQ1", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKQ2_MSG, sizeof(log_Quaternion), "XKQ2", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKFD_MSG, sizeof(log_ekfBodyOdomDebug), \
      "XKFD","Qffffff","TimeUS,IX,IY,IZ,IVX,IVY,IVZ" }, \
    { LOG_XKV1_MSG, sizeof(log_ekfStateVar), \
      "XKV1","Qffffffffffff","TimeUS,V00,V01,V02,V03,V04,V05,V06,V07,V08,V09,V10,V11" }, \
    { LOG_XKV2_MSG, sizeof(log_ekfStateVar), \
      "XKV2","Qffffffffffff","TimeUS,V12,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded" }, \
    { LOG_GPS_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS2_UBX1_MSG, sizeof(log_Ubx1), \
      "UBY1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS2_UBX2_MSG, sizeof(log_Ubx2), \
      "UBY2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli" }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat" }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk" }, \
    { LOG_GPS_SBF_EVENT_MSG, sizeof(log_GPS_SBF_EVENT), \
      "SBFE", "QIHBBdddfffff", "TimeUS,TOW,WN,Mode,Err,Lat,Lng,Height,Undul,Vn,Ve,Vu,COG" }, \
    { LOG_ESC1_MSG, sizeof(log_Esc), \
      "ESC1",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC2_MSG, sizeof(log_Esc), \
      "ESC2",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC3_MSG, sizeof(log_Esc), \
      "ESC3",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC4_MSG, sizeof(log_Esc), \
      "ESC4",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC5_MSG, sizeof(log_Esc), \
      "ESC5",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC6_MSG, sizeof(log_Esc), \
      "ESC6",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC7_MSG, sizeof(log_Esc), \
      "ESC7",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC8_MSG, sizeof(log_Esc), \
      "ESC8",  ESC_FMT, ESC_LABELS }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2",MAG_FMT,    MAG_LABELS }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3",MAG_FMT,    MAG_LABELS }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", ACC_FMT,        ACC_LABELS }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", ACC_FMT,        ACC_LABELS }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", ACC_FMT,        ACC_LABELS }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", GYR_FMT,        GYR_LABELS }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", GYR_FMT,        GYR_LABELS }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", GYR_FMT,        GYR_LABELS }, \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", PID_FMT,  PID_LABELS }, \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", PID_FMT,  PID_LABELS }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", PID_FMT,  PID_LABELS }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", PID_FMT,  PID_LABELS }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", PID_FMT,  PID_LABELS }, \
    { LOG_PIDL_MSG, sizeof(log_PID), \
      "PIDL", PID_FMT,  PID_LABELS }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  BARO_FMT, BARO_LABELS }, \
    { LOG_BAR3_MSG, sizeof(log_BARO), \
      "BAR3",  BARO_FMT, BARO_LABELS }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QfffIII",     "TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2" }, \
    { LOG_IMUDT_MSG, sizeof(log_IMUDT), \
      "IMT",IMT_FMT,IMT_LABELS }, \
    { LOG_IMUDT2_MSG, sizeof(log_IMUDT), \
      "IMT2",IMT_FMT,IMT_LABELS }, \
    { LOG_IMUDT3_MSG, sizeof(log_IMUDT), \
      "IMT3",IMT_FMT,IMT_LABELS }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
      "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2" }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
      "RATE", "Qffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut" }, \
    { LOG_RALLY_MSG, sizeof(log_Rally), \
      "RALY", "QBBLLh", "TimeUS,Tot,Seq,Lat,Lng,Alt" }, \
    { LOG_VISUALODOM_MSG, sizeof(log_VisualOdom), \
      "VISO", "Qffffffff", "TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf" }

*/


// messages for all boards
#define LOG_BASE_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "QNf",        "TimeUS,Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  GPS_FMT, GPS_LABELS }, \
    { LOG_GPS2_MSG, sizeof(log_GPS), \
      "GPS2", GPS_FMT, GPS_LABELS }, \
    { LOG_GPS_HEADINGA_MSG, sizeof(log_GPS_HEADINGA), \
      "RTK",  RTK_FMT, RTK_LABELS }, \
    { LOG_GPS2_HEADINGA_MSG, sizeof(log_GPS_HEADINGA), \
      "RTK2", RTK_FMT, RTK_LABELS }, \
    { LOG_TWP_MSG, sizeof(log_TWP), \
      "TWP",  TWP_FMT, TWP_LABELS }, \
	{ LOG_SENSOR_MSG, sizeof(log_Sensor), \
	  "SEN",  "Qffffffffffff",  "TimeUS,CO,SO2,NO2,O3,X1,X2,CO2,PM1,PM25,PM10,TEMP,HUM" }, \
    { LOG_GPSB_MSG, sizeof(log_GPS), \
      "GPSB", GPS_FMT, GPS_LABELS }, \
    { LOG_GPA_MSG,  sizeof(log_GPA), \
      "GPA",  GPA_FMT, GPA_LABELS }, \
    { LOG_GPA2_MSG, sizeof(log_GPA), \
      "GPA2", GPA_FMT, GPA_LABELS }, \
    { LOG_GPAB_MSG, sizeof(log_GPA), \
      "GPAB", GPA_FMT, GPA_LABELS }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  IMU_FMT,     IMU_LABELS }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "QZ",     "TimeUS,Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "QHHHHHHHHHHHHHH",     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14" }, \
	{ LOG_RSSI_MSG, sizeof(log_RSSI), \
      "RSSI",  "Qf",     "TimeUS,RXRSSI" }, \
	{ LOG_SPRAYER_MSG, sizeof(log_SPRAYER), \
      "SPY",  "QbBBBIIHIBBBddH",     "TimeMS,ab,run,spy,tt,ot,ut,pr,wp,se,fm,pc,vol,h,sa" }, \
    { LOG_CD_MSG, sizeof(log_Communication_drops), \
	 "CD", "Qif", "TMS,D2H,CD" }, \
	{ LOG_PADCMD_MSG, sizeof(log_PADCMD), \
	 "PAD", "QI", "TMS,MID" }, \
	{ LOG_MTR_MSG, sizeof(log_MTR), \
     "MTR",  "QffffffffffBBBB",     "TimeMS,r,p,t,y,thr,ya,rpyl,rpyh,thrb,sc,r1,p1,tl,tu" }, \
    { LOG_CONTROL_MSG, sizeof(log_Control), \
      "CTR",  "QBBBH",     "TimeMS,ov,oa,rv,rc3" }, \
    { LOG_GKPROX_MSG, sizeof(log_GKProx), \
      "GKPX",  "QBffffffBBHHi", "TimeUS,Health,front,back,CMDO,CAn,CDis,Vnum,R1W,R3W,unc,err,TAn"}, \
	{ LOG_PMBUS0_MSG, sizeof(log_BCBPMBus), \
	 "PM0", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS1_MSG, sizeof(log_BCBPMBus), \
	 "PM1", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS2_MSG, sizeof(log_BCBPMBus), \
	 "PM2", PMBUS_FMT, PMBUS_LABELS }, \
	{ LOG_PMBUS3_MSG, sizeof(log_BCBPMBus), \
	 "PM3", PMBUS_FMT, PMBUS_LABELS }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  BARO_FMT, BARO_LABELS }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","QffH","TimeUS,Vcc,VServo,Flags" },  \
    { LOG_CMD_MSG, sizeof(log_Cmd), \
      "CMD", "QHHHfffffffB","TimeUS,CTot,CNum,CId,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,fm" }, \
    { LOG_RADIO_MSG, sizeof(log_Radio), \
      "RAD", "QBBBBBHH", "TimeUS,RSSI,RemRSSI,TxBuf,Noise,RemNoise,RxErrors,Fixed" }, \
    { LOG_CAMERA_MSG, sizeof(log_Camera), \
      "CAM", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_TRIGGER_MSG, sizeof(log_Camera), \
      "TRIG", "QIHLLeeeccC","TimeUS,GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,GPSAlt,Roll,Pitch,Yaw" }, \
    { LOG_ARSP_MSG, sizeof(log_AIRSPEED), \
      "ARSP",  "QffcffB",  "TimeUS,Airspeed,DiffPress,Temp,RawPress,Offset,U" }, \
    { LOG_CURRENT_MSG, sizeof(log_Current), \
      "BAT", CURR_FMT,CURR_LABELS }, \
    { LOG_CURRENT2_MSG, sizeof(log_Current), \
      "BAT2", CURR_FMT,CURR_LABELS }, \
	{ LOG_ATTITUDE_MSG, sizeof(log_Attitude),\
      "ATT", "QccccCCCC", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw" }, \
    { LOG_COMPASS_MSG, sizeof(log_Compass), \
      "MAG", MAG_FMT,    MAG_LABELS }, \
    { LOG_MODE_MSG, sizeof(log_Mode), \
      "MODE", "QMBB",         "TimeUS,Mode,ModeNum,Rsn" }, \
    { LOG_RFND_MSG, sizeof(log_RFND), \
      "RFND", "QCBCBCBCCCB", "TimeUS,Dt1,Ot1,Dt2,Ot2,Dt3,Ot3,Err1,NT1,unv1,Cod1" }, \
    { LOG_DF_MAV_STATS, sizeof(log_DF_MAV_Stats), \
      "DMS", "IIIIIBBBBBBBBBB",         "TimeMS,N,Dp,RT,RS,Er,Fa,Fmn,Fmx,Pa,Pmn,Pmx,Sa,Smn,Smx" }, \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
      "BCN", "QBBfffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ" }, \
    { LOG_PROXIMITY_MSG, sizeof(log_Proximity), \
      "PRX", "QBfffffffffff", "TimeUS,Health,D0,D45,D90,D135,D180,D225,D270,D315,DUp,CAn,CDis" }

// messages for more advanced boards
#define LOG_EXTRA_STRUCTURES \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  IMU_FMT,     IMU_LABELS }, \
    { LOG_IMU3_MSG, sizeof(log_IMU), \
      "IMU3",  IMU_FMT,     IMU_LABELS }, \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4" }, \
    { LOG_POS_MSG, sizeof(log_POS), \
      "POS","QLLfff","TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","QccCfLLffff","TimeUS,Roll,Pitch,Yaw,Alt,Lat,Lng,Q1,Q2,Q3,Q4" }, \
    { LOG_NKF1_MSG, sizeof(log_EKF1), \
      "NKF1","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_NKF2_MSG, sizeof(log_NKF2), \
      "NKF2","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF3_MSG, sizeof(log_NKF3), \
      "NKF3","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF4_MSG, sizeof(log_NKF4), \
      "NKF4","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_NKF5_MSG, sizeof(log_NKF5), \
      "NKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos" }, \
    { LOG_NKF6_MSG, sizeof(log_EKF1), \
      "NKF6","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_NKF7_MSG, sizeof(log_NKF2), \
      "NKF7","QbccccchhhhhhB","TimeUS,AZbias,GSX,GSY,GSZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_NKF8_MSG, sizeof(log_NKF3), \
      "NKF8","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_NKF9_MSG, sizeof(log_NKF4), \
      "NKF9","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_NKF10_MSG, sizeof(log_RngBcnDebug), \
      "NKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD" }, \
    { LOG_NKQ1_MSG, sizeof(log_Quaternion), "NKQ1", QUAT_FMT, QUAT_LABELS }, \
    { LOG_NKQ2_MSG, sizeof(log_Quaternion), "NKQ2", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKF1_MSG, sizeof(log_EKF1), \
      "XKF1","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_XKF2_MSG, sizeof(log_NKF2a), \
      "XKF2","QccccchhhhhhB","TimeUS,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_XKF3_MSG, sizeof(log_NKF3), \
      "XKF3","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_XKF4_MSG, sizeof(log_NKF4), \
      "XKF4","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_XKF5_MSG, sizeof(log_NKF5), \
      "XKF5","QBhhhcccCCfff","TimeUS,NI,FIX,FIY,AFI,HAGL,offset,RI,rng,Herr,eAng,eVel,ePos" }, \
    { LOG_XKF6_MSG, sizeof(log_EKF1), \
      "XKF6","QccCfffffffccce","TimeUS,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH" }, \
    { LOG_XKF7_MSG, sizeof(log_NKF2a), \
      "XKF7","QccccchhhhhhB","TimeUS,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,MI" }, \
    { LOG_XKF8_MSG, sizeof(log_NKF3), \
      "XKF8","Qcccccchhhcc","TimeUS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT" }, \
    { LOG_XKF9_MSG, sizeof(log_NKF4), \
      "XKF9","QcccccfbbHBHHb","TimeUS,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI" }, \
    { LOG_XKF10_MSG, sizeof(log_RngBcnDebug), \
      "XKF0","QBccCCcccccccc","TimeUS,ID,rng,innov,SIV,TR,BPN,BPE,BPD,OFH,OFL,OFN,OFE,OFD" }, \
    { LOG_XKQ1_MSG, sizeof(log_Quaternion), "XKQ1", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKQ2_MSG, sizeof(log_Quaternion), "XKQ2", QUAT_FMT, QUAT_LABELS }, \
    { LOG_XKFD_MSG, sizeof(log_ekfBodyOdomDebug), \
      "XKFD","Qffffff","TimeUS,IX,IY,IZ,IVX,IVY,IVZ" }, \
    { LOG_XKV1_MSG, sizeof(log_ekfStateVar), \
      "XKV1","Qffffffffffff","TimeUS,V00,V01,V02,V03,V04,V05,V06,V07,V08,V09,V10,V11" }, \
    { LOG_XKV2_MSG, sizeof(log_ekfStateVar), \
      "XKV2","Qffffffffffff","TimeUS,V12,V13,V14,V15,V16,V17,V18,V19,V20,V21,V22,V23" }, \
    { LOG_TERRAIN_MSG, sizeof(log_TERRAIN), \
      "TERR","QBLLHffHH","TimeUS,Status,Lat,Lng,Spacing,TerrH,CHeight,Pending,Loaded" }, \
    { LOG_GPS_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS2_UBX1_MSG, sizeof(log_Ubx1), \
      "UBY1", "QBHBBH",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt" }, \
    { LOG_GPS2_UBX2_MSG, sizeof(log_Ubx2), \
      "UBY2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ" }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli" }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat" }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk" }, \
    { LOG_GPS_SBF_EVENT_MSG, sizeof(log_GPS_SBF_EVENT), \
      "SBFE", "QIHBBdddfffff", "TimeUS,TOW,WN,Mode,Err,Lat,Lng,Height,Undul,Vn,Ve,Vu,COG" }, \
    { LOG_ESC1_MSG, sizeof(log_Esc), \
      "ESC1",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC2_MSG, sizeof(log_Esc), \
      "ESC2",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC3_MSG, sizeof(log_Esc), \
      "ESC3",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC4_MSG, sizeof(log_Esc), \
      "ESC4",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC5_MSG, sizeof(log_Esc), \
      "ESC5",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC6_MSG, sizeof(log_Esc), \
      "ESC6",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC7_MSG, sizeof(log_Esc), \
      "ESC7",  ESC_FMT, ESC_LABELS }, \
    { LOG_ESC8_MSG, sizeof(log_Esc), \
      "ESC8",  ESC_FMT, ESC_LABELS }, \
    { LOG_COMPASS2_MSG, sizeof(log_Compass), \
      "MAG2",MAG_FMT,    MAG_LABELS }, \
    { LOG_COMPASS3_MSG, sizeof(log_Compass), \
      "MAG3",MAG_FMT,    MAG_LABELS }, \
    { LOG_ACC1_MSG, sizeof(log_ACCEL), \
      "ACC1", ACC_FMT,        ACC_LABELS }, \
    { LOG_ACC2_MSG, sizeof(log_ACCEL), \
      "ACC2", ACC_FMT,        ACC_LABELS }, \
    { LOG_ACC3_MSG, sizeof(log_ACCEL), \
      "ACC3", ACC_FMT,        ACC_LABELS }, \
    { LOG_GYR1_MSG, sizeof(log_GYRO), \
      "GYR1", GYR_FMT,        GYR_LABELS }, \
    { LOG_GYR2_MSG, sizeof(log_GYRO), \
      "GYR2", GYR_FMT,        GYR_LABELS }, \
    { LOG_GYR3_MSG, sizeof(log_GYRO), \
      "GYR3", GYR_FMT,        GYR_LABELS }, \
    { LOG_PIDR_MSG, sizeof(log_PID), \
      "PIDR", PID_FMT,  PID_LABELS }, \
    { LOG_PIDP_MSG, sizeof(log_PID), \
      "PIDP", PID_FMT,  PID_LABELS }, \
    { LOG_PIDY_MSG, sizeof(log_PID), \
      "PIDY", PID_FMT,  PID_LABELS }, \
    { LOG_PIDA_MSG, sizeof(log_PID), \
      "PIDA", PID_FMT,  PID_LABELS }, \
    { LOG_PIDS_MSG, sizeof(log_PID), \
      "PIDS", PID_FMT,  PID_LABELS }, \
    { LOG_PIDL_MSG, sizeof(log_PID), \
      "PIDL", PID_FMT,  PID_LABELS }, \
    { LOG_BAR2_MSG, sizeof(log_BARO), \
      "BAR2",  BARO_FMT, BARO_LABELS }, \
    { LOG_BAR3_MSG, sizeof(log_BARO), \
      "BAR3",  BARO_FMT, BARO_LABELS }, \
    { LOG_VIBE_MSG, sizeof(log_Vibe), \
      "VIBE", "QfffIII",     "TimeUS,VibeX,VibeY,VibeZ,Clip0,Clip1,Clip2" }, \
    { LOG_IMUDT_MSG, sizeof(log_IMUDT), \
      "IMT",IMT_FMT,IMT_LABELS }, \
    { LOG_IMUDT2_MSG, sizeof(log_IMUDT), \
      "IMT2",IMT_FMT,IMT_LABELS }, \
    { LOG_IMUDT3_MSG, sizeof(log_IMUDT), \
      "IMT3",IMT_FMT,IMT_LABELS }, \
    { LOG_ORGN_MSG, sizeof(log_ORGN), \
      "ORGN","QBLLe","TimeUS,Type,Lat,Lng,Alt" }, \
    { LOG_RPM_MSG, sizeof(log_RPM), \
      "RPM",  "Qff", "TimeUS,rpm1,rpm2" }, \
    { LOG_RATE_MSG, sizeof(log_Rate), \
      "RATE", "Qffffffffffff",  "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut" }, \
    { LOG_RALLY_MSG, sizeof(log_Rally), \
      "RALY", "QBBLLh", "TimeUS,Tot,Seq,Lat,Lng,Alt" }, \
	{ LOG_LD_MSG, sizeof(log_Land_Detector), \
      "LD", "QBBBBffB", "TimeUS,lc,mlc,mtl,atm,al,dr,r" }


// #if SBP_HW_LOGGING
#define LOG_SBP_STRUCTURES \
    { LOG_MSG_SBPHEALTH, sizeof(log_SbpHealth), \
      "SBPH", "QIII", "TimeUS,CrcError,LastInject,IARhyp" }, \
    { LOG_MSG_SBPRAWH, sizeof(log_SbpRAWH), \
      "SBRH", "QQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6" }, \
    { LOG_MSG_SBPRAWM, sizeof(log_SbpRAWM), \
      "SBRM", "QQQQQQQQQQQQQQQ", "TimeUS,msg_flag,1,2,3,4,5,6,7,8,9,10,11,12,13" }, \
    { LOG_MSG_SBPEVENT, sizeof(log_SbpEvent), \
      "SBRE", "QHIiBB", "TimeUS,GWk,GMS,ns_residual,level,quality" }
// #endif

#define LOG_COMMON_STRUCTURES LOG_BASE_STRUCTURES, LOG_EXTRA_STRUCTURES, LOG_SBP_STRUCTURES

// message types 0 to 128 reversed for vehicle specific use





enum LogOriginType {
    ekf_origin = 0,
    ahrs_home = 1
};
