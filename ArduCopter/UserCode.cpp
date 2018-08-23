#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    //	added by ZhangYong
    //	get board id
#if FXTX_AUTH == 1
		char serial_id[AUTH_ID_LEN];
		char lcl_char_l;
		char lcl_char_h;
		uint8_t lcl_cnt;
		uint8_t lcl_out_cnt;

    	memset(auth_msg, 0, 50);
    	memset(serial_id, 0, AUTH_ID_LEN);
		memset(auth_id, 0, AUTH_ID_LEN);

    	memset(&curr_gps_week_ms, 0, sizeof(struct current_gps_week_ms));
    //	memset(&id_para, 0, sizeof(union auth_id_para));


    	(void)hal.util->get_system_id(serial_id);


		for(lcl_out_cnt = 0; lcl_out_cnt < 3; lcl_out_cnt ++)
		{
			for(lcl_cnt = 0; lcl_cnt < 4; lcl_cnt++)
			{
				lcl_char_h = serial_id[6 + lcl_cnt *2 + lcl_out_cnt * 9];

//				printf("%d\n", 6 + lcl_cnt *2 + lcl_out_cnt * 9);


				if(lcl_char_h < 0x40)
					lcl_char_h -= 0x30;
				else
					lcl_char_h -= 0x37;

				lcl_char_h = lcl_char_h << 4;


				lcl_char_l = serial_id[6 + lcl_cnt *2 + 1 + lcl_out_cnt * 9];

//				printf("%d\n", 6 + lcl_cnt *2 + 1 + lcl_out_cnt * 9);


				if(lcl_char_l < 0x40)
					lcl_char_l -= 0x30;
				else
					lcl_char_l -= 0x37;

				lcl_char_h |= lcl_char_l;

				auth_id[lcl_cnt + lcl_out_cnt * 4] = lcl_char_h;
	//			printf("%02x\n", auth_id[lcl_cnt + lcl_out_cnt * 4]);
			}
		}


    	sprintf(auth_msg, "0123456789%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",	\
    		     (unsigned)serial_id[6], 	(unsigned)serial_id[7], 	(unsigned)serial_id[8], 	(unsigned)serial_id[9], 	(unsigned)serial_id[10], (unsigned)serial_id[11],	(unsigned)serial_id[12], (unsigned)serial_id[13], \
    		     (unsigned)serial_id[15], 	(unsigned)serial_id[16], 	(unsigned)serial_id[17], 	(unsigned)serial_id[18], 	(unsigned)serial_id[19], (unsigned)serial_id[20], 	(unsigned)serial_id[21], (unsigned)serial_id[22], \
    		     (unsigned)serial_id[24], 	(unsigned)serial_id[25], 	(unsigned)serial_id[26], 	(unsigned)serial_id[27], 	(unsigned)serial_id[28], (unsigned)serial_id[29], 	(unsigned)serial_id[30], (unsigned)serial_id[31]);


		//	20180418		 
		printf("0123456789%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",	 \
								  (unsigned)serial_id[6],	 (unsigned)serial_id[7],	 (unsigned)serial_id[8],	 (unsigned)serial_id[9],	 (unsigned)serial_id[10], (unsigned)serial_id[11],	 (unsigned)serial_id[12], (unsigned)serial_id[13], \
								  (unsigned)serial_id[15],	 (unsigned)serial_id[16],	 (unsigned)serial_id[17],	 (unsigned)serial_id[18],	 (unsigned)serial_id[19], (unsigned)serial_id[20],	 (unsigned)serial_id[21], (unsigned)serial_id[22], \
								  (unsigned)serial_id[24],	 (unsigned)serial_id[25],	 (unsigned)serial_id[26],	 (unsigned)serial_id[27],	 (unsigned)serial_id[28], (unsigned)serial_id[29],	 (unsigned)serial_id[30], (unsigned)serial_id[31]);
		
			
		memset(&fs_mk, 0, sizeof(FAILSAFE_MARKER));


#endif		//	FXTX_AUTH == 1
    //	added end

	//	added by zhangyong for gcs_mavlink chan number
	printf("Mavlink channel number: %d\n", gcs().num_gcs());
	//	added end

#if FXTX_AUTH == 0
	printf("License Disabled\n");
	//	added by ZhangYong 20170705 for item_int
	gcs().chan(0).set_mission_item_int(false);
	gcs().chan(1).set_mission_item_int(false);
#if MAVLINK_COMM_NUM_BUFFERS > 2
	gcs().chan(2).set_mission_item_int(false);
#endif //	end MAVLINK_COMM_NUM_BUFFERS > 2
	//	added end

#elif FXTX_AUTH == 1
	printf("License Enabled\n");

	//	added by ZhangYong 20170705 for item_int
	gcs().chan(0).set_mission_item_int(true);
	gcs().chan(1).set_mission_item_int(true);
#if MAVLINK_COMM_NUM_BUFFERS > 2
	gcs().chan(2).set_mission_item_int(true);
#endif // end MAVLINK_COMM_NUM_BUFFERS > 2
	//	added end

#endif // end FXTX_AUTH == 0

	printf("WAYPOINT[%d] INT %d\n", 0, gcs().chan(0).get_mission_item_int());
	printf("WAYPOINT[%d] INT %d\n", 1, gcs().chan(1).get_mission_item_int());


    //	added by ZhangYong 20170705
    //printf("Mavlink capabilities %x\n", hal.util->get_capabilities());
    //	added end
		//	added by ZhangYong 20170712
#if PJTPASSOSD == ENABLED
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_PassOSD, 0)))
	{
		passosd.init(serial_manager);
	}
	else
	{

	}
#endif // end PJTPASSOSD == ENABLED

#if PROJECTGKXN == ENABLED
	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_FlowMeter_GKXN,0)))
	{
		flowmeter.init(serial_manager);
	}
	//height_replace = 0;

#endif // endf PROJECTGKXN == ENABLED

#if BCBMONITOR == ENABLED

	if(nullptr != (serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBMonitor,0)))
	{
		bcbmonitor.init(serial_manager);
	}

#endif // end BCBMONITOR == ENABLED

#if BCBPMBUS == ENABLED
	printf("PROJECTBCB\n");
	printf("PMBUS\n");

	if(nullptr !=  serial_manager.find_serial(AP_SerialManager::SerialProtocol_BCBPMBus,0))
	{
		init_bcbpmbus();
	}
#endif // end BCBPMBUS == ENABLED

#if PJTPASSOSD == ENABLED
	printf("PJTPASSOSD\n");
#endif // end PJTPASSOSD == ENABLED



#if PROJECTGKXN == ENABLED
	printf("PROJECTGKXN\n");
#endif // end PROJECTGKXN == ENABLED
	
	//	added by ZhangYong
#if PROJECTFB == ENABLED
	printf("PROJECTFB\n");
#endif	
	//	added ebd

	//	added by ZhangYong 20170705
	printf("Mavlink capabilities %x\n", hal.util->get_capabilities());
	//	added end


    /*edit_management.words = g.edition_management;

    printf("major_edition = 0x%x\n", edit_management.data.major_edition);
    printf("project_edition = 0x%x\n", edit_management.data.project_edition);
    printf("minor_edition = 0x%x\n", edit_management.data.minor_edition);
    printf("revision_edition = 0x%x\n", edit_management.data.revision_edition);

    if(1 != edit_management.data.major_edition)
    {
    	edit_management.data.major_edition = 2;

    	g.edition_management.set_and_save(edit_management.words);
    }*/
	edit_management.words = g.edition_management;

	if(edit_management.data.major_edition != 2)
	{
		edit_management.data.major_edition = 2;
		g.edition_management.set_and_save(edit_management.words);
	}

#if PROJECTGKXN == ENABLED
	if(edit_management.data.project_edition != 1)
	{
		edit_management.data.project_edition = 1;
		g.edition_management.set_and_save(edit_management.words);
	}
#endif	// end PROJECTGKXN == ENABLED

	//	improve the minor edition from 3 to 4
	//	failsafe rc gcs
	//	improve the minor edition from 4 to 5
	//	ABPoint, dual GPS antenna heaidng
	//	improve the minor edition from 5 to 6
	//	FXTX_AUTH debug get_system_id

	//	improve the minor edition from 6 to 7
	//	resetHeightDatum
	if(edit_management.data.minor_edition != 7)
	{	
		edit_management.data.minor_edition = 7;
		g.edition_management.set_and_save(edit_management.words);
	}

	
	//	4G communications
	//	rewrite landinggear
	//	ab mode throttle adjustable
	//	AP_ARMING_COMPASS_MAGFIELD_MAX 1100
	//	MAX_LOG_FILES	100
	//  misson recovery onboard
	//	AB mode altitude control in terrain follow
	//	spraying operation in ABmode
	if(edit_management.data.revision_edition != 11)
	{	
		edit_management.data.revision_edition = 11;
		g.edition_management.set_and_save(edit_management.words);
	}


	printf("major_edition = 0x%x\n", edit_management.data.major_edition);
	printf("project_edition = 0x%x\n", edit_management.data.project_edition);
	printf("minor_edition = 0x%x\n", edit_management.data.minor_edition);
	printf("revision_edition = 0x%x\n", edit_management.data.revision_edition);


#if PTZ_CONTROL == ENABLED
    PtzControl.init();
#endif

#if CHARGINGSTATION == ENABLED
    chargingStation.init();
#endif

#if ABMODE == ENABLED
	rf_abmode.init();
#endif

#if NEWBROADCAST == ENABLED
   newbroadcast.init();
#endif


	//	added by zhangyong for short or long edge judge 20180705
	//wpnav_destination_settled = false;
	//	added end

}
#endif // end USERHOOK_INIT

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif // end USERHOOK_FASTLOOP

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here

    //baiyang added in 20170804
#if PTZ_CONTROL == ENABLED
	//	modified by zhangyong to control the shutter 20180612
	  PtzControl.update(0xff);
#endif // end PTZ_CONTROL == ENABLED
    //added end

#if NEWBROADCAST == ENABLED
	newbroadcast.update();
#endif

}
#endif // end USERHOOK_50HZLOOP

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
#if CHARGINGSTATION == ENABLED
    chargingStation.update();
#endif

#if ABMODE == ENABLED
	rf_abmode.update();
#endif

#if NEWBROADCAST == ENABLED
	newbroadcast.update_view();
#endif

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
#if DGPS_HEADINGA == ENABLED
    if(!EKF2.get_gps_heading_health())
    {
    	set_failsafe_gps_head(true);
    }
    else
    {
    	set_failsafe_gps_head(false);
    }
#endif
    
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
#if FXTX_AUTH== ENABLED
	 if(motors->armed())
	 {
		  local_flight_time_sec++;
	 }//	added end
#endif

	if (!motors->armed())
	{
		//printf("reason err: %d\n",mission.regenerate_airline());
		mission.regenerate_airline();
	}
}
#endif



