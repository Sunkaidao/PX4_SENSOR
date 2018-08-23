#include <AP_Gassensor/AP_Gassensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>





extern const AP_HAL::HAL& hal;

/*
const AP_Param::GroupInfo AP_Flowmeter::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the flowmeter
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AP_Flowmeter, _enabled, 0),

    AP_GROUPEND
};
*/

AP_Gassensor::AP_Gassensor()
{
	_initialised = false;

	//AP_Param::setup_object_defaults(this, var_info);

//	printf("0.%d", _initialised);
	
}

AP_Gassensor::~AP_Gassensor()
{}


void AP_Gassensor::init(const AP_SerialManager& serial_manager)
{
	//if(0 == _enabled)
		//return;
	int i;
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Airsensor, 0);

	if(uart != nullptr)
	{
		
	for(i=0;i<28;i++)
		rx_data12[i]=0;
	_initialised = true;


	}

//	printf("1.%d\n", _initialised);
}


void AP_Gassensor::SendCMD(uint8_t CMD)
{
	uint8_t checksum_msb=0;
	uint8_t checksum_lsb=0;
	uint8_t i;
	uint16_t crc=0xFFFF;
	uint8_t b;
	
	Tx_Buff[0]=0x01;
	Tx_Buff[1]=0x03;
	Tx_Buff[2]=0x03;
	if (CMD==0X00)
	{
	Tx_Buff[3]=0x80;
	}
	else 
	{

	Tx_Buff[3]=0x00;
	}
	Tx_Buff[4]=0x00;
	if (CMD==0X00)
	{
	Tx_Buff[5]=0x0C;
	}
	else 
	{

	Tx_Buff[5]=0x1E;
	}
	for (i = 0; i < 6; i++)
	{
		crc ^= Tx_Buff[i];
		for (b = 0; b < 8; b++)
		{
			bool f = crc & 1;
			crc >>= 1;
			crc = crc&0x7fff;
			if (f)
				crc ^= 0xa001;
		}
	}
		//printf("crc=%x\n");
	checksum_msb =( crc & 0xFF00)>>8;
	checksum_lsb = crc & 0x00FF;
	Tx_Buff[6]=checksum_lsb;
	Tx_Buff[7]=checksum_msb;
/*缺少第九位 换行位*/
	Tx_Buff[8]=0x0d;
	uint8_t a=0x0a;

	for(i=0;i<9;i++)
	{
		uart->write(Tx_Buff[i]);
	}
	
	uart->write(a);
}

void AP_Gassensor::update(const AP_SerialManager& serial_manager,DataFlash_Class DataFlash)
{
	if(!_initialised)
	{
		init(serial_manager);//serial init 
		SendCMD(onboard);//tx-03 00
		//get_sensor6();//rx-03 00
	}

	if(!_initialised)
		return;
 	
	SendCMD(fixed);//tx-03 80
	get_sensor12();//rx-03 80
	log(DataFlash);//log
	SEN_Mav_data();//Mavkink data
	
}
void AP_Gassensor::SEN_Mav_data()
{
	MAV_DATA[0]=rx_data12[3];
	MAV_DATA[1]=rx_data12[4];
	MAV_DATA[2]=rx_data12[5];
	MAV_DATA[3]=rx_data12[6];
	MAV_DATA[4]=rx_data12[7];
	MAV_DATA[5]=rx_data12[8];
	MAV_DATA[6]=rx_data12[9];
	MAV_DATA[7]=rx_data12[10];
	MAV_DATA[8]=rx_data12[11];
	MAV_DATA[9]=rx_data12[12];
	MAV_DATA[10]=rx_data12[13];
	MAV_DATA[11]=rx_data12[14];
	MAV_DATA[12]=rx_data12[15];
	MAV_DATA[13]=rx_data12[16];
	MAV_DATA[14]=rx_data12[17];
	MAV_DATA[15]=rx_data12[18];
	MAV_DATA[16]=rx_data12[19];
	MAV_DATA[17]=rx_data12[20];
	MAV_DATA[18]=rx_data12[21];
	MAV_DATA[19]=rx_data12[22];
	MAV_DATA[20]=rx_data12[23];
	MAV_DATA[21]=rx_data12[24];
	MAV_DATA[22]=rx_data12[25];
	MAV_DATA[23]=rx_data12[26];

}

void AP_Gassensor::get_sensor6()
{
	
	rxdata6_len=uart->available();
	rx_len_flag=0;
	uart->printf("\n %d \n",rxdata12_len);
	while(rx_len_flag<rxdata6_len)
	{
		rx_data6[rx_len_flag]=uart->read();
		uart->write(rx_data6[rx_len_flag]);
		rx_len_flag++;
	}
	
	
	if(CRC16(rx_data6,63)==merge(rx_data6[64],rx_data6[63]))
	{
		Sensor6_warning[0]=merge(rx_data6[3],rx_data6[4]);
		Sensor6_warning[1]=merge(rx_data6[13],rx_data6[14]);
		Sensor6_warning[2]=merge(rx_data6[23],rx_data6[24]);
		Sensor6_warning[3]=merge(rx_data6[33],rx_data6[34]);
		Sensor6_warning[4]=merge(rx_data6[43],rx_data6[44]);
		Sensor6_warning[5]=merge(rx_data6[53],rx_data6[54]);
		
	
		Sensor6_name[0]=merge(rx_data6[9],rx_data6[10]);
		Sensor6_name[1]=merge(rx_data6[19],rx_data6[20]);
		Sensor6_name[2]=merge(rx_data6[29],rx_data6[30]);
		Sensor6_name[3]=merge(rx_data6[39],rx_data6[40]);
		Sensor6_name[4]=merge(rx_data6[49],rx_data6[50]);
		Sensor6_name[5]=merge(rx_data6[59],rx_data6[60]);
		/*
		uart->write(Sensor6_name[0]);
		uart->write(Sensor6_name[1]);
		uart->write(Sensor6_name[2]);
		uart->write(Sensor6_name[3]);
		uart->write(Sensor6_name[4]);
		uart->write(Sensor6_name[5]);
		*/
		Sensor6_unit[0]=merge(rx_data6[11],rx_data6[12]);
		Sensor6_unit[1]=merge(rx_data6[21],rx_data6[22]);
		Sensor6_unit[2]=merge(rx_data6[31],rx_data6[32]);
		Sensor6_unit[3]=merge(rx_data6[41],rx_data6[42]);
		Sensor6_unit[4]=merge(rx_data6[51],rx_data6[52]);
		Sensor6_unit[5]=merge(rx_data6[61],rx_data6[62]);
		/*
		uart->write(Sensor6_unit[0]);
		uart->write(Sensor6_unit[1]);
		uart->write(Sensor6_unit[2]);
		uart->write(Sensor6_unit[3]);
		uart->write(Sensor6_unit[4]);
		uart->write(Sensor6_unit[5]);
		*/
		Sensor6_float[0]=merge(rx_data6[7],rx_data6[8]);
		Sensor6_float[1]=merge(rx_data6[17],rx_data6[18]);
		Sensor6_float[2]=merge(rx_data6[27],rx_data6[28]);
		Sensor6_float[3]=merge(rx_data6[37],rx_data6[38]);
		Sensor6_float[4]=merge(rx_data6[47],rx_data6[48]);
		Sensor6_float[5]=merge(rx_data6[57],rx_data6[58]);

	
		Sensor6_data[0]=merge(rx_data6[5],rx_data6[6])*square(0.1,Sensor6_float[0]);
		Sensor6_data[1]=merge(rx_data6[15],rx_data6[16])*square(0.1,Sensor6_float[1]);
		Sensor6_data[2]=merge(rx_data6[25],rx_data6[26])*square(0.1,Sensor6_float[2]);
		Sensor6_data[3]=merge(rx_data6[35],rx_data6[36])*square(0.1,Sensor6_float[3]);
		Sensor6_data[4]=merge(rx_data6[45],rx_data6[46])*square(0.1,Sensor6_float[4]);
		Sensor6_data[5]=merge(rx_data6[55],rx_data6[56])*square(0.1,Sensor6_float[5]);
		Sensor6_data[6]=merge(rx_data6[64],rx_data6[63]);


		uart->printf("\n%f\n%f\n%f\n%f\n%f\n%f",Sensor6_data[0],Sensor6_data[1],Sensor6_data[2],
			Sensor6_data[3],Sensor6_data[4],Sensor6_data[5]);
		uart->printf("\n%.0f\n",Sensor6_data[6]);

	}
	else
		uart->printf("error %f\n",Sensor6_data[6]);
}


void AP_Gassensor::get_sensor12()
{
	rxdata12_len=uart->available();
	rx_len_flag=0;
	uart->printf("\n %d \n",rxdata12_len);
	while(rx_len_flag<rxdata12_len)
	{
		rx_data12[rx_len_flag]=uart->read();
		//uart->write(rx_data12[rx_len_flag]);
		rx_len_flag++;
	}

	
	if(CRC16(rx_data12,27)==merge(rx_data12[28],rx_data12[27]))
	{

		Sensor12_data[0]=merge(rx_data12[3],rx_data12[4])*0.01;
		Sensor12_data[1]=merge(rx_data12[5],rx_data12[6])*0.001;
		Sensor12_data[2]=merge(rx_data12[7],rx_data12[8])*0.001;
		Sensor12_data[3]=merge(rx_data12[9],rx_data12[10])*0.001;
		Sensor12_data[4]=merge(rx_data12[11],rx_data12[12]);
		Sensor12_data[5]=merge(rx_data12[13],rx_data12[14]);
		Sensor12_data[6]=merge(rx_data12[15],rx_data12[16]);
		Sensor12_data[7]=merge(rx_data12[17],rx_data12[18]);
		Sensor12_data[8]=merge(rx_data12[19],rx_data12[20]);
		Sensor12_data[9]=merge(rx_data12[21],rx_data12[22]);
		Sensor12_data[10]=merge(rx_data12[23],rx_data12[24])*0.1;
		Sensor12_data[11]=merge(rx_data12[25],rx_data12[26])*0.1;
		Sensor12_data[12]=merge(rx_data12[28],rx_data12[27]);
	if(rx_data12[23]==0x80)
		Sensor12_data[10]=Sensor12_data[10]*(-1);
	//uart->printf("%d %d",rx_data[3],rx_data[4]);
	/*
		uart->printf("\n%.2f\n%.3f\n%.3f\n%.3f\n%.0f\n%.0f\n%.0f\n%.0f\n%.0f\n%.0f\n%.1f\n%.1f\n%f ",
		Sensor12_data[0],Sensor12_data[1],Sensor12_data[2],Sensor12_data[3],Sensor12_data[4],
		Sensor12_data[5],Sensor12_data[6],Sensor12_data[7],Sensor12_data[8],Sensor12_data[9],
		Sensor12_data[10],Sensor12_data[11],Sensor12_data[12]);
	*/
	}
	//else
		//uart->printf("error %f\n",Sensor12_data[12]);
}

void AP_Gassensor::log(DataFlash_Class DataFlash)
{
		DataFlash.Log_Write_Sensor(Sensor12_data);
}


uint32_t AP_Gassensor::merge(unsigned char high,unsigned char low)
{
	uint32_t val;
	val=(uint32_t)(high)*256+(uint32_t)low;
	return val;
}

unsigned short AP_Gassensor::CRC16(unsigned char *puchMsg, unsigned int usDataLen)  
{  
 	unsigned short wCRCin = 0xFFFF;  
    unsigned short wCPoly = 0x8005;  
    unsigned char wChar = 0;  
    
    while (usDataLen--)     
    {  
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8); 
        
        for(int i = 0; i < 8; i++)  
        {  
            if(wCRCin & 0x8000) 
			{
                 wCRCin = (wCRCin << 1) ^ wCPoly;  
             }
            else  
            {
                wCRCin = wCRCin << 1; 
            }            
        }  
     }  
     InvertUint16(&wCRCin, &wCRCin);  
	 //uart->printf("\n				%d\n",wCRCin);
     return (wCRCin) ;  

}  

void AP_Gassensor::InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf)
{
    int i;
    unsigned char temp = 0;
    
    for(i = 0; i < 8; i++)
    {
        if(SrcBuf[0] & (1 << i))
        {
            temp |= 1<<(7-i);
        }
    }
    DesBuf[0] = temp;
}
void AP_Gassensor::InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf)	
{	
	 int i;  
	 unsigned short temp = 0;	 
	 
	 for(i = 0; i < 16; i++)  
	 {	
		 if(SrcBuf[0] & (1 << i))
		 {			
			 temp |= 1<<(15 - i);  
		 }
	 }	
	 DesBuf[0] = temp;	
}

float AP_Gassensor::square(float number,uint16_t power)
{
	float num=number;
	if(power==0)
		number=1;
	else
	{
		for(;power>1;power--)
		{
			number=number*num;
		}
	}
	return number;
}
