/*************************************************
Firmware developed for AX12 servo
ATMega8 processor at 16MHz
by: Ricardo Marinheiro
February 2011
robosavvy
Move Interactive
*************************************************/
#include <avr/delay.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include "utils.h"
#include "memory.h"
#include "serial.h"

extern unsigned char MODEL_NUMBER_LOW ;
extern unsigned char MODEL_NUMBER_HIGH ;  
extern unsigned char VERSION_OF_FIRMWARE;  
extern unsigned char ID     ;
extern unsigned char BAUD_RATE;    
extern unsigned char RETURN_DELAY_TIME     ;
extern unsigned char CW_ANGLE_LIMIT_LOW     ;
extern unsigned char CW_ANGLE_LIMIT_HIGH     ;
extern unsigned char CCW_ANGLE_LIMIT_LOW     ;
extern unsigned char CCW_ANGLE_LIMIT_HIGH     ;
extern unsigned char THE_HIGHEST_LIMIT_TEMPERATUR;
extern unsigned char THE_LOWEST_LIMIT_VOLTAGE;    
extern unsigned char THE_HIGHEST_LIMIT_VOLTAGE ;  
extern unsigned char MAX_TORQUE_LOW;     
extern unsigned char MAX_TORQUE_HIGH ;   
extern unsigned char STATUS_RETURN_LEVEL;   
extern unsigned char ALARM_LED ;    
extern unsigned char ALARM_SHUTDOWN ;


extern unsigned char TORQUE_ENABLE;
extern unsigned char LED;
extern unsigned char CW_COMPLIANCE_MARGIN;
extern unsigned char CCW_COMPLIANCE_MARGIN;
extern unsigned char CW_COMPLIANCE_SLOPE;
extern unsigned char CCW_COMPLIANCE_SLOPE;
extern unsigned char GOAL_POSITION_LOW;
extern unsigned char GOAL_POSITION_HIGH;
extern unsigned char MOVING_SPEED_LOW;
extern unsigned char MOVING_SPEED_HIGH;
extern unsigned char TORQUE_LIMIT_LOW;
extern unsigned char TORQUE_LIMIT_HIGH;
extern unsigned char PRESENT_POSITION_LOW;
extern unsigned char PRESENT_POSITION_HIGH;
extern unsigned char PRESENT_SPEED_LOW;
extern unsigned char PRESENT_SPEED_HIGH;
extern unsigned char PRESENT_LOAD_LOW;
extern unsigned char PRESENT_LOAD_HIGH;
extern unsigned char PRESENT_VOLTAGE;
extern unsigned char PRESENT_TEMPERATURE;
extern unsigned char REGISTERED;
extern unsigned char MOVING;
extern unsigned char LOCK;
extern unsigned char PUNCH_LOW;
extern unsigned char PUNCH_HIGH;

extern uint8_t needDesiredSpeedUpdate;

uint8_t EESave(uint16_t adresse, uint8_t data)
{


	if(adresse==ID_addr) // no ID should be 0 or 255(brroadcast)
	{
		if((data==0) || (data==255))
			return 0;
	}
	else
	if(adresse==Baud_Rate_addr) // only the supported bauds shall be accepted
	{
		if((data!=Baud9600)&&(data!=Baud57600)&&(data!=Baud115200)&&(data!=Baud500000)&&(data!=Baud1000000)&&(data!=Baud2000000))
			return 0;
	}

	eeprom_write_byte((uint8_t*)adresse,data);

	return 1;

}



uint8_t EEWrite(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t temp = 0;
	while(temp<length)
	{
		if(EESave((address+temp),data[temp])!=1)
			return 0;
		temp++;
	}

	return 1;
}


void EERead(uint16_t address, uint8_t *data, uint8_t length)
{
	uint8_t temp = 0;
	while(temp<length)
	{
		data[temp]=eeprom_read_byte((uint8_t*)address+temp);
		temp++;
	}

}

uint8_t EEGet(uint16_t Variable)
{
	uint8_t data=0;

	data=eeprom_read_byte((uint8_t*)Variable);

	return data;
}


void EEPromReset(void)
{
  EESave(Model_Number_low_addr, Model_Number_low_default );
  EESave( Model_Number_high_addr,  Model_Number_high_default  );
  EESave( Version_of_Firmware_addr, Version_of_Firmware_default );
  EESave( ID_addr,    ID_default    );
  EESave( Baud_Rate_addr,   Baud_Rate_default   );
  EESave( Return_Delay_Time_addr,    Return_Delay_Time_default    );
  EESave( CW_Angle_Limit_low_addr,    CW_Angle_Limit_low_default    );
  EESave( CW_Angle_Limit_high_addr,    CW_Angle_Limit_high_default    );
  EESave( CCW_Angle_Limit_low_addr,    CCW_Angle_Limit_low_default    );
  EESave( CCW_Angle_Limit_high_addr,    CCW_Angle_Limit_high_default    );
  EESave( the_Highest_Limit_Temperature_addr, the_Highest_Limit_Temperature_default    );
  EESave( the_Lowest_Limit_Voltage_addr,    the_Lowest_Limit_Voltage_default    );
  EESave( the_Highest_Limit_Voltage_addr,   the_Highest_Limit_Voltage_default    );
  EESave( Max_Torque_low_addr,    Max_Torque_low_default    );
  EESave( Max_Torque_high_addr,   Max_Torque_high_default   );
  EESave( Status_Return_Level_addr,  Status_Return_Level_default  );
  EESave( Alarm_LED_addr,    Alarm_LED_default    );
  EESave( Alarm_Shutdown_addr,   Alarm_Shutdown_default);
}



void EELoadVars(void)
{
  if(EEGet(Model_Number_low_addr)!=Model_Number_low_default)
    EEPromReset();

  //Load from EEProm
  MODEL_NUMBER_LOW =EEGet( Model_Number_low_addr);
  MODEL_NUMBER_HIGH =EEGet( Model_Number_high_addr);
  VERSION_OF_FIRMWARE =EEGet(Version_of_Firmware_addr);
  ID   =EEGet(ID_addr);
  BAUD_RATE  =EEGet(Baud_Rate_addr);
  RETURN_DELAY_TIME   =EEGet( Return_Delay_Time_addr);
  CW_ANGLE_LIMIT_LOW   =EEGet( CW_Angle_Limit_low_addr);
  CW_ANGLE_LIMIT_HIGH   =EEGet(     CW_Angle_Limit_high_addr);
  CCW_ANGLE_LIMIT_LOW   =EEGet(     CCW_Angle_Limit_low_addr);
  CCW_ANGLE_LIMIT_HIGH   =EEGet(     CCW_Angle_Limit_high_addr);
  THE_HIGHEST_LIMIT_TEMPERATURE =EEGet(the_Highest_Limit_Temperature_addr);
  THE_LOWEST_LIMIT_VOLTAGE   =EEGet(     the_Lowest_Limit_Voltage_addr);
  THE_HIGHEST_LIMIT_VOLTAGE  =EEGet(    the_Highest_Limit_Voltage_addr);
  MAX_TORQUE_LOW   =EEGet(     Max_Torque_low_addr);
  MAX_TORQUE_HIGH  =EEGet(    Max_Torque_high_addr);
  STATUS_RETURN_LEVEL =EEGet(   Status_Return_Level_addr);
  ALARM_LED   =EEGet(     Alarm_LED_addr);
  ALARM_SHUTDOWN  =EEGet(   Alarm_Shutdown_addr);

  //load from default

  TORQUE_ENABLE=Torque_Enable_default ;
  LED=LED_default ;
  CW_COMPLIANCE_MARGIN=CW_Compliance_Margin_default ;
  CCW_COMPLIANCE_MARGIN=CCW_Compliance_Margin_default ;
  CW_COMPLIANCE_SLOPE=CW_Compliance_Slope_default ;
  CCW_COMPLIANCE_SLOPE=CCW_Compliance_Slope_default ;
  GOAL_POSITION_LOW=Goal_Position_low_default ;
  GOAL_POSITION_HIGH=Goal_Position_high_default ;
  MOVING_SPEED_LOW=Moving_Speed_low_default ;
  MOVING_SPEED_HIGH=Moving_Speed_high_default ;

  TORQUE_LIMIT_LOW= MAX_TORQUE_LOW;
  TORQUE_LIMIT_HIGH=  MAX_TORQUE_HIGH;

  PRESENT_POSITION_LOW=Present_Position_low_default; 
  PRESENT_POSITION_HIGH=Present_Position_high_default ;
  PRESENT_SPEED_LOW=Present_Speed_low_default ;
  PRESENT_SPEED_HIGH=Present_Speed_high_default ;
  PRESENT_LOAD_LOW=Present_Load_low_default ;
  PRESENT_LOAD_HIGH=Present_Load_high_default ;
  PRESENT_VOLTAGE=Present_Voltage_default ;
  PRESENT_TEMPERATURE=Present_Temperature_default; 
  REGISTERED=Registered_default ;
  MOVING=Moving_default ;
  LOCK=Lock_default ;
  PUNCH_LOW=Punch_low_default; 
  PUNCH_HIGH=Punch_high_default;

}


uint8_t read(uint8_t adresse) // not safe to unknown adresse
{
  if(adresse<0x18)
    return EEGet((uint16_t)adresse);

  switch(adresse)
    { 
    case 0X18: return TORQUE_ENABLE;
      break;
    case 0X19: return LED;
      break;
    case 0X1A: return CW_COMPLIANCE_MARGIN;
      break;
    case 0X1B: return CCW_COMPLIANCE_MARGIN;
      break;
    case 0X1C: return CW_COMPLIANCE_SLOPE;
      break;
    case 0X1D: return CCW_COMPLIANCE_SLOPE;
      break;
    case 0X1E: return GOAL_POSITION_LOW;
      break;
    case 0X1F: return GOAL_POSITION_HIGH;
      break;
    case 0X20: return MOVING_SPEED_LOW;
      break;
    case 0X21: return MOVING_SPEED_HIGH;
      break;
    case 0X22: return TORQUE_LIMIT_LOW;
      break;
    case 0X23: return TORQUE_LIMIT_HIGH;
      break;
    case 0X24: return PRESENT_POSITION_LOW;
      break;
    case 0X25: return PRESENT_POSITION_HIGH;
      break;
    case 0X26: return PRESENT_SPEED_LOW;
      break;
    case 0X27: return PRESENT_SPEED_HIGH;
      break;
    case 0X28: return PRESENT_LOAD_LOW;
      break;
    case 0X29: return PRESENT_LOAD_HIGH;
      break;
    case 0X2A: return PRESENT_VOLTAGE;
      break;
    case 0X2B: return PRESENT_TEMPERATURE;
      break;
    case 0X2C: return REGISTERED;
      break;
    case 0X2E: return MOVING;
      break;
    case 0X2F: return LOCK;
      break;
    case 0X30: return PUNCH_LOW;
      break;
    case 0X31: return PUNCH_HIGH;
      break;
    default:
      return 0;
    }
}

void write(uint8_t adresse,uint8_t data) // not safe to unknown adresse
{
  if(adresse<=0x02 || (adresse>=0x24 && adresse<= 0x2E))// read only variables
    return;
  if(adresse<0x18)
     EESave((uint16_t)adresse,data);

  switch(adresse)
    {
    case 0X00: MODEL_NUMBER_LOW = data;
      break;
    case 0X01: MODEL_NUMBER_HIGH = data;
      break;
    case 0X02: VERSION_OF_FIRMWARE = data;
      break;
    case 0X03: ID = data;
      break;
    case 0X04: BAUD_RATE = data;
      USARTInit();      
      break;
    case 0X05: RETURN_DELAY_TIME = data;
      break;
    case 0X06: CW_ANGLE_LIMIT_LOW = data;
      break;
    case 0X07: CW_ANGLE_LIMIT_HIGH = data;
      break;
    case 0X08: CCW_ANGLE_LIMIT_LOW = data;
      break;
    case 0X09: CCW_ANGLE_LIMIT_HIGH = data;
      break;
    case 0X0B: THE_HIGHEST_LIMIT_TEMPERATURE = data;
      break;
    case 0X0C: THE_LOWEST_LIMIT_VOLTAGE = data;
      break;
    case 0X0D: THE_HIGHEST_LIMIT_VOLTAGE = data;
      break;
    case 0X0E: MAX_TORQUE_LOW = data;
      break;
    case 0X0F: MAX_TORQUE_HIGH = data;
      break;
    case 0X10: STATUS_RETURN_LEVEL = data;
      break;
    case 0X11: ALARM_LED = data;
      break;
    case 0X12: ALARM_SHUTDOWN = data;
      break;
    case 0X18: TORQUE_ENABLE = data;
      break;
    case 0X19: LED = data;
      break;
    case 0X1A: CW_COMPLIANCE_MARGIN = data;
      break;
    case 0X1B: CCW_COMPLIANCE_MARGIN = data;
      break;
    case 0X1C: CW_COMPLIANCE_SLOPE = data;
      break;
    case 0X1D: CCW_COMPLIANCE_SLOPE = data;
      break;
    case 0X1E: GOAL_POSITION_LOW = data;
      break;
    case 0X1F: GOAL_POSITION_HIGH = data;
      break;
    case 0X20: MOVING_SPEED_LOW = data;
      break;
    case 0X21: MOVING_SPEED_HIGH = data;
      needDesiredSpeedUpdate=1;
      break;
    case 0X22: TORQUE_LIMIT_LOW = data;
      break;
    case 0X23: TORQUE_LIMIT_HIGH = data;
      break;
    case 0X2F: LOCK = data;
      break;
    case 0X30: PUNCH_LOW = data;
      break;
    case 0X31: PUNCH_HIGH = data;
      break;
    default:
      break;
    }
  return;
}
