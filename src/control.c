#include "control.h"
#include "serial.h"
#include "motor.h"

#include <stdlib.h>

#define NBPOS 15
#define LSPEED 8
#define BLINDAREA 10


extern uint8_t TORQUE_ENABLE;
extern uint8_t CW_COMPLIANCE_MARGIN;
extern uint8_t CCW_COMPLIANCE_MARGIN;
extern uint8_t CW_COMPLIANCE_SLOPE;

volatile uint16_t lastPositions[NBPOS];
volatile uint16_t lastPositionsTime[NBPOS];
volatile uint8_t idLastPosition=0;
volatile int velocity;

void InitControl(void)
{

  TIMSK|=bit(TOIE0); // timer overflow interupt enable 
  TCNT0=0x00;
  TCCR0|=(/*bit(CS02)|*/bit(CS01)|bit(CS00)); // only CSOO= no prescaler
  //  TCCR1B|=(/*bit(CS02)|bit(CS01)|*/bit(CS00)); // only CSOO= no prescaler timer 16bits
  



  //init ad for positioning                                                                                          
  //0 to 5v                                                                                                          
  // the best way to have accurate and fast position sampling, it leaving the ADC0 int free running mode, wich       
  // means the the ADC is constantly sampling, and we can read the register any time that is requested.              
  // no need to wait for conversion                                                                                  
  //IMPORTANTE                                                                                                       
  // if need to get temperature, need to have in mind that in this mode every channel selection reflects the previous
  // channel as result, just on the next cycle of sampling we will have the selected channel value.                  

  Motor_Dir|= (Motor_Clockwise|Motor_Anticlockwise);

  Position_Dir &= Position_Pin;
  Temperature_Dir &= Temperature_Pin;

  ADCSRA|=(bit(ADPS2)|bit(ADPS1)|bit(ADPS0)); // this selects the adc prescaler to XTALfreq/128= 125KHz              
  // this is needed once the best resolution is got with 50 to 200KHz freq                                           

  ADMUX|=bit(REFS0); // REFS = 0 1  Mean ARFPIN as reference, 5V refence on this case                                


  ADCSRA|=bit(ADSC);  // start the conversion processes                                                            
  ADCSRA|=bit(ADFR); //enable the free running, mean continuousely sampling                                        
  ADCSRA|=bit(ADIE); // enable the adc interrupt                                                              

  ADCSRA|=bit(ADEN); // enable adc                                                                                   
  ADCSRA&=~bit(ADIF); // clear adc interrupt flag                                                                    
  
}

uint8_t waiting0 =0;
uint8_t waiting1024=0;


void goingBlindZone(uint16_t position)
{
  if(position>1024-BLINDAREA && velocity>0)
    waiting0=1;
  if(position<BLINDAREA && velocity<0)  
    waiting1024=1;

}


uint8_t needUpdate=0;
uint8_t init=1;




void addPosition( uint16_t position)
{
  if(init)
    {  
      for(int i=0;i<NBPOS;i++)
	{
	  lastPositions[i]=position;
	  lastPositionsTime[i]=1;
	}
      init=0;
    }
  
  lastPositionsTime[idLastPosition]++;

  if(waiting0)
    {
      LED_ON
	if( position > BLINDAREA || position==0)
	return;
      else
	waiting0=0;
    }

  if(waiting1024)
    {
      LED_ON
	if( position <1024-BLINDAREA || position ==1023)
	return;
      else
	waiting1024=0;
    }

LED_OFF

  idLastPosition++;
  if (idLastPosition>=NBPOS)
    idLastPosition=0;
  
  lastPositions[idLastPosition]=position;
  lastPositionsTime[idLastPosition]=0;

  goingBlindZone(position);
  needUpdate=1;

}


void updateVelocity()
{
  if(!needUpdate)
    return;
  uint8_t sreg=SREG;
  cli();
  needUpdate=0;
  int new_velocity=0;
  uint8_t count=0;
  for(int i=0; i< LSPEED;i++)
    {
     
      int diff=(lastPositions[(idLastPosition-i>=0)?idLastPosition-i:idLastPosition-i+NBPOS]
		-lastPositions[(idLastPosition-i-1>=0)?idLastPosition-i-1:idLastPosition-i-1+NBPOS]);
      if( ( (Motor_Port & Motor_Anticlockwise) && diff>=0) || ( ( Motor_Port & Motor_Clockwise) && diff<0) ) 
	{
	  count+= lastPositionsTime[(idLastPosition-i-1>=0)?idLastPosition-i-1:idLastPosition-i-1+NBPOS];
	  new_velocity+= diff;
	}
    }



  velocity= new_velocity/count;

  sei();
  SREG=sreg;

  PRESENT_SPEED_LOW= ((velocity+1000) & 0x00FF);
  PRESENT_SPEED_HIGH= (velocity+1000)>>8 ;


}





uint8_t uncount=20;

ISR(ADC_vect)
{
  PRESENT_POSITION_LOW=ADCL;
  PRESENT_POSITION_HIGH=ADCH;
  if(uncount==0)
    {
      uncount=20;
      uint16_t position=(PRESENT_POSITION_HIGH<<8)+PRESENT_POSITION_LOW;
      addPosition(position);
    }
  else
    uncount--;
}


int speed_I=0;
int prev_error=0;
unsigned long  meanPWM=1000;
long blindCount=0;
uint16_t desired_speed=0;
uint8_t direction=0; //counter clock wise;

uint8_t needDesiredSpeedUpdate=0;



extern uint8_t msg_timeout;
ISR (TIMER0_OVF_vect)
{
  msg_timeout++;
  if(msg_timeout>200)
    msg_timeout=200;
  if(needDesiredSpeedUpdate && !(waiting0 || waiting1024))
    {
      desired_speed=(MOVING_SPEED_HIGH<<8)+MOVING_SPEED_LOW;
      direction=0; //counter clock wise;
      if(desired_speed>=1024)
	{
	  desired_speed-=1024;
	  direction = 1; // clock wise
	}
      blindCount=0;
      meanPWM=1000;
      needDesiredSpeedUpdate=0;
    }

  if(!TORQUE_ENABLE || desired_speed==0  )
    {
      speed_I=0;
      prev_error=0;
      SetPWM(1000);
      return;
    }

  MOVING = blindCount/50;

  if(waiting0 || waiting1024)
    {
      int temp=meanPWM+(1-(2*direction))*blindCount/100;
      if(temp>2000)
	temp=2000;
      else if(temp<0)
	temp=0;

      SetPWM(temp);
      blindCount+=1+blindCount/100;
      return;
    }

  blindCount=0;
  int error=(1-(2*direction))*desired_speed - velocity;  

  speed_I+= error;

  if (speed_I > 1000)
    speed_I = 1000;
  if (speed_I <- 1000)
    speed_I = -1000;
  int speed_D=error-prev_error;

  prev_error=error;


  int pwm = 1000 + 
    CW_COMPLIANCE_MARGIN * speed_D/10 +
    CCW_COMPLIANCE_MARGIN * speed_I/10 +
    CW_COMPLIANCE_SLOPE * error;


  meanPWM=(meanPWM*19+pwm)/20;
  

  if (pwm > 2000)
    pwm = 2000;
  if (pwm < 00)
    pwm = 00;
  SetPWM(pwm);
}
