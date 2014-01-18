#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "utils.h"
#include "serial.h"
#include "motor.h"
#include "control.h"

#define BUF_SIZE 20
//type definition of buffer structure








typedef struct{
  //Array of chars
  uint8_t buffer[BUF_SIZE];
  //array element index
  uint8_t index;
}u8buf;
//declare buffer
u8buf buf;
//initialize buffer
void BufferInit(u8buf *buf)
{
  //set index to start of buffer
  buf->index=0;
}
//write to buffer routine
uint8_t BufferWrite(u8buf *buf, uint8_t u8data)
{
  if (buf->index<BUF_SIZE)
    {
      buf->buffer[buf->index] = u8data;
      //increment buffer index
      buf->index++;
      return 0;
    }
  else return 1;
}
uint8_t BufferRead(u8buf *buf, volatile uint8_t *u8data)
{
  if(buf->index>0)
    {
      buf->index--;
      //      *u8data=buf->buffer[buf->index];
      UDR='c';
      return 0;
    }
  else return 1;
}








int main (void)
{
  LED_OFF
  _delay_ms(100);
  LED_ON
  _delay_ms(100);
  LED_OFF
  
EELoadVars();

  //Init buffer
  //  BufferInit(&buf);
  //set sleep mode
  //set_sleep_mode(SLEEP_MODE_IDLE);
  //Initialize USART0

  InitMotor();
  InitControl();
  USARTInit();

  //  SET_WRITE
  //  InitSerial(34);
  //enable global interrupts
  //  LED_OFF


  sei();
  //  SetPWM(1950);
  while(1)
    {
      //put MCU to sleep
      //  sleep_mode();
      msg_management();
      updateVelocity();
      //LED_ON

    }
}
