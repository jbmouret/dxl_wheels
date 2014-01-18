#include "serial.h"

extern unsigned char ID;
extern unsigned char BAUD_RATE;    


volatile Msg msg1;
volatile Msg msg2;
volatile Msg* rmsg=&msg1;
volatile Msg* wmsg=&msg2;
volatile Msg* waitingMsg=NULL;




void swap_msg()
{
volatile Msg* tmp=rmsg;
  rmsg=wmsg;
  wmsg=tmp;
}

/*uint8_t checkCRC(Msg* msg)
{
  uint8_t tmpCRC=MY_ID;
  tmpCRC+=(msg->length+2);
  tmpCRC+=msg->instruction;
  for(uint8_t i=0;i<msg->length;i++)
    tmpCRC+=msg->data[i];

  tmpCRC^=0xFF; // equivalent to NOT
  
  return msg->crc==tmpCRC;
}
*/
uint8_t computeCRC(volatile Msg* msg)
{
  uint8_t tmpCRC=ID;
  tmpCRC+=(msg->length+2);
  tmpCRC+=msg->instruction;
  for(uint8_t i=0;i<msg->length;i++)
    tmpCRC+=msg->data[i];

  tmpCRC^=0xFF; // equivalent to NOT
  
  return tmpCRC;
}


void send_msg( volatile Msg* msg)
{
  //  LED_ON
  uint8_t sreg =SREG;
  cli();
  SET_WRITE
  while( !( UCSRA & (1<<UDRE)) ); 

  UDR=0xFF;
  while( !( UCSRA & (1<<UDRE)) ); 
  UDR=0xFF;
  while( !( UCSRA & (1<<UDRE)) ); 
  UDR=ID;
  while( !( UCSRA & (1<<UDRE)) ); 
  UDR=msg->length+2;
  while( !( UCSRA & (1<<UDRE)) ); 
  UDR=msg->instruction; // ici instruction= error
  for(uint8_t i=0; i< msg->length;i++)
    {
      while( !( UCSRA & (1<<UDRE)) ); 
      UDR=msg->data[i];
    }

  while( !( UCSRA & (1<<UDRE)) ); 
  UCSRA &= ~(1<<TXC);
  UDR=computeCRC(msg);
  while( !( UCSRA & (1<<TXC)) ); 
  //  _delay_ms(100);
  SET_READ
  sei();
  SREG=sreg;
      //LED_OFF
}


uint8_t msg_timeout=0;

void read_incomming_char(uint8_t incomeChar)
{

  static uint8_t fillingInd=0;
  static uint8_t select=0;


  if( msg_timeout>=200)// nothing received since more thant 200ms
    {
      fillingInd=0;
      select=0;
      msg_timeout=0;
    }


  switch(select)
    {
    case 0:
    case 1:
      if(incomeChar==0xFF) // header found
	{
	  select++;
	} 
      else
	select=0;
      break;

    case 2: 

      if(incomeChar==ID) // my id found
	{ 
	  select++;
	} 
      else
	{
	  select=0;
	}
      break;

    case 3: //getting length
      wmsg->length=incomeChar-2;
      select++;
      break;

    case 4: //getting instruction
      if(incomeChar >0x06 && incomeChar != 0x86)// not a valid instruction
	{
	  if(incomeChar==0xFF)
	    select=1;
	  else
	    select=0;
	  swap_msg();

	}
      else
	{
	  wmsg->instruction=incomeChar;
	  select++;
	  /*      if(wmsg->length==0)
		  select=6;*/
	}
      break;

    case 5: // reading data
      if(fillingInd < wmsg->length)
	{
          wmsg->data[fillingInd]=incomeChar;
          fillingInd++; 
	  break;                  
	}
      else                    
	{                     
	  fillingInd=0;       
	  select++;           
	}                     

    case 6:                   
      
      wmsg->crc=incomeChar;   
      //      if(incomeChar == 0xFB)
      //	LED_ON
      if(computeCRC(wmsg) == wmsg->crc)//checkCRC(wmsg))      
	{                     
	  //LED_ON              
	  
	  waitingMsg=wmsg;
	  //  msg_management();

	  select=0;
	}                     
      else
	if(incomeChar==0xFF)
	  select=1;
	else
	  select=0;
      
      swap_msg();          //even if crc doesn't match, we swap to get the next message, otherwise the servo is blocked in this state.      

      break;                  
    }                         
  
}

volatile Msg msg;
void msg_management()
{
  if(!waitingMsg)
   return;
  
  //  LED_ON
  //cli();
  //send_msg(&emptyMsg);
  //  LED_ON
  switch(waitingMsg->instruction)
    {
    case 1: //ping
      //      LED_ON
      msg.instruction=0;
      msg.length=0;
      
      send_msg(&msg);
      break;

    case 2: // read data
      msg.instruction=0;
      msg.length=waitingMsg->data[1];
      for(uint8_t i=0;i<waitingMsg->data[1];i++)
	msg.data[i]=read(i+waitingMsg->data[0]);
      send_msg(&msg);
      break;

    case 3: // write data 
      msg.instruction=0;
      msg.length=0;

      for(uint8_t i=0;i<waitingMsg->length-1;i++)
	write(i + waitingMsg->data[0],waitingMsg->data[i+1]);
      send_msg(&msg);
      
      break;

    case 4: // reg write
      // NOT IMPLEMENTED
      break;

    case 5:// action
      // NOT IMPLEMENTED
      break;

    case 6: // reset
      msg.instruction=0;
      msg.length=0;
      EEPromReset();
      send_msg(&msg);
      break;

    case 0x86: //sync write
      break;
      

      }
  //sei();
  waitingMsg=NULL;
}



extern 

void USARTInit(void)
{
  // Set baud rate
  UCSRA = bit(U2X); //using double speed                                                                                                                                                                

  UCSRA &= ~(1<<TXC);
  //enable reception and RC complete interrupt
  UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE)|(1<<TXCIE);
  //UCSRB = (1<<TXEN)/*|(1<<UDRIE);//*/|(1<<TXCIE);

  //  UBRRH = (uint8_t)(UBRR_VALUE>>8);
  //UBRRL = (uint8_t)UBRR_VALUE;
  // Set frame format to 8 data bits, no parity, 1 stop bit

  // UCSRC |= (1<<UCSZ1)|(1<<UCSZ0);
  UCSRB &= ~(bit(RXB8)|bit(TXB8)|bit(UCSZ2));   // disable  TX and RX nineth bit and reset UCSZ2, this is used with UCSZ1 and UCSZ0 to determine the length of the word                                 
  UCSRC = ~(bit(UMSEL)|bit(UPM0)|bit(UPM1)|bit(USBS)|bit(UCPOL));//disable parity and set assyncronous, set 1stopBit and polarity on rising edge                                                        
  UCSRC |= (bit(UCSZ1)|bit(UCSZ0)|bit(UCPOL));
  //   UCSZ2=0 UCSZ1=1 UCSZ0=1 => 8bit length                                                                                                                                                           

  UBRRH = ((BAUD_RATE)>>8);   //set baud counter high     
  UBRRL = (BAUD_RATE & 0xFF);   //set baud counter low    



}


//RX Complete interrupt service routine
ISR(USART_RXC_vect)
{
  //  LED_OFF
  uint8_t u8temp;
  u8temp=UDR;

  read_incomming_char(u8temp);

  //check if period char or end of buffer
  /*  if ((BufferWrite(&buf, u8temp)==0)||(u8temp=='.'))
    {
      //      LED_ON
      //disable reception and RX Complete interrupt
      UCSRB &= ~((1<<RXEN)|(1<<RXCIE));
      //enable transmission and UDR0 empty interrupt
      UCSRB |= (1<<TXEN)|(1<<UDRIE);
      //      SET_WRITE
      }*/
}
//UDR0 Empty interrupt service routine
//ISR(USART_UDRE_vect)
//{
//  SET_WRITE
    //if index is not at start of buffer
    /*  if (BufferRead(&buf, &UDR)==1)
    {
      LED_OFF
      //start over
      //reset buffer
      BufferInit(&buf);
      //disable transmission and UDR0 empty interrupt
      UCSRB &= ~((1<<TXEN)|(1<<UDRIE));
      //enable reception and RC complete interrupt
      UCSRB |= (1<<RXEN)|(1<<RXCIE);

      }*/
//}
ISR(USART_TXC_vect)
{
  //LED_ON
  //SET_READ
    //LED_ON
}
