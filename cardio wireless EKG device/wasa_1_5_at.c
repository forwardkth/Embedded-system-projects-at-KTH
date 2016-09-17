/*********************************************************************
Copyright (C) <2010>  <M.T. Smith>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
********************************************************************
* modified by Fu Tang, Chao Li 
* KTH  ICT school  Design and Implementation of ICT Products and System
* 2010.2.20
*********************************************************************/

#include <msp430f2418.h> //modified to fit the wasa with WT12
#include "wasa_1_5_at.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "flash.h"

/*
 * 0:means in commad mode commucate with BT
 * 1:means process data,cannot received command from BT
 * 2:enter sleep mode.
 * */
static _program_status program_status;
int keypressed=0;//1 start,2 stop
int data_processed=0;
int data_gathered=0;
static int Button_status=0;

/* ##############################################################
   Bluetooth related function decleration
   ##############################################################
   
*/
int BT_kbhit();
void BT_init();
void BT_sendchar2BT(char a_char_to_print);
void BT_sendCMD2BT(char *a_string_to_print);
int  BT_process_data( void );
char BT_getchar( void );
void BT_gets( char *buffer_to_use);
void BT_sleepOn();
void BT_sleepOff();

void TurnOffCPU();
int HaveHistoryData();
int sendDataToTerminal();

void ADC_start(void);
void ADC_stop(void);
int ADC_getdata( void );
int ADC_dataproc();



//////////////////////////////////////////////////////////////////////////////////////

void mts_putchar(char a_char_to_print)
{
  while (!(IFG2&UCA0TXIFG));   // wait until USCI_A0 TX buffer empty
  UCA0TXBUF = a_char_to_print;
}

void mts_puts(char *a_string_to_print)
{
  while(*a_string_to_print != 0)  {
    mts_putchar(*a_string_to_print);
    ++a_string_to_print;
  }
}

/* mts_kbhit will return non-zero if the input queue is not empty.
*/

int mts_kbhit( void )
{
  if(queue_in_ptr == queue_out_ptr) return 0;
  else return 1;
}

/* mts_getchar works by getting characters out of the input queue.  It
does not directly read the UART registers. If there is NOT a character
in the queue already, then go into LPM0 and wait for one.  Else, if
there is a character, don't wait, just get it out of the queue.
*/

char mts_getchar( void )
{ 
  if(!mts_kbhit()) {  // need to wait for a char
  __bis_SR_register(LPM0_bits);   // Enter LPM0
  }
  //return input_queue[queue_out_ptr++];  // return character
  return 'a';//modified
}

/* mts_ungetchar just moves the queue pointer back by 1 to effect an ungetch
like command.  It effectively pushes the last read character back into the
queue by just moving the queue pointer.
*/
void mts_ungetchar( void )
{
  --queue_out_ptr; // back up the queue pointer
}

/* mts_gets collects characters into a buffer until it sees a CR.  It then
returns the NULL terminated string.
*/
void mts_gets( char *buffer_to_use)
{
  do {
   *buffer_to_use = mts_getchar();
  } while (*buffer_to_use++ != 0x0d);  // look for a CR
  *buffer_to_use = 0;     // NULL terminate
}

/* mts_uint2str is a recursive routine that will convert an unsigned
integer into an ascii string representing its value.  It returns the
number of digits in the number.
*/

int mts_uint2str(unsigned int a_unsigned_int, char *buffer_to_use)
{
unsigned int x;
unsigned int r;
int numdigits;

  numdigits = 0;
  x = a_unsigned_int / 10;
  r = a_unsigned_int % 10;
  if(x != 0) numdigits = mts_uint2str(x, buffer_to_use); // recurse thru

  buffer_to_use += numdigits; // index into array

  *buffer_to_use++ = (char) r + 0x30;  // convert to ascii
  *buffer_to_use = 0;  //null terminate
  return ++numdigits;
}
int futang_ulongint2str(unsigned long a_unsigned_int, char *buffer_to_use)
{
  unsigned long x;
  unsigned long r;
  int numdigits;

  numdigits = 0;
  x = a_unsigned_int / 10;
  r = a_unsigned_int % 10;
  if(x != 0) numdigits = futang_ulongint2str(x, buffer_to_use); // recurse thru

  buffer_to_use += numdigits; // index into array

  *buffer_to_use++ = (char) r + 0x30;  // convert to ascii
  *buffer_to_use = 0;  //null terminate
  return ++numdigits;
}
/* mts_str2uint does the reverse of mts_uint2str.  It takes a string of
ASCII digits and returns them an unsigned integer.  The string should just
be formed of characters representing numbers 0 thru 9.  The conversion will
stop on the first non-numeric number found, so the string should be NULL
terminated.
*/

unsigned int mts_str2uint( char *buffer_to_use )
{
unsigned int x;

   x = 0;
   while((*buffer_to_use >= '0') && (*buffer_to_use <= '9'))  {
     x *= 10; // scale the number
     x += (unsigned int) (*buffer_to_use++ - 0x30); // convert to uint
   }
   return x;
}


/* mts_toupper is a simple routine that will raise a lower case character
to upper case.  This is useful because many AT commands are supposed to be
case insensitive.
*/

char mts_toupper( char inchar )
{
  if((inchar >= 'a') && (inchar <= 'z')) return (inchar &= ~0x20);
  else return inchar;
}

/* get_numeric_args expects to see a numeric string in the input queue and
will return it null terminated.  It will return how big the string of
characters is, not including the null.  Numeric arguments for AT commands
usually don't have huge numeric arguments, so this routine will exit after
either:
- Seeing more than 9 numbers (almost bigger than a 32 bit number)
- Finding a non-numeric character
*/

int get_numeric_args(char *working_buffer)
{
int x;

  x = 0;
  while (x < 8)  {
    *working_buffer = mts_getchar();  // get a char
    if((*working_buffer < '0') || (*working_buffer > '9')) { //not a number
      mts_ungetchar();  // put whatever it was back
      break;
    }
    else {  // its a number
      ++x;
      ++working_buffer;
    }
  }
  *working_buffer = 0; // null terminate
  return x;
}

// ######################################################################

////////////////////////////////////////////////////
// Init wasa board.
////////////////////////////////////////////////////

void init_wasa_board( void )
{
int i;

  /* On this version of the board, P3.0 thru P3.3 are used as board version
  ID bits.  They can allow this one version of code to accomodate small
  board differences.  Set up to be able to read these bits.
*/
 //   P3DIR &= 0xf0;                 // Make P3.0-3 INPUTS

// P1.4 thru P1.6 control the accelerometer.  They set sensitivity etc.  
 // P1DIR |= 0x70;                        // Set P1.4 thru P1.6 to outputs
 // P1OUT = 0x40;                         // Enable accelerometer
  
  /* Set up the processor clocks.  Enable the XT2 clock and get MCLK (Main clock) working
  off of it.
*/
  BCSCTL1 &= ~XT2OFF;                // Turn on XT2
  BCSCTL3 |= XT2S_2;                 // High frequency crystal

do
  {
    IFG1 &= ~OFIFG;                   // Clear OSCFault flag
    for (i = 0xFF; i > 0; i--);       // Time for flag to set
  }
  while (IFG1 & OFIFG);               // Do this until XT2 is 'stable'

   BCSCTL2 |= SELM_2;                 // Then make MCLK = XT2
   
   
   /* Set up the circular input character queue.  The global queue_in_ptr
   points to the next position in the queue where an incoming character
   will go.  The global queue_out_ptr points to the next character to be
   read from the queue.  Both of these pointers are unsigned chars, and
   they are supposed to roll over, thus forming a circular queue.  When
   they are equal in value it either means that the queue is empty, or
   it means that the queue has wrapped around on top of itself, and
   characters are about to be lost.  The latter should not happen.  If
   there is a risk of this, a good thing would be to enable hardware
   flow control.
   */
   
   queue_in_ptr = 0;  // init both, queue is empty
   queue_out_ptr = 0;
   
   UinxTimeStamp=1267911000;
   
   /*Init the memory*/
   memset(ecg_workingbuf,0x00,sizeof(ecg_workingbuf));
   memset(ecg_input_queue,0x00,sizeof(ecg_input_queue));
   memset(BT_input_queue,0x00,sizeof(BT_input_queue));
   ecg_queue_in_ptr=0;
   ecg_queue_out_ptr=0;
/*Iinit the BlueTooth queue,added by tangfu*/

   BT_queue_in_ptr = 0;  // init both, queue is empty
   BT_queue_out_ptr = 0;
   program_status = general_idle;
   
}
/////////////////////////////////////////////////////////////////////////////////
//set up UART A0
void set_up_UART_A0( void )
{
  /* Now, set up UART A0.  It defaults to 115200 baud.  It uses XT2
   for the source of the baud clock.
*/
  P3SEL = 0xf0;                   // Use P3.4,5 for UART A0 TXD and RXD
  BCSCTL2 |= SELS;                // SMCLK now sourced from XT2
  UCA0CTL1 |= UCSSEL_2;           // UART clock now sourced from SMCLK
  
  // Set up the baud divisors
  UCA0BR0 = 138;                        // 16 Mhz giving 115200 baud
  UCA0BR1 = 0;                          // 16 Mhz giving 115200 baud
  UCA0MCTL = UCBRS2 + UCBRS1 + UCBRS0;  // Modulation UCBRSx = 7
  
  UCA0CTL1 &= ~UCSWRST;            // Start the UART
  IE2 |= UCA0RXIE;                 // And enable UART A0 RX interrupt
 // __bis_SR_register(GIE);          // And enable interupts
}

/////////////////////////////////////////////////////////////////////////////////
//set up UART A1 which communicate with bluegiga
/////////////////////////////////////////////////////////////////////////////////

void set_up_UART_A1( void )
{
  /* Now, set up UART A1.  It defaults to 115200 baud.  It uses XT2
   for the source of the baud clock.
*/
  UCA1CTL1 |= UCSSEL_2;                 // UART clock now sourced from SMCLK
  // Set up the baud divisors
  UCA1BR0 = 138;                        // 16 Mhz giving 115200 baud
  UCA1BR1 = 0;                          // 16 Mhz giving 115200 baud
  UCA1MCTL = UCBRS2 + UCBRS1 + UCBRS0;  // Modulation UCBRSx = 7
  
  UCA1CTL1 &= ~UCSWRST;                 // Start the UART
  UC1IE |= UCA1RXIE;                    // And enable UART A1 RX interrupt
 // __bis_SR_register(GIE);               // And enable interupts
}

//////////////////////////////////////////////////////////////////////////////////

/* Next, set up the ADC12 section.  We assume that for basic board operation,
A0 through A7 are mapped to their corresponding ADC channels. We will default
assume single conversions.  Other extended commands may use multiple
conversions, ie to read a 3 axis accelerometer, but in that case the right
bits to do multiple conversions will be set up then.
*/

void set_up_adc12_IO( void )
{
  
  // The analog inputs are connected to P6.0 thru P6.7
  
  P6DIR = 0x00;             // Be sure all are inputs
  P6SEL = 0x80;            // select A7 as A/D channel inputs for ECG signal
  
  // P6.4 as input of Button 0;  P6.5 as input of Button 1.
  
  // We will use the internal ADC12OSC and default to single conversions.
  
  ADC12CTL0 = 0;  					// be sure everything inits off, especially ENC
  ADC12CTL0 = ADC12ON + SHT0_4; 		// Sampling time, set sample hold time
  ADC12CTL1 = SHP+CONSEQ_0;     			// use sampling timer
  ADC12CTL1 |= 0x7000;
  ADC12MCTL7 = INCH_7;             // VR+ = AVCC and VR- = AVSS, channel = A7
 
  TACCR0 = 1500;                            // Delay to allow Ref to settle
  TACCTL0 |= CCIE;                          // Compare-mode interrupt.
  TACTL = TASSEL_1 | MC_1;                  // TACLK = ACLK, Up mode.
  __bis_SR_register(LPM3_bits + GIE);       // Wait for delay, Enable interrupts
  TACCTL0 &= ~CCIE;                         // Disable timer
}
void set_up_timer( void )
{
  TACCR2 = 0x7FFF;  						//ADD FOR TIMESTAMP
  TACCTL2 |= CCIE;  						//ADD FOR TIMESTAMP
  //P2SEL |= BIT3;                            // Set for Timer A1
  //P2DIR |= 0x08;
  TACCR0 = 0x7FFF;                          // Init TACCR0 w/ sample prd=CCR0+1
 // TACCR1 = 0x4FFF;                          // Trig for ADC12 sample & convert
  //TACCTL1 = OUTMOD_3;                       // Set/reset
  TACTL = TACLR | MC_2 | TASSEL_1;          // ACLK, clear TAR, Continous up 
}
////////////////////////////////////////////////////////////////////////////////////////////
//Button port 1.2 setup and related functions
///////////////////////////////////////////////////////////////////////////////////////////////

void set_up_P1( void )
{
	  P1REN |= 0x04;             // PULL up enable
	  P1DIR &= 0xFB;            // Be sure P 1.2 is input for interrupt
	  P1IN |=0x04;              // PULL UP
	  P1IE |= 0x04;             // enable P1.2 interrupt
	  P1IES |= 0x04;            // set P1.2 interrupt as down level trigger 
	  //_BIS_SR(LPM3 +GIE);
}

int keypick(void) // Key detect
{
	int x;
	x=P1IN&0x04;
	return x;
}
void Delay(void)
{
	int i;
	for(i=400;i>0;i++);
}
/* We assume some default conditions with respect to AT commands.  They are:
1. Verbose response mode (V1)
2. Echo off (E0)
3. Result codes are NOT suppressed (Q0)

Note that these can be changed by sending the appropriate AT command.
*/

void set_up_at_defaults( void )
{
  basic_at_command_flags = flag_v;
  future_at_command_flags = flag_v;
}


/////////////////////////////////////////////////////////////////////////
int A_command( void )
{
	int ret_value=0;
	int counter=0;
	char cmd1,cmd2;
	//char address[32];
	char mybuff[32];
	cmd1=mts_toupper(BT_getchar());

	switch(cmd1)
	{
		case 'T':
		if('O'==mts_toupper(BT_getchar()))
		{
			cmd2=mts_toupper(BT_getchar());
			if('N'==cmd2)
			{   
				mts_puts("ATON\r\n");
			   // ADC_start();
				//program_status = 1;
			    ret_value=1;
			    BT_gets(mybuff);//clear a line
			}else if('F'==cmd2)
			{
					mts_puts("ATOFF\r\n");
		           // ADC_stop();
		           
					//program_status = 2;
					ret_value=1;
					BT_gets(mybuff);//clear a line
			}
		}
		break;
		case 'U':
		mts_puts("AUTH EVENT received\r\n");
		if('T'==mts_toupper(BT_getchar()))
		if('H'==mts_toupper(BT_getchar()))
		{
			cmd2 = mts_toupper(BT_getchar());
			if(' '==cmd2)
			{
			for(counter=0;counter<16;counter++)
			{
			//	address[counter]=mts_toupper(BT_getchar());
			}
			mts_puts("address:");
			//mts_puts(address);
			mts_puts("\r\n");
			}
			BT_sendCMD2BT("AUTH 00:25:47:5c:e9:b6 0000\r\n");
			ret_value=1;
			BT_gets(mybuff);//clear a line
		}
		break;
		default:
		mts_putchar(cmd1);
		mts_puts("at default\r\n");
		ret_value=0;
		break;
	}
	
 return ret_value;	
}
  
// ######################################################################
//incluing READY,RING
int R_command( void )
{
	int ret_value=0;
	char cmd1;
	char mybuff[32];
	cmd1=mts_toupper(BT_getchar());
	switch(cmd1)
	{
		case 'E':
		if('A'==mts_toupper(BT_getchar()))
		{
			if('D'==mts_toupper(BT_getchar()))
			{
				
				if('Y'==mts_toupper(BT_getchar()))
				{
					mts_puts("ready\r\n");
					//BT_sendCMD2BT("CALL 00:25:47:5C:E9:B6 1101 RFCOMM\r\n");
					//BT_sendCMD2BT("CALL 00:1F:7E:4C:72:CF 1101 RFCOMM\r\n");
					BT_sendCMD2BT("CALL 00:07:80:89:EB:10 1101 RFCOMM\r\n");//li chao
					//BT_sendCMD2BT("CALL 00:23:6C:A3:BA:4D 1101 RFCOMM\r\n");//dima
					mts_puts("CALLING...\r\n");
					ret_value=1;
					BT_gets(mybuff);//clear a line
				}
			}
		}
		break;
		case 'I':
		if('N'==mts_toupper(BT_getchar()))
		{
		if('G'==mts_toupper(BT_getchar()))
		{
			mts_puts("RING\r\n");
			ret_value=1;
			BT_gets(mybuff);//clear a line
			BT_sendCMD2BT("received ring event!\r\n");
		}
		}
		break;
		default:
		mts_puts("R default\r\n");
		ret_value=0;
		break;
	}
	
 return ret_value;
}
//incluing SET,ETC
int S_command( void )
{
	int ret_value=0;
	char mybuff[32];
	char cmd1;
	cmd1=mts_toupper(BT_getchar());
	//mts_putchar(cmd1);
	switch(cmd1)
	{
		case 'E':
		if('T'==mts_toupper(BT_getchar()))
		{
		mts_puts("SET\r\n");
		ret_value=1;
		BT_gets(mybuff);
		}
		break;
		default:
		mts_puts("S default\r\n");
		ret_value=0;
		break;
	}
	
 return ret_value;
}
/////////////////////////////////////////////////////////////////////////
//
//if the bluetooth communication connected, 
//
/////////////////////////////////////////////////////////////////////////
int C_command()
{
	int ret_value=0;
	
	char mybuff[32];
	char cmd1;
	cmd1=mts_toupper(BT_getchar());
	switch(cmd1)
	{
		case 'O':
		if('N'==mts_toupper(BT_getchar()))
		{
			if('N'==mts_toupper(BT_getchar()))
			{
				if('E'==mts_toupper(BT_getchar()))
				{
					if('C'==mts_toupper(BT_getchar()))
					{
						if('T'==mts_toupper(BT_getchar()))
						{
							mts_puts("CONNECT OK\r\n");
							BT_sendCMD2BT("CONNECT OK\r\n");
							ret_value=1;
							BT_gets(mybuff);
						}
					}
				}
			}
		}
		break;
		case 'A':
		if('L'==mts_toupper(BT_getchar()))
		if('L'==mts_toupper(BT_getchar()))
		{
			mts_puts("CALL EVENT\r\n");
			ret_value=1;
			BT_gets(mybuff);
		}
		break;
		default:
		mts_puts("C_command default\r\n");
		ret_value=0;
		break;
	}
	
 return ret_value;
}
int P_command()
{
	int ret_value=0;
	char mybuff[32];
	char cmd1;
	cmd1=mts_toupper(BT_getchar());
	switch(cmd1)
	{
		case 'A':
		if('I'==mts_toupper(BT_getchar()))
		{
			if('R'==mts_toupper(BT_getchar()))
			{
			mts_puts("PAIR\r\n");
			ret_value=1;
			BT_gets(mybuff);
			BT_sendCMD2BT("CALL 00:07:80:89:EB:10 1101 RFCOMM\r\n");
			}
		}
		break;
		default:
		mts_puts("P default\r\n");
		ret_value=0;
		break;
	}
	
 return ret_value;
}
//////////////////////////////////////////////////////////////////////////////////
// Main function
//////////////////////////////////////////////////////////////////////////////////

int main( void )
{
//char test[]="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbcccccc";
// Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

// Initialize ID straps, clocks and input queues on the WASA board. 
  init_wasa_board();
  set_up_UART_A0();
// Set up and initialize the communication UART (A1) between CPU and Bluetooth 
  set_up_UART_A1();
  
// Set up and initialize the ADC12 Analog to Digital section
  
  set_up_adc12_IO();
 // Set up and initialize the KEYS
  set_up_P1();
  // Set up and initialize the timer
  set_up_timer();
  //initiate the external flash memory
  #if 0
  init_flash();
  
  memset(DF_buffer,0x00,sizeof(DF_buffer));
  memcpy(DF_buffer,test,sizeof(test));
 // mts_puts(test);
 // DF_write_page(1,0);

  //DF_read_page(1,0);
  #endif
   __bis_SR_register(GIE);               // And enable interupts
// Set up our defaults for the AT command set
  //set_up_at_defaults();
  
  
while(1)
{
	
 	if(general_idle ==program_status)// general active
 	{
 		if(1 < data_gathered)
 		program_status=data_sending;
 		if (1== keypressed%2)
 		{
 			if (1==HaveHistoryData())
 			{
 				program_status=data_sending;
 			}
 			else
 			{
 				program_status=data_gathering;
 				BT_sleepOff();//BT sleep
 			}
 		}
 		if(1==data_processed)//data send ok or fail
 		{
 			program_status=program_sleep;
 			data_processed=0;//recovery for next time
  			BT_sleepOff();
  			TurnOffCPU();
 		}
 		if(0 ==keypressed)//never used
 		{
 			program_status=program_sleep;
  			BT_sleepOff();
  			TurnOffCPU();
 		}
 	}
 	else if( data_gathering ==program_status)
 	{
 		//mts_puts(BT_input_queue);
		data_gathered=ADC_dataproc();
		data_in_ptr = 0;
		program_status=general_idle;
 	}
 	else if( data_sending ==program_status)
 	{
 		#if 0//as bluetooth cannot work,we send data to UART0
 		BT_sleepOn();//wake up the BT
 		//initiate the bluetooth module
  		BT_init();
 		BT_process_data();
 		#endif
 		data_processed=1;//data send ok or fail
 		sendDataToTerminal();
 		keypressed=0;
 		program_status=general_idle;
 	}
 	else if ( program_sleep ==program_status)
 	{
 		//Just wait for key interupt routine keypressed++
 		//keypressed = 1;
 	}
}//close of while

} // close of main()

////////////////////////////////////////////////////////////////////////////////////////////

/* ##############################################################
   Interrupt routines are out here.
   ##############################################################
*/

// For UART1 bluetooth commands and responds  receive

#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCI1RX_ISR(void)
{
  UC1IE &= ~UCA1RXIE;                 // disable UART A1 RX interrupt
  BT_input_queue[BT_queue_in_ptr] = UCA1RXBUF;  //put character in queue
 // mts_putchar('+');
  ++BT_queue_in_ptr;                   // increment the pointer
  _BIC_SR_IRQ(LPM0_bits);           // Clear CPUOFF bit from 0(SR)
  UC1IE |= UCA1RXIE;                 // And enable UART A1 RX interrupt
  __bis_SR_register(GIE);          // And enable interupts
}
#if 0
//flash memory interuput
#pragma vector=USCIAB1RX_VECTOR
__interrupt void USCIB1RX_ISR (void)
{
  while (!(UC1IE & UCB1TXIFG));              // USCI_B1 TX buffer ready?
  //UCB0TXBUF = UCB1RXBUF;
  mts_putchar(UCB1RXBUF);
}
#endif

// Timer_A0 Interrupt Service Routine
#pragma vector=TIMERA0_VECTOR
__interrupt void ta0_isr(void)
{
  TACCTL0 &= ~CCIE; 
  TACTL = 0;
  __bic_SR_register_on_exit(LPM3_bits);
  __bis_SR_register(GIE);        // And enable interupts 
}
#pragma vector=TIMERA1_VECTOR
__interrupt void TIMERA1_ISR(void)
{
	//if(TACCTL1&= ~CCIFG){TACCTL1 &= ~CCIFG; return;}
	TACCTL1 &= ~CCIFG;
	TACCTL2 &= ~CCIE;
	TACCTL2 &= ~CCIFG; 
	UinxTimeStamp++;
	TACCTL2 |= CCIE;
}
// For ADC12 analog to digital conversions
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR (void)
{
 static int counter=0;
 
  char mybuff[16];
  counter++;
  //ADC12IE = 0x0000;                // Disable all ADC interrupts
  ADC12IFG = 0x0000;              // Clear off all interrupt flags
  ADC12CTL0 &= ~ENC;             // Conversion disabled
  ecg_input_queue[ecg_queue_in_ptr] = ADC12MEM7;  //put character in queue
  sprintf(mybuff,"%d,%d.",ecg_input_queue[ecg_queue_in_ptr],counter);
  mts_puts(mybuff);
  ecg_queue_in_ptr+=2; 
  //ADC12CTL0 |= ENC;
   ADC12CTL0 |= ENC+ADC12SC;            // enable ADC12   
  _BIC_SR_IRQ(LPM0_bits);        // Clear CPUOFF bit from 0(SR)
  //ADC12IE= 0x0007;
  __bis_SR_register(GIE);        // And enable interupts 
}

//////////////////////////////////////////////////////////////////////////
#pragma vector= PORT1_VECTOR
__interrupt void Button_ISR(void)
{   
	P1IFG=0;
	_BIC_SR_IRQ(LPM0_bits);        // Clear CPUOFF bit from 0(SR)
	while(keypick()!= 0x04)// button down
	{
		Delay();
		while(keypick()!= 0x04) // make sure button down
		{
			if(keypick()== 0x04)// button up
			{
				keypressed++;
				program_status=general_idle;
				if ( Button_status==0)
				{
					 Button_status=1;// data steam on
					 
					 return;
				}else if(Button_status==1)
				{
					Button_status=0; //data stream off
					
					return;
				}
			}
		}
		
	}
}
///////////////////////////////////////////////////////////////////////////////
//AD Conversion control
///////////////////////////////////////////////////////////////////////////////
void ADC_start(void)
{

   ADC12IE    = 0x0080; 					// enable interrupt
   ADC12IFG = 0x0000;
  // ADC12CTL0 |= ADC12ON;				// turn on ADC12
   ADC12CTL0 |= ENC+ADC12SC;            // enable ADC12
}

void ADC_stop(void)
{   
	ADC12IE    = 0;						   // disable interrupt
	ADC12CTL0 &= ~ADC12SC;				   // Stop conversion
    ADC12CTL0 &= ~ENC;                     // enable ADC12
    //ADC12CTL0 &= ~ADC12ON;				   // turn off ADC12 to save power
}

int ADC_kbhit()
{
	if(ecg_queue_in_ptr == ecg_queue_out_ptr) return 0;
  else return 1;
}
int ADC_getdata( void )
{ 
  if(!ADC_kbhit()) {  // need to wait for a char
  	return 0xFFFF;
  //__bis_SR_register(LPM0_bits);   // Enter LPM0
  }
  ecg_queue_out_ptr+=2;
  return ecg_input_queue[ecg_queue_out_ptr-2];  // return character
}

int ADC_dataproc()
{ 
	int numtimes=0;
	
	//long time=1266685663;
	//char mybuff[32];
	int i=0; 
	long sum=0;
	int counter=0;
	long temp=0;
	
	ADC_start();
	data_in_ptr=0;// the first byte of the working buffer
	//sprintf(mybuff,"{\"measurements\":");
	//BT_sendCMD2BT(mybuff);
	//sprintf(mybuff,"[{\"timestamp\":%ld,\"sample\":[",time);
	//BT_sendCMD2BT(mybuff);
	for(numtimes=0;numtimes<MAX_DATA_CYCLE;numtimes++)
	{
	while(1)
	{
		sum = 0;
		for(i=0;i<50;i++)
		{
			if(0==keypressed%2)
			{
				ADC_stop();
				
				return data_in_ptr;
			}
			temp=ADC_getdata();
			if(0xFFFF!=temp)
			sum+=temp;
			else
			i--;
		}
		//ecg_data[counter++]=sum/50;
		ecg_workingbuf[data_in_ptr++] = sum/50;
		counter++;
		//sprintf(mybuff,"%d,",ecg_data[counter-1]);
		//BT_sendCMD2BT(mybuff);
		if (120==counter)
		{
			counter=0;
 			//sprintf(mybuff,"]},");
			//BT_sendCMD2BT(mybuff);
			break;
		}
	}
	}
	keypressed++;
	ADC_stop();
	return data_in_ptr;
	
}
int sendDataToTerminal()
{
	char mybuff[32];
	int counter=0;
	//int counterj=0;
	long time=1266685663;
	long mytimeHigh = (time<<0xF)|0x00;
	long mytimeLow  = (time>>0xF)|0x00;
	//{"measurements":[{"timestamp":1266685663,"sample":[
	sprintf(mybuff,"{\"measurements\":[");
	mts_puts(mybuff);
	sprintf(mybuff,"{\"timestamp\":");
	mts_puts(mybuff);
	futang_ulongint2str(UinxTimeStamp,mybuff);
	mts_puts(mybuff);
	sprintf(mybuff,",\"sample\":[");
	mts_puts(mybuff);
	for(counter=0;counter<data_gathered;counter++)
	{
		sprintf(mybuff,"%d,",ecg_workingbuf[counter]);
		mts_puts(mybuff);
	}
	sprintf(mybuff,"]},");
	mts_puts(mybuff);
	sprintf(mybuff,"]}\r\n");
	mts_puts(mybuff);
	mts_puts("***");
	data_gathered=0;
	return 1;
}
//////////////////////////////////////////////////////////////////////////////////////////
/* ##############################################################
   bluetooth related code
   ##############################################################
   
*/
void BT_sendchar2BT(char a_char_to_print)
{
  while (!(UC1IFG&UCA1TXIFG));   // wait until USCI_A1 TX buffer empty
  UCA1TXBUF = a_char_to_print;
}
void BT_sendCMD2BT(char *a_string_to_print)
{
	while(*a_string_to_print != 0)  
	{
    BT_sendchar2BT(*a_string_to_print);
    ++a_string_to_print;
  }
}

/////////////////////////////////////////////////////////////////////////////////////
void BT_init()
{		                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
	//BT_sendCMD2BT("SET CONTROL ECHO 4\r\n");
	BT_sendCMD2BT("SET CONTROL ECHO 5\r\n");
	BT_sendCMD2BT("SET PROFILE SPP SPP\r\n");
	BT_sendCMD2BT("SET BT NAME RedTomato ECG V1.0\r\n");
	BT_sendCMD2BT("SET BT CLASS 001f00\r\n");
	BT_sendCMD2BT("SET BT AUTH * 0000\r\n");
	BT_sendCMD2BT("SET BT LAP 9e8b33\r\n");
	BT_sendCMD2BT("SET BT PAGEMODE 3 2000 1\r\n");
	BT_sendCMD2BT("SET CONTROL BAUD 115200,8n1\r\n");
	BT_sendCMD2BT("SET CONTROL CD 80 0\r\n");
	//BT_sendCMD2BT("SET CONTROL CONFIG 8AD\r\n");
	BT_sendCMD2BT("SET BT ROLE 1 f 7d00\r\n");
	BT_sendCMD2BT("RESET\r\n");

}
void BT_sleepOn()
{
	BT_sendCMD2BT("AT\r\n");   /*if BT is deep sleep,two ways to wake up
								1.BT module get some data from UART in command mode
								2.BT received connection*/	
}
void BT_sleepOff()
{
	BT_sendCMD2BT("SLEEP\r\n");/* entry deep sleep mode*/
}
void TurnOffCPU()
{
	__bis_SR_register(LPM0_bits);
}
int HaveHistoryData()
{
	// check flash 
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////

int BT_process_data( void )
{
	char mychar;
	char my_buffer[256];
	int cmd_ok=1;
	if(1== BT_kbhit())// check data
	{
		do {    // loop around for the entire command line
			mychar = mts_toupper(BT_getchar()); // fetch a command descriptor
			//mts_putchar(mychar);
			switch(mychar)  {  // parse the command descriptor
	  case SEMICOLON_ASCII:  // a NOP
	  case SPACE_ASCII: // a NOP
	  case TAB_ASCII: // a NOP
		  cmd_ok = 1;  // NOPs always succeed
		  break;
	  case CR_ASCII:  // CR, so must be command end
		  cmd_ok = 1;
		  break;
	  case 'A':
		  cmd_ok = A_command();
		  break;
	  case 'R':
		  cmd_ok = R_command();
		  break;
	  case 'S':
		  cmd_ok = S_command();
		  break;
	  case 'C':
		  cmd_ok = C_command();
		  break;
	  case 'P':
		  cmd_ok = P_command();
		  break;
	  case '\n':
		  cmd_ok=1;
		  break;
	  default: // if unknown, then it's an error
		  cmd_ok = 0;  // flag as an error
		  break;
			}
			if(cmd_ok == 0)  {  // oops, we have an error condition
				BT_gets(my_buffer);  //flush the command line
			} // close of if(x==0)

		}while(((mychar != '\n') && (cmd_ok == 1)));
	}
	return 1;
}
/* BT_getchar works by getting characters out of the input queue.  It
does not directly read the UART registers. If there is NOT a character
in the queue already, then go into LPM0 and wait for one.  Else, if
there is a character, don't wait, just get it out of the queue.
*/
int BT_kbhit()
{
	if(BT_queue_in_ptr == BT_queue_out_ptr) return 0;
  else return 1;
}
char BT_getchar( void )
{ 
 int data=1;
 do
 {
 	data=BT_kbhit();
 }while(!data);
  return BT_input_queue[BT_queue_out_ptr++];  // return character
}

////////////////////////////////////////////////////////////////////////////////////////////

void BT_gets( char *buffer_to_use)
{
  do {
   *buffer_to_use = BT_getchar();
  } while (*buffer_to_use++ != 0x0a);  // 
  *buffer_to_use = 0;     // NULL terminate
}
//////////////////////////////////////////////////////////////////////////////////////////////

