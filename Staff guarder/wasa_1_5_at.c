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
//#include "cJSON.h"

//static int program_status=0;


/* ##############################################################
   Bluetooth related function decleration
   ##############################################################
   
*/

int UART_getdata( void );
void UART_dataproc();
void sensor_datasamp();
void lightsensor_dataproc();
	void sound_alarm();
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
  return input_queue[queue_out_ptr++];  // return character
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
    P3DIR &= 0xf0;                 // Make P3.0-3 INPUTS

   // P1.4 thru P1.6 control the accelerometer.  They set sensitivity etc.  
    P1DIR |= 0x70;                        // Set P1.4 thru P1.6 to outputs
    P1OUT = 0x40;                         // Enable accelerometer
  
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
   
   
/*Iinit the BlueTooth queue,added by tangfu*/

   BT_queue_in_ptr = 0;  // init both, queue is empty
   BT_queue_out_ptr = 0;
   
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
  __bis_SR_register(GIE);               // And enable interupts
}



//////////////////////////////////////////////////////////////////////////////////

/* Next, set up the ADC12 section.  We assume that for basic board operation,
A0 through A7 are mapped to their corresponding ADC channels. We will default
assume single conversions.  Other extended commands may use multiple
conversions, ie to read a 3 axis accelerometer, but in that case the right
bits to do multiple conversions will be set up then.
*/

void set_up_adc12( void )
{
  
  // The analog inputs are connected to P6.0 thru P6.7
  P6DIR = 0x00;             // Be sure all are inputs
  P6SEL = 0x57;            // and all are A/D channel inputs
  
  // We will use the internal ADC12OSC and default to single conversions.
  
  ADC12CTL0 = 0;  // be sure everything inits off, especially ENC
  ADC12CTL0 = SHT0_2 + ADC12ON; // Sampling time, turn on ADC12
  ADC12CTL1 = SHP + CONSEQ_2;     // Use sampling timer and single conversion
  ADC12MCTL0 = INCH_0;            // ref+=AVcc, channel = A0
  ADC12MCTL1 = INCH_1;            // ref+=AVcc, channel = A1
  ADC12MCTL2 = INCH_2;            // ref+=AVcc, channel = A2
  //ADC12MCTL3 = INCH_3;            // ref+=AVcc, channel = A3
  ADC12MCTL4 = INCH_4;            // ref+=AVcc, channel = A4
 // ADC12MCTL5 = INCH_5;            // ref+=AVcc, channel = A5
  ADC12MCTL6 = INCH_6;            // ref+=AVcc, channel = A6
 // ADC12MCTL7 = INCH_7;            // ref+=AVcc, channel = A7
 
 P6REN |=0xA8;
 //P5DIR &=0x00;
 P6IN|=0xA8;
}

////////////////////////////////////////////////////////////////////////////////////////////
//Button port 5 setup and related functions
///////////////////////////////////////////////////////////////////////////////////////////////

void set_up_P5( void )
{
	 /* P1REN |= 0x04;             // PULL up enable
	  P1DIR &= 0xFB;            // Be sure P 1.2 is input for interrupt
	  P1IN |=0x04;              // PULL UP
	  P1IE |= 0x04;             // enable P1.2 interrupt
	  P1IES |= 0x04;            // set P1.2 interrupt as down level trigger 
	  _BIS_SR(LPM3 +GIE);*/
	    P5DIR = 0xFF;             // Be sure all are inputs
	 
	    //P5REN |=0xFF;
 //P5DIR &=0x00;
        P5IN|=0x00;
}


////////////////////////////////////////////////////////////////////////////////////////////



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


/* The send_AT_result routine will send out the correct AT command
result taking into account the current settings of the V and Q
flags.

- If the Q flag is set, then no result is sent.
- If the Q flag os cleared, then:
- If the V flag is set, then send a verbose result
- If the V flag is cleared, then send a numeric result
*/

void send_AT_result( int cmdok )
{
  if( !(basic_at_command_flags & flag_q) ) { // want results
    if( basic_at_command_flags & flag_v ) {  // want verbose
      if( !cmdok ) mts_puts("\r\nERROR\r\n");
      else mts_puts("\r\nOK\r\n");
    }
    else { // want numeric
      if( !cmdok ) mts_puts("\r\n4\r\n"); // error
      else mts_puts("\r\n0\r\n"); // ok
    }
  }
}


// ######################################################################
// Basic AT commands
// ######################################################################

/* V command sets response type:
A. Takes up to 1 numeric arg
B. Arg of 1 sets verbose response
C. Arg of 0 sets numeric response
D. No arg sets default (verbose)
E. Returns 1 on success, 0 on error
*/
int V_set_response_type( void )
 {
int x;
char working_buffer[10];

  x = get_numeric_args(working_buffer);
  if(x > 1) return 0;  // error return
  if(!x)  {
      basic_at_command_flags |= flag_v; // take default
      return 1; // success return
  }
  else {  // we have one numeric arg
    switch(working_buffer[0])  {
    case '0':
        basic_at_command_flags &= ~flag_v; // want verbose
        x = 1;  // success return
        break;
    case '1':
        basic_at_command_flags |= flag_v;  // want numeric
        x = 1;  // success return
        break;
    default:
      x = 0;  // error return
    }
  }
     return x;
 }
 
 /* E command sets echo:
- Takes up to 1 numeric arg
- Arg of 1 sets echo on
- Arg of 0 sets echo off
- No arg sets default (off)
- Returns 1 on success, 0 on error
*/
int E_set_echo_mode( void )
 {
int x;
char working_buffer[10];

  x = get_numeric_args(working_buffer);
  if(x > 1) return 0;  // error return
  future_at_command_flags = basic_at_command_flags;  // get current values
  if(!x)  {
      future_at_command_flags &= ~flag_e; // take default
      future_at_command_flags |= chg_flag; // indicate changed flags
      return 1; // success return
  }
  else {  // we have one numeric arg
    switch(working_buffer[0])  {
    case '0':
        future_at_command_flags &= ~flag_e; // no echo
        future_at_command_flags |= chg_flag; // indicate changed flags
        x = 1;  // success return
        break;
    case '1':
        future_at_command_flags |= flag_e + chg_flag;  // want echo
        x = 1;  // success return
        break;
    default:
      x = 0;  // error return
    }
  }
     return x;
 }
 
 /* Q command sets result response (negative logic):
- Takes up to 1 numeric arg
- Arg of 1 sets no result response
- Arg of 0 sets result response on
- No arg sets default (result response on)
- Returns 1 on success, 0 on error
*/
int Q_set_response_mode( void )
 {
int x;
char working_buffer[10];

  x = get_numeric_args(working_buffer);
  if(x > 1) return 0;  // error return
  if(!x)  {
      basic_at_command_flags &= ~flag_q; // take default
      return 1; // success return
  }
  else {  // we have one numeric arg
    switch(working_buffer[0])  {
    case '0':
        basic_at_command_flags &= ~flag_q; // want response
        x = 1;  // success return
        break;
    case '1':
        basic_at_command_flags |= flag_q;  // no response
        x = 1;  // success return
        break;
    default:
      x = 0;  // error return
    }
  }
     return x;
 }
 
 // ######################################################################
// S register commands
// S120-127 set up the digital GPIOs.  Choices are:
// 0 is an input with no pull up or pull down
// 1 is an input with pull up
// 2 is an input with pull down
// 3 is an output
//
// S130-137 is data read or write to a digital GPIO, for example:
// S130?  commands to read GPIO0
// S131=1 command to SET the GPIO1 bit
// S132=0 command to RESET the GPIO2 bit
//
// S200-207 are commands to read individual ADC ports.
// They are read only.  Example:
// S200?  commands to read ADC0
// ######################################################################

int S_register_operation( void )
{
int x;
int reg_type, reg_target;
int ret_value;
char reg_mask;
char cx;
char working_buffer[10];

  x = get_numeric_args(working_buffer);  // get S register number
  if(!x) return 0;  // if no register number then error return
  x = mts_str2uint(working_buffer); // convert S reg into an uint
  reg_type = x / 10; // determine S reg type
  reg_target = x % 10; // determine S reg target
  ret_value = 1;  // init return value to success
  switch(reg_type)  {
  	
  	case 12:  // this type is GPIO pin setup registers
    if(reg_target > 7) return 0; // valid target values are 0 thru 7
    if(mts_getchar() != '=') return 0; // this type is write only
    x = get_numeric_args(working_buffer); // get value to set
    if(!x) working_buffer[0] = '0';  // if no value then default as input
    if( x > 1) return 0;  // invalid value
    reg_mask = 1 << reg_target;  // generate the register mask
    switch(working_buffer[0])  {  // switch on value to set
    case '0':  // make the pin an input
      P5DIR &= ~reg_mask; // set the pin to be an input
      P5REN &= ~reg_mask; // disable pull up and pull down
      break;
    case '1':  // make the pin an input with pull up.
      P5DIR &= ~reg_mask; // set the pin to be an input
      P5OUT |= reg_mask; // select pull up
      P5REN |= reg_mask; // and turn it on
      break;
    case '2':  // make the pin an input with pull down
      P5DIR &= ~reg_mask; // set the pin to be an input
      P5OUT &= ~reg_mask; // select pull down
      P5REN |= reg_mask; // and turn it on
      break;
    case '3':  // make the pin an output
      P5DIR |= reg_mask; // set the pin to be an output
      break;
    default:
      ret_value = 0;  // error return
    } // end of switch(working_buffer[0])
    break;
    
    case 13: // this type is read/write a GPIO pin
        if(reg_target > 7) return 0; // valid target values are 0 thru 7
        cx = mts_getchar();  // next char says what to do
        switch(cx) {
        case '=':  // want to write to a pin
          x = get_numeric_args(working_buffer); // get value to write
// Check if the value to write is either missing, or a bad value.
// If so, then give an error return
          if(!x || (x > 1) || (working_buffer[0] > '1')) ret_value = 0;
          else { // all is well, so set the pin
            reg_mask = 1 << reg_target;  // generate the register mask
            if(working_buffer[0] == '0')
              P5OUT &= ~reg_mask;  // set the pin to a zero
            else
              P5OUT |= reg_mask;  // set the pin to a one
          }
          break;
        case '?':  // want to read a pin
          reg_mask = 1 << reg_target;  // generate the register mask
          if(P5IN & reg_mask) mts_puts("\r\n1\r\n");
          else mts_puts("\r\n0\r\n");
          break;
        default:  // neither read nor write, so error
      ret_value = 0;  // error return
    } // end of switch(cx)
    break;

    case 20:  // this type is single pin analog to digital conversions
       if(reg_target > 7) return 0; // valid target values are 0 thru 7
       if(mts_getchar() != '?') return 0; // this type is read only
       ADC12CTL0 = SHT0_2 + ADC12ON; // Sampling time, turn on ADC12 
       ADC12CTL1 = SHP + CONSEQ_0; // Use sampling timer and single conversion
       ADC12CTL1 |= (reg_target << 12); // and select channel to use
       reg_mask = 1 << reg_target;  // generate the register mask
       ADC12IE = reg_mask;  // enable interrupt for selected target
       ADC12CTL0 |= ENC + ADC12SC;  // enable ADC and start conversion
       __bis_SR_register(LPM0_bits + GIE); // enable interrupts go into LPM0

       switch(reg_target) {
       case 0:
         mts_uint2str(ADC12MEM0, working_buffer); // convert result to string
         break;
       case 1:
         mts_uint2str(ADC12MEM1, working_buffer); // convert result to string
         break;
       case 2:
         mts_uint2str(ADC12MEM2, working_buffer); // convert result to string
         break;
       case 3:
         mts_uint2str(ADC12MEM3, working_buffer); // convert result to string
         break;
       case 4:
         mts_uint2str(ADC12MEM4, working_buffer); // convert result to string
         break;
       case 5:
         mts_uint2str(ADC12MEM5, working_buffer); // convert result to string
         break;
       case 6:
         mts_uint2str(ADC12MEM6, working_buffer); // convert result to string
         break;
       case 7:
         mts_uint2str(ADC12MEM7, working_buffer); // convert result to string
         break;
       } // end of switch(reg_target)
       mts_puts("\r\n");
       mts_puts(working_buffer); // output the ADC result
       mts_puts("\r\n");
       break;

  default: // unknown S register type
    ret_value = 0;  // error return
  } // end of switch(reg_type)
  return ret_value;
}  // end of S_register_operation()


// ######################################################################
// Extended AT commands
// Current extended commands are:
//
// +OAO  This puts the triple axis acceleromter into low power standby
//
// +OAx These commands read a vector of X.Y and Z values from the
// triple axis accelerometer.  There are 4 different choices:
// +OAW  This requests 'weak' (1.5g) g-force settings
// +OAL  This requests 'low' (2g) g-force settings
// +OAM  This requests 'medium' (4g) g-force settings
// +OAS  This requests 'strong' (6g) g-force settings
// See accelerometer data sheet for more information.  Note that none
// of these commands use a '?' as part of the command, and they all start
// with +O (upper case 'oh').  There are no numbers in these commands.
//
// ######################################################################

int AT_extended_command( void )
{
int ret_value;
char cmd_a, cmd_b, cmd_c;
char working_buffer[10];

  ret_value = 1;  // init success

/* For this architecture, extended commands can be described by up to 3
sub-commands.  Start by getting the first sub-command.
*/
  cmd_a = mts_toupper(mts_getchar());
  switch(cmd_a)  {
  case 'O':  // This is the letter 'O', meansing (sensor) Object
    cmd_b = mts_toupper(mts_getchar()); // get next sub-command
    switch(cmd_b)  {
    case 'A':  // this is an accelerometer object
      cmd_c = mts_toupper(mts_getchar()); // get next sub-command
      switch(cmd_c)  {
        case 'W':  // this requests 'weak' (1.5g) g-force settings
          P1OUT = 0x40;  // Enable and select 800 mv/g
          break;
        case 'L': // this requests 'low' (2g) g-force settings
          P1OUT = 0x60;  // Enable and select 600 mv/g
          break;
        case 'M': // this requests 'medium' (4g) g-force settings
          P1OUT = 0x50;  // Enable and select 300 mv/g
          break;
        case 'S': // this requests 'strong' (6g) g-force settings
          P1OUT = 0x70;  // Enable and select 200 mv/g
          break;
        case 'O': // this means turn off the accelerometer
          P1OUT = 0x00;   // Turn off accelerometer
          break;
        default: // unknown request
          ret_value = 0; // error return
        } // end of switch(cmd_c);
        if(ret_value != 0 && (cmd_c != 'O')) {  // if no error, get readings
          ADC12CTL0 = SHT0_2 + ADC12ON + MSC; // Set sampling time, turn on ADC12 and MSC
          ADC12CTL1 = SHP + CONSEQ_1;   // Use sampling timer and sequence of chans
          ADC12MCTL2 |= EOS;   // Indicate that channel A2 is end seq.
          ADC12IE = 0x04;      // Enable interrupt chan A2
          ADC12CTL0 |= ENC + ADC12SC;  // enable ADC and start conversion
          __bis_SR_register(LPM0_bits + GIE); // enable interrupts go into LPM0
         // Output the data from the acclerometer delimited by commas
          mts_uint2str(ADC12MEM0, working_buffer); // convert result to string
          mts_puts("\r\n");
          mts_puts("\r\n");
          mts_puts(working_buffer); // output the ADC result for X        
          mts_uint2str(ADC12MEM1, working_buffer); // convert result to string
          mts_puts(",");
          mts_puts(working_buffer); // output the ADC result for Y
          mts_uint2str(ADC12MEM2, working_buffer); // convert result to string
          mts_puts(",");
          mts_puts(working_buffer); // output the ADC result for Z
          mts_puts("\r\n");
        } // the if(ret_value != 0 && (cmd_c != 'O'))
      break;  // this break belongs to case 'A'      
    default:  // unknown sensor object
    ret_value = 0;  // error return
  } // end of switch(cmd_b)
  break;  // this break belongs to case 'O'
  
    default:  // unknown extended command
    ret_value = 0;  // error return
  } // end of switch(cmd_a)

  return ret_value;
} // end of AT_extended_command()



//////////////////////////////////////////////////////////////////////////////////
// Main function
//////////////////////////////////////////////////////////////////////////////////

int main( void )
{
	
//char mychar;
//char my_buffer[256];


// Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

// Initialize ID straps, clocks and input queues on the WASA board. 
  init_wasa_board();
  
  set_up_UART_A0();
  
 
// Set up and initialize the communication UART (A1) between CPU and Bluetooth 
  set_up_UART_A1();
  
  set_up_P5();
  
  set_up_adc12();
  
  mts_puts("clothes guarder running now!\r\n");
  
/////////////////////////////////////////////////////////////////////////////////////////////////
// command detector module
/////////////////////////////////////////////////////////////////////////////////////////////////

while(1)
{
	/*
	while(1) {    // loop around for the entire command line
                   //mts_puts("main looping begin!\r\n");
       P6IN &= 0xF8; 
      if((P6IN&0xa8)==0x28){
      	program_status=1;        // button 1 pressed
      	P5OUT|=0x00;              // light up LED 1
      		 	P5IN|=0x00; 
      	break;
      	
      }else if((P6IN&0xa8)==0x88){
   		program_status=2;        // button 2 pressed
      	P5OUT |= 0x00;              // light up LED 2
      		 	P5IN|=0x00;  
      	P1OUT = 0x40;  // Enable and select 800 mv/g
      	break;
      }else if((P6IN&0xa8)==0xa0){
      	program_status=3;        // button 2 pressed
      	P5OUT |= 0x00;              // light up LED 3
      		 	P5IN|=0x00; 
      	P1OUT = 0x70;  // Enable and select 200 mv/g
      	break;
      }
	}
	*/
	////////////////////////////////////////////////////////////////////////////////
	
	//P5OUT|=0xFF;              // light up LED 1
      		 	//P5IN|=0xFF; 
      		 	sound_alarm();

	
	//sensor_datasamp();

	
}


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
  ++BT_queue_in_ptr;                   // increment the pointer
  _BIC_SR_IRQ(LPM0_bits);           // Clear CPUOFF bit from 0(SR)
  UC1IE |= UCA1RXIE;                 // And enable UART A1 RX interrupt
  __bis_SR_register(GIE);          // And enable interupts
}

// For ADC12 analog to digital conversions
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR (void)
{
  ADC12CTL0 &= ~ENC;             // Conversion disabled
  ADC12IE = 0x0000;                // Disable all ADC interrupts
  ADC12IFG = 0x0000;              // Clear off all interrupt flags
  _BIC_SR_IRQ(LPM0_bits);        // Clear CPUOFF bit from 0(SR)
  __bis_SR_register(GIE);        // And enable interupts
}
///////////////////////////////////////////////////////////////////////////////
//AD Conversion control
///////////////////////////////////////////////////////////////////////////////


int UART_kbhit()
{
  if(BT_queue_in_ptr == BT_queue_out_ptr) return 0;
  else return 1;
}


int UART_getdata( void )
{ 
  if(!UART_kbhit()) {  // need to wait for a char
  	return 0xFFFF;
  //__bis_SR_register(LPM0_bits);   // Enter LPM0
  }
  //ecg_queue_out_ptr+=2;
  return BT_input_queue[BT_queue_out_ptr++];  
 // return ecg_input_queue[ecg_queue_out_ptr-2];  // return character
}

void UART_dataproc()
{ 
	char mybuff[16];
	//int i=0; int sum=0;
	//int counter=0;
	int temp=0;
	
	while(1)
	{
		temp=UART_getdata();
		if(0xFFFF!=temp)
		{
		   mts_uint2str(temp,mybuff);
		   mts_puts(mybuff);
		   mts_puts("\r\n");
		   
		}
		
	}
	/*
	switch(program_status)
	{
		case 1:
		    
			while(1)
			{
				for(i=0;i<5;i++)
				{
					temp=UART_getdata();
					if(0xFFFF!=temp)
					sum+=temp;
			
				}
				ecg_data[counter++]=sum/5;
				if (12==counter)
				{
					counter=0;
					sprintf(mybuff,"sensor_data processing\r\n");
 					mts_puts(mybuff);
					return;
				}
			}
			
		case 2:
		mts_puts("ADC STOP NOW!\r\n");
		break;
		
		case 3:
		break;
		default:
		break;
	}*/
	
}
///////////////////////////////////////////////////////////////////////////////
// acceleor meter data proc
void sensor_datasamp()
{    while(1)
           {
	char working_buffer[10];
	// The analog inputs are connected to P6.0 thru P6.7
  P6DIR = 0x00;             // Be sure all are inputs
  P6SEL = 0x57;            // and all are A/D channel inputs
  
  // We will use the internal ADC12OSC and default to single conversions.
  
  ADC12CTL0 = 0;  // be sure everything inits off, especially ENC
  ADC12CTL0 = SHT0_2 + ADC12ON; // Sampling time, turn on ADC12
  ADC12CTL1 = SHP + CONSEQ_2;     // Use sampling timer and single conversion
  ADC12MCTL0 = INCH_0;            // ref+=AVcc, channel = A0
  ADC12MCTL1 = INCH_1;            // ref+=AVcc, channel = A1
  ADC12MCTL2 = INCH_2;            // ref+=AVcc, channel = A2
  //ADC12MCTL3 = INCH_3;            // ref+=AVcc, channel = A3
  ADC12MCTL4 = INCH_4;            // ref+=AVcc, channel = A4
 // ADC12MCTL5 = INCH_5;            // ref+=AVcc, channel = A5
  ADC12MCTL6 = INCH_6;            // ref+=AVcc, channel = A6
 // ADC12MCTL7 = INCH_7;            // ref+=AVcc, channel = A7
 
	 ADC12CTL0 = SHT0_2 + ADC12ON + MSC; // Set sampling time, turn on ADC12 and MSC
          ADC12CTL1 = SHP + CONSEQ_1;   // Use sampling timer and sequence of chans
         // ADC12MCTL2 |= EOS;   // Indicate that channel A2 is end seq.
          //ADC12IE = 0x04;      // Enable interrupt chan A2
          ADC12MCTL6 |= EOS;   // Indicate that channel A2 is end seq.
          ADC12IE = 0x40;      // Enable interrupt chan A2
          ADC12CTL0 |= ENC + ADC12SC;  // enable ADC and start conversion
          __bis_SR_register(LPM0_bits + GIE); // enable interrupts go into LPM0
          
           P1OUT = 0x70;  // Enable and select 200 mv/g
          // P1OUT = 0x50;  // Enable and select 300 mv/g
         //  P1OUT = 0x60;  // Enable and select 600 mv/g
          // P1OUT = 0x40;  // Enable and select 800 mv/g
          
         // Output the data from the acclerometer delimited by commas
          mts_uint2str(ADC12MEM0, working_buffer); // convert result to string
          mts_puts("\r\n");
          mts_puts("\r\n");
          mts_puts(working_buffer); // output the ADC result for X        
          mts_uint2str(ADC12MEM1, working_buffer); // convert result to string
          mts_puts(",");
          mts_puts(working_buffer); // output the ADC result for Y
          mts_uint2str(ADC12MEM2, working_buffer); // convert result to string
          mts_puts(",");
          mts_puts(working_buffer); // output the ADC result for Z
          mts_puts("\r\n");
          
    //////////////////////////////////////////////////////////////////////////////////      
          
       mts_uint2str(ADC12MEM4, working_buffer); // light sensor 1
       mts_puts("\r\n");
       mts_puts(working_buffer); // output the ADC result
       mts_puts("\r\n");
       /*mts_uint2str(ADC12MEM6, working_buffer); // light sensor 2
       mts_puts("\r\n");
       mts_puts(working_buffer); // output the ADC result
       mts_puts("\r\n");*/
           }
}
///////////////////////////////////////////////////////////////////////////////
void sound_alarm()
{
	int i=0;
	int j=0;
	int z=0;
	while(1){
	       {
	       	
	       	for(i=0;i<30000;i++)
			{
				z=(z+1)*(z+1);
			} 
			
			P5OUT|=0xFF;              // light up LED 1
      		 	P5IN|=0xFF;
      		 	z=0;
	       for(j=0;j<30000;j++)
			{
					z=(z+1)*(z+1);
			} 
			P5OUT|=0x00;              // light up LED 1
      		 	P5IN|=0x00;
	       }
}
