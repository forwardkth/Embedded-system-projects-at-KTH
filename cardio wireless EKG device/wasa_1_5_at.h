/*********************************************************************
Copyright (C) <2009>  <M.T. Smith>

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
*********************************************************************/
#ifndef WASA_1_5_AT_H_
#define WASA_1_5_AT_H_
#endif /*WASA_1_5_AT_H_*/

// Globals.

//char input_queue[256];  // Circular input queue
unsigned char queue_in_ptr, queue_out_ptr;

char BT_input_queue[256];  // Circular input queue
unsigned char BT_queue_in_ptr, BT_queue_out_ptr;
unsigned long UinxTimeStamp;
#define MAX_DATA_CYCLE (2)
typedef enum
{
	general_idle,      // CPU ON,BT DEEP SLEEP
	program_sleep,     // CPU OFF,BT DEEP SLEEP
	data_gathering,    // CPU ON,BT DEEP SLEEP
	data_sending       // CPU ON, BT ACTIVE
	
}_program_status;

///////////////////////////////////////////////////////////////////




int ecg_workingbuf[120*MAX_DATA_CYCLE];//used to store the final ECG data.
//int ecg_data[120];
unsigned int data_in_ptr;

int ecg_input_queue[256];  // Circular input queue
unsigned char ecg_queue_in_ptr, ecg_queue_out_ptr;


unsigned char basic_at_command_flags;
unsigned char future_at_command_flags;

// End of globals.
/* Define bits in both basic_at_command_flags and future_at_command_flags.
The 'chg_flag' is ONLY defined for the future_at_command_flags.  It serve
to indicate that the flag set has changed, and so should be transferred to
the basic_at_command_flags at the END of the current command line.  We use
this method for those commands that should wait until the end of the line
to change these settings.  One such command is the echo (E) command.  If it
doesn't wait until the end of the command line, then the current command line
will start or stop echoing half way though it, which could cause problem.

Note that other commands, like the V or Q commands do not wait until the end
of the command line, as they should take effect immediately.
*/

#define flag_v 0x01
#define flag_e 0x02
#define flag_q 0x04
#define chg_flag 0x80

// These are some ASCII character definitions that are useful to use.  Makes
// the source code more readable.

#define TAB_ASCII 0x09
#define SEMICOLON_ASCII 0x3b
#define SPACE_ASCII 0x20
#define CR_ASCII 0x0d
