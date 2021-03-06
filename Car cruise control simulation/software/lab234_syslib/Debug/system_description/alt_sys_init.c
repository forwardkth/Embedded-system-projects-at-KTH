/* alt_sys_init.c - HAL initialisation source
 *
 * Machine generated for a CPU named "cpu" as defined in:
 * f:\lab2\software\lab234_syslib\..\..\DE2_Nios2_Lab2.ptf
 *
 * Generated: 2011-02-12 10:51:53.643
 *
 */

/*

DO NOT MODIFY THIS FILE

   Changing this file will have subtle consequences
   which will almost certainly lead to a nonfunctioning
   system. If you do modify this file, be aware that your
   changes will be overwritten and lost when this file
   is generated again.

DO NOT MODIFY THIS FILE

*/

/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2003 Altera Corporation, San Jose, California, USA.           *
* All rights reserved.                                                        *
*                                                                             *
* Permission is hereby granted, free of charge, to any person obtaining a     *
* copy of this software and associated documentation files (the "Software"),  *
* to deal in the Software without restriction, including without limitation   *
* the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
* and/or sell copies of the Software, and to permit persons to whom the       *
* Software is furnished to do so, subject to the following conditions:        *
*                                                                             *
* The above copyright notice and this permission notice shall be included in  *
* all copies or substantial portions of the Software.                         *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
* This agreement shall be governed in all respects by the laws of the State   *
* of California and by the laws of the United States of America.              *
*                                                                             *
******************************************************************************/

#include "system.h"
#include "sys/alt_sys_init.h"
#include "sys/alt_irq.h"
#include "nios2.h"

#include <stddef.h>

/*
 * device headers
 *
 */

#include "altera_avalon_uart.h"
#include "altera_avalon_jtag_uart.h"
#include "altera_avalon_timer.h"

/*
 * Allocate the device storage
 *
 */

ALTERA_AVALON_UART_INSTANCE( UART_0, uart_0 );
ALTERA_AVALON_JTAG_UART_INSTANCE( JTAG_UART, jtag_uart );
ALTERA_AVALON_UART_INSTANCE( UART_1, uart_1 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_0, timer_0 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER100U, timer100u );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_1, timer_1 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_2, timer_2 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_3, timer_3 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_4, timer_4 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_5, timer_5 );
ALTERA_AVALON_TIMER_INSTANCE( TIMER_6, timer_6 );
ALTERA_AVALON_UART_INSTANCE( UART2, uart2 );

/*
 * Initialize the interrupt controller devices
 * and then enable interrupts in the CPU.
 * Called before alt_sys_init().
 * The "base" parameter is ignored and only
 * present for backwards-compatibility.
 */

void alt_irq_init ( const void* base )
{
    NIOS2_WRITE_IENABLE(0);
    alt_irq_cpu_enable_interrupts();
}

/*
 * Initialise the devices
 *
 */

void alt_sys_init( void )
{
    ALTERA_AVALON_TIMER_INIT( TIMER_0, timer_0 );
    ALTERA_AVALON_TIMER_INIT( TIMER100U, timer100u );
    ALTERA_AVALON_TIMER_INIT( TIMER_1, timer_1 );
    ALTERA_AVALON_TIMER_INIT( TIMER_2, timer_2 );
    ALTERA_AVALON_TIMER_INIT( TIMER_3, timer_3 );
    ALTERA_AVALON_TIMER_INIT( TIMER_4, timer_4 );
    ALTERA_AVALON_TIMER_INIT( TIMER_5, timer_5 );
    ALTERA_AVALON_TIMER_INIT( TIMER_6, timer_6 );
    ALTERA_AVALON_UART_INIT( UART_0, uart_0 );
    ALTERA_AVALON_JTAG_UART_INIT( JTAG_UART, jtag_uart );
    ALTERA_AVALON_UART_INIT( UART_1, uart_1 );
    ALTERA_AVALON_UART_INIT( UART2, uart2 );
}
