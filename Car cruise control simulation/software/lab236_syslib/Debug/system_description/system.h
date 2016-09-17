/* system.h
 *
 * Machine generated for a CPU named "cpu" as defined in:
 * f:\lab2\software\lab236_syslib\..\..\DE2_Nios2_Lab2.ptf
 *
 * Generated: 2011-02-12 23:13:11.285
 *
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

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

/*
 * system configuration
 *
 */

#define ALT_SYSTEM_NAME "Nios2system"
#define ALT_CPU_NAME "cpu"
#define ALT_CPU_ARCHITECTURE "altera_nios2"
#define ALT_DEVICE_FAMILY "CYCLONEII"
#define ALT_STDIN "/dev/jtag_uart"
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN_BASE 0x00000800
#define ALT_STDIN_DEV jtag_uart
#define ALT_STDIN_PRESENT
#define ALT_STDOUT "/dev/jtag_uart"
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT_BASE 0x00000800
#define ALT_STDOUT_DEV jtag_uart
#define ALT_STDOUT_PRESENT
#define ALT_STDERR "/dev/jtag_uart"
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDERR_BASE 0x00000800
#define ALT_STDERR_DEV jtag_uart
#define ALT_STDERR_PRESENT
#define ALT_CPU_FREQ 50000000
#define ALT_IRQ_BASE NULL
#define ALT_LEGACY_INTERRUPT_API_PRESENT

/*
 * processor configuration
 *
 */

#define NIOS2_CPU_IMPLEMENTATION "small"
#define NIOS2_BIG_ENDIAN 0
#define NIOS2_INTERRUPT_CONTROLLER_ID 0

#define NIOS2_ICACHE_SIZE 4096
#define NIOS2_DCACHE_SIZE 0
#define NIOS2_ICACHE_LINE_SIZE 32
#define NIOS2_ICACHE_LINE_SIZE_LOG2 5
#define NIOS2_DCACHE_LINE_SIZE 0
#define NIOS2_DCACHE_LINE_SIZE_LOG2 0
#define NIOS2_FLUSHDA_SUPPORTED

#define NIOS2_EXCEPTION_ADDR 0x00800020
#define NIOS2_RESET_ADDR 0x00800000
#define NIOS2_BREAK_ADDR 0x00000020

#define NIOS2_HAS_DEBUG_STUB

#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0

/*
 * A define for each class of peripheral
 *
 */

#define __ALTERA_AVALON_NEW_SDRAM_CONTROLLER
#define __ALTERA_AVALON_UART
#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_UP_AVALON_CHARACTER_LCD
#define __ALTERA_AVALON_TIMER
#define __ALTERA_AVALON_PIO

/*
 * sdram configuration
 *
 */

#define SDRAM_NAME "/dev/sdram"
#define SDRAM_TYPE "altera_avalon_new_sdram_controller"
#define SDRAM_BASE 0x00800000
#define SDRAM_SPAN 8388608
#define SDRAM_REGISTER_DATA_IN 1
#define SDRAM_SIM_MODEL_BASE 0
#define SDRAM_SDRAM_DATA_WIDTH 16
#define SDRAM_SDRAM_ADDR_WIDTH 12
#define SDRAM_SDRAM_ROW_WIDTH 12
#define SDRAM_SDRAM_COL_WIDTH 8
#define SDRAM_SDRAM_NUM_CHIPSELECTS 1
#define SDRAM_SDRAM_NUM_BANKS 4
#define SDRAM_REFRESH_PERIOD 15.625
#define SDRAM_POWERUP_DELAY 100.0
#define SDRAM_CAS_LATENCY 3
#define SDRAM_T_RFC 70.0
#define SDRAM_T_RP 20.0
#define SDRAM_T_MRD 3
#define SDRAM_T_RCD 20.0
#define SDRAM_T_AC 5.5
#define SDRAM_T_WR 14.0
#define SDRAM_INIT_REFRESH_COMMANDS 2
#define SDRAM_INIT_NOP_DELAY 0.0
#define SDRAM_SHARED_DATA 0
#define SDRAM_SDRAM_BANK_WIDTH 2
#define SDRAM_TRISTATE_BRIDGE_SLAVE ""
#define SDRAM_STARVATION_INDICATOR 0
#define SDRAM_IS_INITIALIZED 1
#define ALT_MODULE_CLASS_sdram altera_avalon_new_sdram_controller

/*
 * uart_0 configuration
 *
 */

#define UART_0_NAME "/dev/uart_0"
#define UART_0_TYPE "altera_avalon_uart"
#define UART_0_BASE 0x00000860
#define UART_0_SPAN 32
#define UART_0_IRQ 4
#define UART_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART_0_BAUD 115200
#define UART_0_DATA_BITS 8
#define UART_0_FIXED_BAUD 1
#define UART_0_PARITY 'N'
#define UART_0_STOP_BITS 1
#define UART_0_USE_CTS_RTS 0
#define UART_0_USE_EOP_REGISTER 0
#define UART_0_SIM_TRUE_BAUD 0
#define UART_0_SIM_CHAR_STREAM ""
#define UART_0_FREQ 50000000
#define ALT_MODULE_CLASS_uart_0 altera_avalon_uart

/*
 * jtag_uart configuration
 *
 */

#define JTAG_UART_NAME "/dev/jtag_uart"
#define JTAG_UART_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_BASE 0x00000800
#define JTAG_UART_SPAN 8
#define JTAG_UART_IRQ 0
#define JTAG_UART_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_READ_CHAR_STREAM ""
#define JTAG_UART_SHOWASCII 0
#define JTAG_UART_READ_LE 0
#define JTAG_UART_WRITE_LE 0
#define JTAG_UART_ALTERA_SHOW_UNRELEASED_JTAG_UART_FEATURES 0
#define ALT_MODULE_CLASS_jtag_uart altera_avalon_jtag_uart

/*
 * uart_1 configuration
 *
 */

#define UART_1_NAME "/dev/uart_1"
#define UART_1_TYPE "altera_avalon_uart"
#define UART_1_BASE 0x00000880
#define UART_1_SPAN 32
#define UART_1_IRQ 5
#define UART_1_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART_1_BAUD 115200
#define UART_1_DATA_BITS 8
#define UART_1_FIXED_BAUD 1
#define UART_1_PARITY 'N'
#define UART_1_STOP_BITS 1
#define UART_1_USE_CTS_RTS 0
#define UART_1_USE_EOP_REGISTER 0
#define UART_1_SIM_TRUE_BAUD 0
#define UART_1_SIM_CHAR_STREAM ""
#define UART_1_FREQ 50000000
#define ALT_MODULE_CLASS_uart_1 altera_avalon_uart

/*
 * character_lcd configuration
 *
 */

#define CHARACTER_LCD_NAME "/dev/character_lcd"
#define CHARACTER_LCD_TYPE "altera_up_avalon_character_lcd"
#define CHARACTER_LCD_BASE 0x00000808
#define CHARACTER_LCD_SPAN 8
#define ALT_MODULE_CLASS_character_lcd altera_up_avalon_character_lcd

/*
 * timer_0 configuration
 *
 */

#define TIMER_0_NAME "/dev/timer_0"
#define TIMER_0_TYPE "altera_avalon_timer"
#define TIMER_0_BASE 0x00000820
#define TIMER_0_SPAN 32
#define TIMER_0_IRQ 1
#define TIMER_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_0_ALWAYS_RUN 0
#define TIMER_0_FIXED_PERIOD 0
#define TIMER_0_SNAPSHOT 1
#define TIMER_0_PERIOD 1.0
#define TIMER_0_PERIOD_UNITS "ms"
#define TIMER_0_RESET_OUTPUT 0
#define TIMER_0_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_0_MULT 0.001
#define TIMER_0_COUNTER_SIZE 32
#define TIMER_0_FREQ 50000000
#define ALT_MODULE_CLASS_timer_0 altera_avalon_timer

/*
 * de2_pio_redled18 configuration
 *
 */

#define DE2_PIO_REDLED18_NAME "/dev/de2_pio_redled18"
#define DE2_PIO_REDLED18_TYPE "altera_avalon_pio"
#define DE2_PIO_REDLED18_BASE 0x00000810
#define DE2_PIO_REDLED18_SPAN 16
#define DE2_PIO_REDLED18_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_REDLED18_DRIVEN_SIM_VALUE 0
#define DE2_PIO_REDLED18_HAS_TRI 0
#define DE2_PIO_REDLED18_HAS_OUT 1
#define DE2_PIO_REDLED18_HAS_IN 0
#define DE2_PIO_REDLED18_CAPTURE 0
#define DE2_PIO_REDLED18_DATA_WIDTH 18
#define DE2_PIO_REDLED18_EDGE_TYPE "NONE"
#define DE2_PIO_REDLED18_IRQ_TYPE "NONE"
#define DE2_PIO_REDLED18_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_redled18 altera_avalon_pio

/*
 * de2_pio_keys4 configuration
 *
 */

#define DE2_PIO_KEYS4_NAME "/dev/de2_pio_keys4"
#define DE2_PIO_KEYS4_TYPE "altera_avalon_pio"
#define DE2_PIO_KEYS4_BASE 0x00000840
#define DE2_PIO_KEYS4_SPAN 16
#define DE2_PIO_KEYS4_IRQ 2
#define DE2_PIO_KEYS4_IRQ_INTERRUPT_CONTROLLER_ID 0
#define DE2_PIO_KEYS4_DO_TEST_BENCH_WIRING 1
#define DE2_PIO_KEYS4_DRIVEN_SIM_VALUE 15
#define DE2_PIO_KEYS4_HAS_TRI 0
#define DE2_PIO_KEYS4_HAS_OUT 0
#define DE2_PIO_KEYS4_HAS_IN 1
#define DE2_PIO_KEYS4_CAPTURE 1
#define DE2_PIO_KEYS4_DATA_WIDTH 4
#define DE2_PIO_KEYS4_EDGE_TYPE "ANY"
#define DE2_PIO_KEYS4_IRQ_TYPE "EDGE"
#define DE2_PIO_KEYS4_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_keys4 altera_avalon_pio

/*
 * de2_pio_toggles18 configuration
 *
 */

#define DE2_PIO_TOGGLES18_NAME "/dev/de2_pio_toggles18"
#define DE2_PIO_TOGGLES18_TYPE "altera_avalon_pio"
#define DE2_PIO_TOGGLES18_BASE 0x00000850
#define DE2_PIO_TOGGLES18_SPAN 16
#define DE2_PIO_TOGGLES18_IRQ 3
#define DE2_PIO_TOGGLES18_IRQ_INTERRUPT_CONTROLLER_ID 0
#define DE2_PIO_TOGGLES18_DO_TEST_BENCH_WIRING 1
#define DE2_PIO_TOGGLES18_DRIVEN_SIM_VALUE 279
#define DE2_PIO_TOGGLES18_HAS_TRI 0
#define DE2_PIO_TOGGLES18_HAS_OUT 0
#define DE2_PIO_TOGGLES18_HAS_IN 1
#define DE2_PIO_TOGGLES18_CAPTURE 1
#define DE2_PIO_TOGGLES18_DATA_WIDTH 18
#define DE2_PIO_TOGGLES18_EDGE_TYPE "ANY"
#define DE2_PIO_TOGGLES18_IRQ_TYPE "EDGE"
#define DE2_PIO_TOGGLES18_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_toggles18 altera_avalon_pio

/*
 * timer100u configuration
 *
 */

#define TIMER100U_NAME "/dev/timer100u"
#define TIMER100U_TYPE "altera_avalon_timer"
#define TIMER100U_BASE 0x00000900
#define TIMER100U_SPAN 32
#define TIMER100U_IRQ 9
#define TIMER100U_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER100U_ALWAYS_RUN 1
#define TIMER100U_FIXED_PERIOD 1
#define TIMER100U_SNAPSHOT 0
#define TIMER100U_PERIOD 100.0
#define TIMER100U_PERIOD_UNITS "us"
#define TIMER100U_RESET_OUTPUT 0
#define TIMER100U_TIMEOUT_PULSE_OUTPUT 0
#define TIMER100U_MULT 0.001
#define TIMER100U_COUNTER_SIZE 32
#define TIMER100U_FREQ 50000000
#define ALT_MODULE_CLASS_timer100u altera_avalon_timer

/*
 * de2_pio_jp1_in1_5 configuration
 *
 */

#define DE2_PIO_JP1_IN1_5_NAME "/dev/de2_pio_jp1_in1_5"
#define DE2_PIO_JP1_IN1_5_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_IN1_5_BASE 0x000008a0
#define DE2_PIO_JP1_IN1_5_SPAN 16
#define DE2_PIO_JP1_IN1_5_IRQ 6
#define DE2_PIO_JP1_IN1_5_IRQ_INTERRUPT_CONTROLLER_ID 0
#define DE2_PIO_JP1_IN1_5_DO_TEST_BENCH_WIRING 1
#define DE2_PIO_JP1_IN1_5_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_IN1_5_HAS_TRI 0
#define DE2_PIO_JP1_IN1_5_HAS_OUT 0
#define DE2_PIO_JP1_IN1_5_HAS_IN 1
#define DE2_PIO_JP1_IN1_5_CAPTURE 1
#define DE2_PIO_JP1_IN1_5_DATA_WIDTH 5
#define DE2_PIO_JP1_IN1_5_EDGE_TYPE "ANY"
#define DE2_PIO_JP1_IN1_5_IRQ_TYPE "EDGE"
#define DE2_PIO_JP1_IN1_5_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_in1_5 altera_avalon_pio

/*
 * de2_pio_jp1_in2_8 configuration
 *
 */

#define DE2_PIO_JP1_IN2_8_NAME "/dev/de2_pio_jp1_in2_8"
#define DE2_PIO_JP1_IN2_8_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_IN2_8_BASE 0x000008b0
#define DE2_PIO_JP1_IN2_8_SPAN 16
#define DE2_PIO_JP1_IN2_8_IRQ 7
#define DE2_PIO_JP1_IN2_8_IRQ_INTERRUPT_CONTROLLER_ID 0
#define DE2_PIO_JP1_IN2_8_DO_TEST_BENCH_WIRING 1
#define DE2_PIO_JP1_IN2_8_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_IN2_8_HAS_TRI 0
#define DE2_PIO_JP1_IN2_8_HAS_OUT 0
#define DE2_PIO_JP1_IN2_8_HAS_IN 1
#define DE2_PIO_JP1_IN2_8_CAPTURE 1
#define DE2_PIO_JP1_IN2_8_DATA_WIDTH 8
#define DE2_PIO_JP1_IN2_8_EDGE_TYPE "ANY"
#define DE2_PIO_JP1_IN2_8_IRQ_TYPE "EDGE"
#define DE2_PIO_JP1_IN2_8_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_in2_8 altera_avalon_pio

/*
 * de2_pio_jp1_in3_5 configuration
 *
 */

#define DE2_PIO_JP1_IN3_5_NAME "/dev/de2_pio_jp1_in3_5"
#define DE2_PIO_JP1_IN3_5_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_IN3_5_BASE 0x000008c0
#define DE2_PIO_JP1_IN3_5_SPAN 16
#define DE2_PIO_JP1_IN3_5_IRQ 8
#define DE2_PIO_JP1_IN3_5_IRQ_INTERRUPT_CONTROLLER_ID 0
#define DE2_PIO_JP1_IN3_5_DO_TEST_BENCH_WIRING 1
#define DE2_PIO_JP1_IN3_5_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_IN3_5_HAS_TRI 0
#define DE2_PIO_JP1_IN3_5_HAS_OUT 0
#define DE2_PIO_JP1_IN3_5_HAS_IN 1
#define DE2_PIO_JP1_IN3_5_CAPTURE 1
#define DE2_PIO_JP1_IN3_5_DATA_WIDTH 5
#define DE2_PIO_JP1_IN3_5_EDGE_TYPE "ANY"
#define DE2_PIO_JP1_IN3_5_IRQ_TYPE "EDGE"
#define DE2_PIO_JP1_IN3_5_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_in3_5 altera_avalon_pio

/*
 * de2_pio_jp1_out1_5 configuration
 *
 */

#define DE2_PIO_JP1_OUT1_5_NAME "/dev/de2_pio_jp1_out1_5"
#define DE2_PIO_JP1_OUT1_5_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_OUT1_5_BASE 0x000008d0
#define DE2_PIO_JP1_OUT1_5_SPAN 16
#define DE2_PIO_JP1_OUT1_5_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_JP1_OUT1_5_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_OUT1_5_HAS_TRI 0
#define DE2_PIO_JP1_OUT1_5_HAS_OUT 1
#define DE2_PIO_JP1_OUT1_5_HAS_IN 0
#define DE2_PIO_JP1_OUT1_5_CAPTURE 0
#define DE2_PIO_JP1_OUT1_5_DATA_WIDTH 5
#define DE2_PIO_JP1_OUT1_5_EDGE_TYPE "NONE"
#define DE2_PIO_JP1_OUT1_5_IRQ_TYPE "NONE"
#define DE2_PIO_JP1_OUT1_5_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_out1_5 altera_avalon_pio

/*
 * de2_pio_jp1_out2_8 configuration
 *
 */

#define DE2_PIO_JP1_OUT2_8_NAME "/dev/de2_pio_jp1_out2_8"
#define DE2_PIO_JP1_OUT2_8_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_OUT2_8_BASE 0x000008e0
#define DE2_PIO_JP1_OUT2_8_SPAN 16
#define DE2_PIO_JP1_OUT2_8_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_JP1_OUT2_8_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_OUT2_8_HAS_TRI 0
#define DE2_PIO_JP1_OUT2_8_HAS_OUT 1
#define DE2_PIO_JP1_OUT2_8_HAS_IN 0
#define DE2_PIO_JP1_OUT2_8_CAPTURE 0
#define DE2_PIO_JP1_OUT2_8_DATA_WIDTH 8
#define DE2_PIO_JP1_OUT2_8_EDGE_TYPE "NONE"
#define DE2_PIO_JP1_OUT2_8_IRQ_TYPE "NONE"
#define DE2_PIO_JP1_OUT2_8_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_out2_8 altera_avalon_pio

/*
 * de2_pio_jp1_out3_5 configuration
 *
 */

#define DE2_PIO_JP1_OUT3_5_NAME "/dev/de2_pio_jp1_out3_5"
#define DE2_PIO_JP1_OUT3_5_TYPE "altera_avalon_pio"
#define DE2_PIO_JP1_OUT3_5_BASE 0x000008f0
#define DE2_PIO_JP1_OUT3_5_SPAN 16
#define DE2_PIO_JP1_OUT3_5_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_JP1_OUT3_5_DRIVEN_SIM_VALUE 0
#define DE2_PIO_JP1_OUT3_5_HAS_TRI 0
#define DE2_PIO_JP1_OUT3_5_HAS_OUT 1
#define DE2_PIO_JP1_OUT3_5_HAS_IN 0
#define DE2_PIO_JP1_OUT3_5_CAPTURE 0
#define DE2_PIO_JP1_OUT3_5_DATA_WIDTH 5
#define DE2_PIO_JP1_OUT3_5_EDGE_TYPE "NONE"
#define DE2_PIO_JP1_OUT3_5_IRQ_TYPE "NONE"
#define DE2_PIO_JP1_OUT3_5_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_jp1_out3_5 altera_avalon_pio

/*
 * timer_1 configuration
 *
 */

#define TIMER_1_NAME "/dev/timer_1"
#define TIMER_1_TYPE "altera_avalon_timer"
#define TIMER_1_BASE 0x00000920
#define TIMER_1_SPAN 32
#define TIMER_1_IRQ 10
#define TIMER_1_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_1_ALWAYS_RUN 0
#define TIMER_1_FIXED_PERIOD 0
#define TIMER_1_SNAPSHOT 1
#define TIMER_1_PERIOD 1.0
#define TIMER_1_PERIOD_UNITS "ms"
#define TIMER_1_RESET_OUTPUT 0
#define TIMER_1_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_1_MULT 0.001
#define TIMER_1_COUNTER_SIZE 32
#define TIMER_1_FREQ 50000000
#define ALT_MODULE_CLASS_timer_1 altera_avalon_timer

/*
 * timer_2 configuration
 *
 */

#define TIMER_2_NAME "/dev/timer_2"
#define TIMER_2_TYPE "altera_avalon_timer"
#define TIMER_2_BASE 0x00000940
#define TIMER_2_SPAN 32
#define TIMER_2_IRQ 11
#define TIMER_2_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_2_ALWAYS_RUN 0
#define TIMER_2_FIXED_PERIOD 0
#define TIMER_2_SNAPSHOT 1
#define TIMER_2_PERIOD 1.0
#define TIMER_2_PERIOD_UNITS "ms"
#define TIMER_2_RESET_OUTPUT 0
#define TIMER_2_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_2_MULT 0.001
#define TIMER_2_COUNTER_SIZE 32
#define TIMER_2_FREQ 50000000
#define ALT_MODULE_CLASS_timer_2 altera_avalon_timer

/*
 * timer_3 configuration
 *
 */

#define TIMER_3_NAME "/dev/timer_3"
#define TIMER_3_TYPE "altera_avalon_timer"
#define TIMER_3_BASE 0x00000960
#define TIMER_3_SPAN 32
#define TIMER_3_IRQ 12
#define TIMER_3_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_3_ALWAYS_RUN 0
#define TIMER_3_FIXED_PERIOD 0
#define TIMER_3_SNAPSHOT 1
#define TIMER_3_PERIOD 1.0
#define TIMER_3_PERIOD_UNITS "ms"
#define TIMER_3_RESET_OUTPUT 0
#define TIMER_3_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_3_MULT 0.001
#define TIMER_3_COUNTER_SIZE 32
#define TIMER_3_FREQ 50000000
#define ALT_MODULE_CLASS_timer_3 altera_avalon_timer

/*
 * timer_4 configuration
 *
 */

#define TIMER_4_NAME "/dev/timer_4"
#define TIMER_4_TYPE "altera_avalon_timer"
#define TIMER_4_BASE 0x00000980
#define TIMER_4_SPAN 32
#define TIMER_4_IRQ 13
#define TIMER_4_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_4_ALWAYS_RUN 0
#define TIMER_4_FIXED_PERIOD 0
#define TIMER_4_SNAPSHOT 1
#define TIMER_4_PERIOD 1.0
#define TIMER_4_PERIOD_UNITS "ms"
#define TIMER_4_RESET_OUTPUT 0
#define TIMER_4_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_4_MULT 0.001
#define TIMER_4_COUNTER_SIZE 32
#define TIMER_4_FREQ 50000000
#define ALT_MODULE_CLASS_timer_4 altera_avalon_timer

/*
 * timer_5 configuration
 *
 */

#define TIMER_5_NAME "/dev/timer_5"
#define TIMER_5_TYPE "altera_avalon_timer"
#define TIMER_5_BASE 0x000009a0
#define TIMER_5_SPAN 32
#define TIMER_5_IRQ 14
#define TIMER_5_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_5_ALWAYS_RUN 0
#define TIMER_5_FIXED_PERIOD 0
#define TIMER_5_SNAPSHOT 1
#define TIMER_5_PERIOD 1.0
#define TIMER_5_PERIOD_UNITS "ms"
#define TIMER_5_RESET_OUTPUT 0
#define TIMER_5_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_5_MULT 0.001
#define TIMER_5_COUNTER_SIZE 32
#define TIMER_5_FREQ 50000000
#define ALT_MODULE_CLASS_timer_5 altera_avalon_timer

/*
 * timer_6 configuration
 *
 */

#define TIMER_6_NAME "/dev/timer_6"
#define TIMER_6_TYPE "altera_avalon_timer"
#define TIMER_6_BASE 0x000009c0
#define TIMER_6_SPAN 32
#define TIMER_6_IRQ 15
#define TIMER_6_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_6_ALWAYS_RUN 0
#define TIMER_6_FIXED_PERIOD 0
#define TIMER_6_SNAPSHOT 1
#define TIMER_6_PERIOD 1.0
#define TIMER_6_PERIOD_UNITS "ms"
#define TIMER_6_RESET_OUTPUT 0
#define TIMER_6_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_6_MULT 0.001
#define TIMER_6_COUNTER_SIZE 32
#define TIMER_6_FREQ 50000000
#define ALT_MODULE_CLASS_timer_6 altera_avalon_timer

/*
 * de2_pio_hex_low28 configuration
 *
 */

#define DE2_PIO_HEX_LOW28_NAME "/dev/de2_pio_hex_low28"
#define DE2_PIO_HEX_LOW28_TYPE "altera_avalon_pio"
#define DE2_PIO_HEX_LOW28_BASE 0x000009f0
#define DE2_PIO_HEX_LOW28_SPAN 16
#define DE2_PIO_HEX_LOW28_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_HEX_LOW28_DRIVEN_SIM_VALUE 0
#define DE2_PIO_HEX_LOW28_HAS_TRI 0
#define DE2_PIO_HEX_LOW28_HAS_OUT 1
#define DE2_PIO_HEX_LOW28_HAS_IN 0
#define DE2_PIO_HEX_LOW28_CAPTURE 0
#define DE2_PIO_HEX_LOW28_DATA_WIDTH 28
#define DE2_PIO_HEX_LOW28_EDGE_TYPE "NONE"
#define DE2_PIO_HEX_LOW28_IRQ_TYPE "NONE"
#define DE2_PIO_HEX_LOW28_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_hex_low28 altera_avalon_pio

/*
 * de2_pio_hex_high28 configuration
 *
 */

#define DE2_PIO_HEX_HIGH28_NAME "/dev/de2_pio_hex_high28"
#define DE2_PIO_HEX_HIGH28_TYPE "altera_avalon_pio"
#define DE2_PIO_HEX_HIGH28_BASE 0x00000a00
#define DE2_PIO_HEX_HIGH28_SPAN 16
#define DE2_PIO_HEX_HIGH28_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_HEX_HIGH28_DRIVEN_SIM_VALUE 0
#define DE2_PIO_HEX_HIGH28_HAS_TRI 0
#define DE2_PIO_HEX_HIGH28_HAS_OUT 1
#define DE2_PIO_HEX_HIGH28_HAS_IN 0
#define DE2_PIO_HEX_HIGH28_CAPTURE 0
#define DE2_PIO_HEX_HIGH28_DATA_WIDTH 28
#define DE2_PIO_HEX_HIGH28_EDGE_TYPE "NONE"
#define DE2_PIO_HEX_HIGH28_IRQ_TYPE "NONE"
#define DE2_PIO_HEX_HIGH28_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_hex_high28 altera_avalon_pio

/*
 * de2_pio_greenled9 configuration
 *
 */

#define DE2_PIO_GREENLED9_NAME "/dev/de2_pio_greenled9"
#define DE2_PIO_GREENLED9_TYPE "altera_avalon_pio"
#define DE2_PIO_GREENLED9_BASE 0x00000a10
#define DE2_PIO_GREENLED9_SPAN 16
#define DE2_PIO_GREENLED9_DO_TEST_BENCH_WIRING 0
#define DE2_PIO_GREENLED9_DRIVEN_SIM_VALUE 0
#define DE2_PIO_GREENLED9_HAS_TRI 0
#define DE2_PIO_GREENLED9_HAS_OUT 1
#define DE2_PIO_GREENLED9_HAS_IN 0
#define DE2_PIO_GREENLED9_CAPTURE 0
#define DE2_PIO_GREENLED9_DATA_WIDTH 9
#define DE2_PIO_GREENLED9_EDGE_TYPE "NONE"
#define DE2_PIO_GREENLED9_IRQ_TYPE "NONE"
#define DE2_PIO_GREENLED9_FREQ 50000000
#define ALT_MODULE_CLASS_de2_pio_greenled9 altera_avalon_pio

/*
 * uart2 configuration
 *
 */

#define UART2_NAME "/dev/uart2"
#define UART2_TYPE "altera_avalon_uart"
#define UART2_BASE 0x00000a20
#define UART2_SPAN 32
#define UART2_IRQ 17
#define UART2_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART2_BAUD 19200
#define UART2_DATA_BITS 8
#define UART2_FIXED_BAUD 1
#define UART2_PARITY 'N'
#define UART2_STOP_BITS 1
#define UART2_USE_CTS_RTS 0
#define UART2_USE_EOP_REGISTER 0
#define UART2_SIM_TRUE_BAUD 0
#define UART2_SIM_CHAR_STREAM ""
#define UART2_FREQ 50000000
#define ALT_MODULE_CLASS_uart2 altera_avalon_uart

/*
 * MicroC/OS-II configuration
 *
 */

#define ALT_MAX_FD 32
#define OS_MAX_TASKS 10
#define OS_LOWEST_PRIO 20
#define OS_FLAG_EN 1
#define OS_THREAD_SAFE_NEWLIB 1
#define OS_MUTEX_EN 1
#define OS_SEM_EN 1
#define OS_MBOX_EN 1
#define OS_Q_EN 1
#define OS_MEM_EN 1
#define OS_FLAG_WAIT_CLR_EN 1
#define OS_FLAG_ACCEPT_EN 1
#define OS_FLAG_DEL_EN 1
#define OS_FLAG_QUERY_EN 1
#define OS_FLAG_NAME_SIZE 32
#define OS_MAX_FLAGS 20
#define OS_FLAGS_NBITS 16
#define OS_MUTEX_ACCEPT_EN 1
#define OS_MUTEX_DEL_EN 1
#define OS_MUTEX_QUERY_EN 1
#define OS_SEM_ACCEPT_EN 1
#define OS_SEM_SET_EN 1
#define OS_SEM_DEL_EN 1
#define OS_SEM_QUERY_EN 1
#define OS_MBOX_ACCEPT_EN 1
#define OS_MBOX_DEL_EN 1
#define OS_MBOX_POST_EN 1
#define OS_MBOX_POST_OPT_EN 1
#define OS_MBOX_QUERY_EN 1
#define OS_Q_ACCEPT_EN 1
#define OS_Q_DEL_EN 1
#define OS_Q_FLUSH_EN 1
#define OS_Q_POST_EN 1
#define OS_Q_POST_FRONT_EN 1
#define OS_Q_POST_OPT_EN 1
#define OS_Q_QUERY_EN 1
#define OS_MAX_QS 20
#define OS_MEM_QUERY_EN 1
#define OS_MEM_NAME_SIZE 32
#define OS_MAX_MEM_PART 60
#define OS_ARG_CHK_EN 1
#define OS_CPU_HOOKS_EN 1
#define OS_DEBUG_EN 1
#define OS_SCHED_LOCK_EN 1
#define OS_TASK_STAT_EN 1
#define OS_TASK_STAT_STK_CHK_EN 1
#define OS_TICK_STEP_EN 1
#define OS_EVENT_NAME_SIZE 32
#define OS_MAX_EVENTS 60
#define OS_TASK_IDLE_STK_SIZE 512
#define OS_TASK_STAT_STK_SIZE 512
#define OS_TASK_CHANGE_PRIO_EN 1
#define OS_TASK_CREATE_EN 1
#define OS_TASK_CREATE_EXT_EN 1
#define OS_TASK_DEL_EN 1
#define OS_TASK_NAME_SIZE 32
#define OS_TASK_PROFILE_EN 1
#define OS_TASK_QUERY_EN 1
#define OS_TASK_SUSPEND_EN 1
#define OS_TASK_SW_HOOK_EN 1
#define OS_TIME_TICK_HOOK_EN 1
#define OS_TIME_GET_SET_EN 1
#define OS_TIME_DLY_RESUME_EN 1
#define OS_TIME_DLY_HMSM_EN 1
#define OS_TMR_EN 1
#define OS_TMR_CFG_MAX 16
#define OS_TMR_CFG_NAME_SIZE 16
#define OS_TMR_CFG_TICKS_PER_SEC 10
#define OS_TMR_CFG_WHEEL_SIZE 2
#define OS_TASK_TMR_STK_SIZE 512
#define OS_TASK_TMR_PRIO 1
#define ALT_SYS_CLK TIMER_0
#define ALT_TIMESTAMP_CLK none
#define OS_TICKS_PER_SEC 1000

/*
 * Devices associated with code sections.
 *
 */

#define ALT_TEXT_DEVICE       SDRAM
#define ALT_RODATA_DEVICE     SDRAM
#define ALT_RWDATA_DEVICE     SDRAM
#define ALT_EXCEPTIONS_DEVICE SDRAM
#define ALT_RESET_DEVICE      SDRAM

/*
 * The text section is initialised so no bootloader will be required.
 * Set a variable to tell crt0.S to provide code at the reset address and
 * to initialise rwdata if appropriate.
 */

#define ALT_NO_BOOTLOADER


#endif /* __SYSTEM_H_ */
