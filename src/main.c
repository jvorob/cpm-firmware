// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Gabriel Marcano, 2023

/** This is an example main executable program */

#include <example.h>

#include <uart.h>
#include <syscalls.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include <sys/time.h>

#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

struct uart uart;

__attribute__((constructor))
static void redboard_init(void)
{
	// Prepare MCU by init-ing clock, cache, and power level operation
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	uart_init(&uart, UART_INST0);
	initialize_sys_uart(&uart);

	// After init is done, enable interrupts
	am_hal_interrupt_master_enable();
}

__attribute__((destructor))
static void redboard_shutdown(void)
{
	// Any destructors/code that should run when main returns should go here
}

int main(void)
{
	am_util_stdio_printf("Hello World!!!\r\n");
	printf("Hello World!!!\r\n");
	return 0;
}
