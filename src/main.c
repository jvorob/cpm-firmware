// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Gabriel Marcano, 2023

/** This is an example main executable program */

//#include <example.h>

//#include <uart.h>
//#include <syscalls.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include <morse.h>

//#include <sys/time.h>

//#include <string.h>
//#include <assert.h>
//#include <math.h>
//#include <stdio.h>
//#include <time.h>

//struct uart uart;

#define BLINK_PERIOD 2000

#define JV_CS_PIN 28

// __attribute__((constructor))
// static void redboard_init(void)
// {
//
// 	//uart_init(&uart, UART_INST0);
// 	//syscalls_uart_init(&uart);
//
// }
//
// __attribute__((destructor))
// static void redboard_shutdown(void)
// {
// 	// Any destructors/code that should run when main returns should go here
// }


void set_leds(bool led_state) {
#ifdef AM_BSP_NUM_LEDS
    uint32_t ui32GPIONumber;
    for (uint32_t ux = 0; ux < AM_BSP_NUM_LEDS; ux++) {
        ui32GPIONumber = am_bsp_psLEDs[ux].ui32GPIONumber;
        (led_state) ? am_devices_led_on(am_bsp_psLEDs, ux) :
                      am_devices_led_off(am_bsp_psLEDs, ux);
    }
#endif // AM_BSP_NUM_LEDS
}




int main(void)
{

	// Prepare MCU by init-ing clock, cache, and power level operation
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	//am_hal_sysctrl_fpu_enable();
	//am_hal_sysctrl_fpu_stacking_enable(true);
    
    // ====== Init GPIO Pins (using CS (p28) as a gpio)
    // Random functions im collecting
    am_hal_gpio_pinconfig(JV_CS_PIN,  g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pinconfig(JV_CS_PIN,  g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(JV_CS_PIN, AM_HAL_GPIO_OUTPUT_SET);
    //AM_HAL_GPIO_OUTPUT_SET   
    //AM_HAL_GPIO_OUTPUT_CLEAR 
    //AM_HAL_GPIO_OUTPUT_TOGGLE


    // g_AM_HAL_GPIO_OUTPUT


	//// After init is done, enable interrupts
	//am_hal_interrupt_master_enable();

    init_morse_table();

    bool led_state = false;
    set_leds(led_state);

    char *morse_str = " Test 1 2 3. the war of 1812. jabberwocky."
                      " we hold these truths to be self evident. "
                      " 4 score and 7 years ago."
    " to be or not to be, that is the question. whether tis nobler in the mind"
    " to suffer the slings and arrows of outrageous fortune, or to take arms"
    " against a sea of troubles and by opposing end them"
    ;

    // Initialize morse_state to be at first phase of first char of str
    struct morse_state state = {0};
    state.p_str = morse_str;

    while (1) {
      am_hal_gpio_state_write(JV_CS_PIN, AM_HAL_GPIO_OUTPUT_TOGGLE);

      struct morse_output result = morseAdvance(&state);

  
      if(result.valid) {
        set_leds(result.out_high);
        am_util_delay_ms(result.duration_ms);

      } else { //end of string, reset
        state.char_offset = 0;
        am_util_delay_ms(BLINK_PERIOD);
      }
    }

    // Blink forever
    //while (1)
    //{
    //    // Toggle LEDs
    //    led_state = !led_state;
    //    set_leds(led_state);

    //    // Delay
    //    am_util_delay_ms(BLINK_PERIOD);


    //}
}
