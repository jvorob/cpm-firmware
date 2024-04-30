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
    //uint32_t ui32GPIONumber;
    for (uint32_t ux = 0; ux < AM_BSP_NUM_LEDS; ux++) {
        //ui32GPIONumber = am_bsp_psLEDs[ux].ui32GPIONumber;
        (led_state) ? am_devices_led_on(am_bsp_psLEDs, ux) :
                      am_devices_led_off(am_bsp_psLEDs, ux);
    }
#endif // AM_BSP_NUM_LEDS
}


//*****************************************************************************
//
//      HAL ERROR HANDLING
//
//*****************************************************************************

volatile uint32_t ui32LastError;
void error_handler(uint32_t ui32ErrorStatus) {
    ui32LastError = ui32ErrorStatus;
    while (1);
}
#define CHECK_ERRORS(x) \
    if ((x) != AM_HAL_STATUS_SUCCESS) {error_handler(x);}

//*********************************************
//
//             UART configuration.
//
//*********************************************

uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

const am_hal_uart_config_t g_sUartConfig =
{
    // Standard UART settings: 115200-8-N-1
    .ui32BaudRate = 115200,
    .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity      = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits    = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    // Set TX and RX FIFOs to interrupt at half-full.
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    // Buffers
    .pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = g_pui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

// uart handle?
void *phUART;


//*********************************************
//
//             UART funcs
//
//*********************************************

void uart_print(char *pcStr) {
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    // Measure the length of the string.
    while (pcStr[ui32StrLen] != 0) { ui32StrLen++; }

    // Print the string via the UART.
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen) {
        // Couldn't send the whole string!!
        while(1); //ERROR
    }
}

// Should override default am_uart_isr, defined in bsp I think?
void am_uart_isr(void) {
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
}


void uart_init() {

    // initialize the uart
    CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    // Set the uart pins to their configs (specified in sfe/.../am_bsp_pins.h)
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    // Enable interrupts? (AM_BSP_UART_PRINT_INST is 0 I think?)
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    am_hal_interrupt_master_enable();

    // Set the main print interface to use the UART print function we defined.
    am_util_stdio_printf_init(uart_print);
}


//*********************************************
//
//                    main
//
//*********************************************

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
    

    // === Setup UART, send hello
    uart_init();

    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World! (Over UART!)\n\n");
    am_hal_uart_tx_flush(phUART);


    init_morse_table();

    bool led_state = false;
    set_leds(led_state);

    // === Setup Morse
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

        int last_offset = state.char_offset;
        struct morse_output result = morseAdvance(&state);
        if((int)state.char_offset != last_offset) {
            am_util_stdio_printf("%c", state.p_str[last_offset]);
        }



    
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
