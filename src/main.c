// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Gabriel Marcano, 2023

/** This is an example main executable program */

//#include <example.h>

// ==== Let's try bringing in some asimple stuff
#include <uart.h>
#include <syscalls.h>
#include <spi.h>
#include <lora.h>

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


//===== PIN ASSIGNMENTS For: CPS board mk1 revA
#define JV_LORA_CS_CHAN 0 //GPIO 11, on IOM0
//#define JV_LORA_CS_CHAN 1 //TEMP DEBUG: GPIO 17, on IOM0

#define JV_PIN_LED 0 //GPIO 0

#define JV_PIN_LORA_EN 36
#define JV_PIN_LORA_RST 3
#define JV_PIN_LORA_DI0 38

#define JV_PIN_ADP_PGOOD 39
#define JV_PIN_ADP_DIS_SW 49

#define JV_PIN_ADC_VIN  16 // ADC0/TRIG0/CMPIN0
#define JV_PIN_ADC_VBAT 31 // ADC3
#define JV_PIN_ADC_DP0N 12 
#define JV_PIN_ADC_DP0P 13 
#define JV_PIN_ADC_DP1N 15  //yes 15&14 are swapped
#define JV_PIN_ADC_DP1P 14 

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
    (led_state) ? am_hal_gpio_state_write(JV_PIN_LED, AM_HAL_GPIO_OUTPUT_SET) :
                  am_hal_gpio_state_write(JV_PIN_LED, AM_HAL_GPIO_OUTPUT_CLEAR);
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
//                 SPI STUFF
//
//*********************************************
#define report(s) am_util_stdio_printf("status: 0x%08X (function: %s, file: %s, line: %d)\n", s, __func__, __FILE__, __LINE__)


// // for RFM95W
// // 7bit reg addr, high bit is W/NR
// void spi_write_lora_reg(int reg, int val) {
//     uint8_t cmd[4];
//     uint8_t rx_buf[4];
//     cmd[0] = (reg & 0x7F) | 0x80; //high bit 1 for write
//     cmd[1] = val & 0xFF;
// 
//     //NOTE: i think IOM0 CS0 might be pin 11? see bsps
//     xfer.uPeerInfo.ui32SpiChipSelect = JV_LORA_CS_CHAN;
//     xfer.ui32InstrLen = 0;
//     xfer.ui32Instr = 0;
//     xfer.ui32NumBytes = 2;
//     xfer.eDirection = AM_HAL_IOM_FULLDUPLEX;
//     xfer.pui32TxBuffer = (uint32_t*)cmd;
//     xfer.pui32RxBuffer = (uint32_t*)rx_buf;
//     xfer.bContinue = false;
//     xfer.ui8RepeatCount = 0;
//     xfer.ui8Priority = 1;
//     xfer.ui32PauseCondition = 0;
//     xfer.ui32StatusSetClr = 0;
// 
//     //am_util_stdio_printf("reading reg %d:\n", reg);
//     uint32_t status = AM_HAL_STATUS_SUCCESS;
//     status = am_hal_iom_spi_blocking_fullduplex(iom_handle, &xfer);
//     if(status != AM_HAL_STATUS_SUCCESS) { report(status);}
// 
//     //return rx_buf[1]; //the 0th byte was the addr, the 1st byte is resp
// }

uint8_t tmp_read_reg(struct spi_device *device, uint8_t addr){
    uint8_t buff_val;
    spi_device_cmd_read(device, addr, &buff_val, 1);
    return buff_val;
}

void tmp_write_reg(struct spi_device *device, uint8_t addr, uint8_t val){
    uint8_t buff_val = val;
    spi_device_cmd_write(device, addr | 0x80, &buff_val, 1);
}

//*********************************************
//
//              low-power helpers
//
//*********************************************


// Enable/disable the debugger UART?
// NOTE: copied mostly from bsp file, but changed to 1M baud
// (couldn't get 2M working)
void jv_itm_printf_enable() {
    //am_bsp_uart_printf_enable();
    //am_bsp_itm_printf_enable();

    // ======== TEMP MANUALLY ENABLE ITM PRINTF
    am_hal_tpiu_config_t TPIUcfg;

    // Enable the ITM interface and the SWO pin.
    am_hal_itm_enable();

    // Enable the ITM and TPIU
    // Set the BAUD clock for 1M
    //TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_DEFAULT;
    TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_1M;
    am_hal_tpiu_enable(&TPIUcfg);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_AM_BSP_GPIO_ITM_SWO);
    // Attach the ITM to the STDIO driver.
    am_util_stdio_printf_init(am_hal_itm_print);
    // ======== END MANUAL ITM
    
    ////Unused bonus funcs?
    //am_hal_tpiu_clock_enable();
}
void jv_itm_printf_disable() {
    am_hal_itm_disable();
    am_util_stdio_printf_init(0);

    ////Unused bonus funcs?
    //am_hal_tpiu_clock_disable();
}

//*********************************************
//
//          Test functions (inf loops)
//
//*********************************************

// Prints hello world and blinks
void testprog_helloblinky() {
    am_util_stdio_printf("Starting Blinky\n");

    bool led_state = 0;
    while(1){
        am_util_stdio_printf("Hello!\n");

        led_state = !led_state;
        set_leds(led_state);

        am_util_delay_ms(2000);

    }
}

// Initializes LORA module and sends a packet every few seconds
void testprog_lora() {
    am_util_stdio_printf("Starting LORA test program:\n");

    //*********************************************
    //           INITIALIZE Lora module
    //*********************************************
    
    // Asimple spi init
    struct spi_bus* spi_bus_0;
    struct spi_device* lora_spi; // handle to the spi device for the lora module
    struct lora lora_storage = {};
    struct lora* lora_obj = &lora_storage; // handle for the lora module, wraps the spi device
                                           
    //===== try resetting the Lora module (active low reset)
    //const int CONF_SPI_FREQ = 10000000; //10MHz
    const int CONF_SPI_FREQ = 10000; //10KHz
    const uint32_t CONF_LORA_FREQ = 902300000u;
    //LORA_FREQ = 915000000; //915MHZ? TODO: pick a channel?
                           //902.3MHZ? chan 0
                           
    const uint8_t CONF_LORA_SF = 7; // min 6, fast, noisy, max 12, slow, robust

    // 0b0111: 125KHz bandwidth, should be for channels 0-63 in US915? 
    const uint8_t CONF_LORA_BW = 7;

    //0b100: 4/8 coding (2x overhead?)
    const uint8_t CONF_LORA_CODERATE = 4;

    am_util_stdio_printf("RESETTING LORA MODULE:\n");
    am_hal_gpio_state_write(JV_PIN_LORA_RST, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_util_delay_ms(20);
    am_hal_gpio_state_write(JV_PIN_LORA_RST, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_ms(10);


    // Init LORA as spi device, 10MHz
    spi_bus_0 = spi_bus_get_instance(SPI_BUS_0);
    lora_spi = spi_device_get_instance(spi_bus_0, SPI_CS_0, CONF_SPI_FREQ);

    if(!spi_bus_enable(spi_bus_0)) { report(-1); }

    tmp_read_reg(lora_spi, 0x01);


    am_util_stdio_printf("Initializing asimple LORA\n");
    if(!lora_init(lora_obj, lora_spi, CONF_LORA_FREQ, JV_PIN_LORA_DI0)) { report(-1); }
    
    if(!lora_set_spreading_factor(lora_obj, CONF_LORA_SF)) { report(-1); } 
    lora_set_bandwidth(lora_obj, CONF_LORA_BW); 
    lora_set_coding_rate(lora_obj, CONF_LORA_CODERATE); 

    //*********************************************
    //           Initialization complete
    //*********************************************

    am_util_stdio_printf("READING INITIAL REGOPMODE\n");
    am_util_stdio_printf("lora reg 0x%x: 0x%x\n", 1, tmp_read_reg(lora_spi, 0x01));
    am_util_stdio_printf("lora version (@0x%x): 0x%x\n", 0x42, tmp_read_reg(lora_spi, 0x42));

    //am_util_stdio_printf("READING LORA REGS:\n");
    //for (int reg = 0; reg < 8; reg++) {
    //    int res = spi_read_lora_reg(reg);
    //    am_util_stdio_printf("lora reg 0x%x: 0x%x\n", reg, res);
    //}

    //am_util_stdio_printf("Trying burst read LORA REGS:\n");
    //spi_read_lora_nregs(0x41, 0x42); //read first 15 bytes? skipping addr 0

    //am_util_stdio_printf("Trying Wakeup\n");
    //tmp_write_reg(lora_spi, 0x1, 0x89); //RegOpMode, set to sleep
    //am_util_stdio_printf("lora reg 0x%x: 0x%x\n", 1, tmp_read_reg(lora_spi, 1));

    //am_util_stdio_printf("Enabling LORA MODE\n");
    //tmp_write_reg(lora_spi, 0x1, 0x08); //RegOpMode, set to sleep
    //am_util_stdio_printf("lora reg 0x%x: 0x%x\n", 1, tmp_read_reg(lora_spi, 1));
    //tmp_write_reg(lora_spi, 0x1, 0x88); //RegOpMode, enable lora
    //tmp_write_reg(lora_spi, 0x1, 0x89); //RegOpMode, wake to stdby mode
    //set to lora mode (bit 7)
    //set to Low freq mode (bit3 hi)
    //set to stdby mode (low bits 001)
    //
    am_util_stdio_printf("READING LORA REGS:\n");
    for (int reg = 0; reg < 8; reg++) {
        //int res = spi_read_lora_reg(reg);
        int res = tmp_read_reg(lora_spi, reg);
        am_util_stdio_printf("lora reg 0x%x: 0x%x\n", reg, res);
    }

    // ==== Trying to send a packet
    
    
    int num_packets = 0;
    int len = 0;
    int tx_bytes = 0;
    //am_util_stdio_printf("LORA tx_addr = 0x%x\n", lora_obj->tx_addr);

    while(1) {

        // Blink on
        set_leds(true);
        am_util_stdio_printf("Sending packet %d...\n", num_packets);

        // Fmt Packet
        unsigned char tx_buff[1024] = {};
        len = am_util_stdio_snprintf((char *)tx_buff, sizeof(tx_buff)-1, 
                "TX Test, packet #%d\n", num_packets);

        // Can send at most 0x80 bytes (possibly 255 if modify asimple?)
        if(len >= 0x80) { 
            am_util_stdio_printf("(Truncated packet from %d bytes to 127)\n", len); 
            len = 0x7F; 
        }

        // Transmit
        tx_bytes = (int)lora_send_packet(lora_obj, tx_buff, len);
        if (tx_bytes != len) {
            am_util_stdio_printf("Error: Only sent %d bytes, expected %d!\n", 
                    tx_bytes, len);
        } else {
            am_util_stdio_printf("Sent %d bytes!\n", tx_bytes);
        }

        num_packets++;

        set_leds(false);

        am_util_delay_ms(2000);
    }
}

// Blinks the LED transmitting a long morse code string
void testprog_morse() {
    am_util_stdio_printf("Starting Morse code program:\n");

    bool led_state = false;
    
    char *morse_str = " Test 1 2 3. the war of 1812. jabberwocky."
                      " we hold these truths to be self evident. "
                      " 4 score and 7 years ago."
    " to be or not to be, that is the question. whether tis nobler in the mind"
    " to suffer the slings and arrows of outrageous fortune, or to take arms"
    " against a sea of troubles and by opposing end them\0"
    ;


    init_morse_table();

    set_leds(led_state);

    // === Setup Morse

    // Initialize morse_state to be at first phase of first char of str
    struct morse_state state = {0};
    state.p_str = morse_str;

    while (1) {
        //am_hal_gpio_state_write(JV_CS_PIN, AM_HAL_GPIO_OUTPUT_TOGGLE);

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
}


struct tp_sleep_state {
    bool led_state;
} tp_sleep_state = {};

// v0: use a timer/IRQ to blink an LED
// v1: go to sleep and use a timer to wake up
void testprog_sleep() {
    am_util_stdio_printf("Starting sleep program:\n");

    am_util_stdio_printf("Test 1:\n");
    jv_itm_printf_disable();
    am_util_delay_ms(1000);
    //am_util_stdio_printf("Test 2(asleep):\n");
    jv_itm_printf_enable();
    am_util_stdio_printf("Test 3:\n");
    // Test Uart on/off
    


    // ==== SETUP RTC, ENABLE INTERRUPTS
    //  Start the RTC
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
    am_hal_rtc_osc_enable();
                                              
    // Configure alarm to interrupt every second
    am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_SEC);

    // Clear, then enable rtc interrupt
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
    am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);
    NVIC_EnableIRQ(RTC_IRQn);

    // enable interrupts corewide
    am_hal_interrupt_master_enable();

    am_util_stdio_printf("RTC INTEN = %0x\n", am_hal_rtc_int_enable_get());

    while(1){} //busy loop

}

void am_rtc_isr() { //overrides the main isr
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    am_util_stdio_printf("rtc ISR recieved\n");

    tp_sleep_state.led_state = !tp_sleep_state.led_state;
    set_leds(tp_sleep_state.led_state);
}



//*********************************************
//
//                    main
//
//*********************************************

int main(void)
{

    //*********************************************
    //        INITIALIZE CHIP-WIDE STUFF
    //*********************************************

	// Prepare MCU by init-ing clock, cache, and power level operation
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	//am_hal_sysctrl_fpu_enable();
	//am_hal_sysctrl_fpu_stacking_enable(true);
    
    //*********************************************
    //             INITIALIZE GPIOS
    //*********************************************

    // Random functions im collecting
    //am_hal_gpio_pinconfig(JV_CS_PIN,  g_AM_HAL_GPIO_OUTPUT);
    //am_hal_gpio_pinconfig(JV_CS_PIN,  g_AM_HAL_GPIO_OUTPUT);
    //am_hal_gpio_state_write(JV_CS_PIN, AM_HAL_GPIO_OUTPUT_SET);

	//// After init is done, enable interrupts
	//am_hal_interrupt_master_enable();
    
    // == Enable LED output, 2mA drive
    am_hal_gpio_pinconfig(JV_PIN_LED,  g_AM_HAL_GPIO_OUTPUT);

    //Enable Lora (LORA_EN high, nRESET high)
    am_hal_gpio_pinconfig(  JV_PIN_LORA_EN,   g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(JV_PIN_LORA_EN,   AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(JV_PIN_LORA_RST,    g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(JV_PIN_LORA_RST, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(JV_PIN_LORA_DI0,    g_AM_HAL_GPIO_INPUT);

    //Setup ADP Pins to do nothing (DIS_SW low)
    am_hal_gpio_pinconfig(  JV_PIN_ADP_PGOOD,  g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(  JV_PIN_ADP_DIS_SW, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(JV_PIN_ADP_DIS_SW, AM_HAL_GPIO_OUTPUT_CLEAR);

    // Setup ADC pins as inputs
    am_hal_gpio_pinconfig(JV_PIN_ADC_VIN,     g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_VBAT,    g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_VBAT,    g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_DP0N,    g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_DP0P,    g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_DP1N,    g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(JV_PIN_ADC_DP1P,    g_AM_HAL_GPIO_INPUT);

    //*********************************************
    //             INITIALIZE UART
    //*********************************************

    // === Setup UART, send hello
    jv_itm_printf_enable();
    am_util_stdio_printf("Initialization complete\n");

    //*********************************************
    //          END OF INITIALIZATION
    //*********************************************
    
    //am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World! (Over UART!!)\n=============\n");
    //am_hal_uart_tx_flush(phUART);
    
    // Go into one of our test program (these dont return!)

    //testprog_helloblinky();
    //testprog_lora();
    //testprog_morse();
    testprog_sleep();
}
