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
//#define JV_PIN_LED 5 //TEMP! For redboard ATP

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

void jv_CharPrint_noop(char *) {}
void jv_itm_printf_disable() {
    am_hal_itm_disable();
    am_hal_tpiu_disable();
    am_util_stdio_printf_init(jv_CharPrint_noop); //disables printf while allowing them to still go through?

    ////Unused bonus funcs?
    //am_hal_tpiu_clock_disable();
}



// prints clock status
// NOTE: must be called 2 seconds after updating RTC settings
void jvutil_print_clkgen_state() {
    am_hal_clkgen_status_t clk_stat = {};
    am_hal_clkgen_status_get(&clk_stat);

    am_util_stdio_printf("Current sysclk freq = %.dHz\n", clk_stat.ui32SysclkFreq);
    am_util_stdio_printf("RTC Oscillator:  %s\n",
            clk_stat.eRTCOSC == AM_HAL_CLKGEN_STATUS_RTCOSC_LFRC ? "LFRC":"XTAL");
    am_util_stdio_printf("bXtalFailure (or  LFRC is clkout): %s\n", clk_stat.bXtalFailure ? "true":"false");
}

// Sets all gpios disabled (except for keeping LORA low)
void jv_gpio_disable_most() {
    for(int i = 0; i < AM_HAL_GPIO_MAX_PADS; i++) {
        if (i == JV_PIN_LORA_EN) { continue; }
        am_hal_gpio_pinconfig(i, g_AM_HAL_GPIO_DISABLE);
    }
}

void jv_gpio_enable_led() {
    am_hal_gpio_pinconfig(JV_PIN_LED, g_AM_HAL_GPIO_OUTPUT);
}


void jv_dump_clock_regs() {
  am_util_stdio_printf("== Dumping Clock registers:\n");
  // Display which devices currently are powered
  am_util_stdio_printf("PWRCTRL->DEVPWREN: 0x%x: 0x%x\n", &(PWRCTRL->DEVPWREN), PWRCTRL->DEVPWREN);

  // This register enables power to various memories during runmode.
  // If a memory is not enabled in runnmode, it will not be powered in deep sleep either!
  am_util_stdio_printf("PWRCTRL->MEMPWREN: 0x%x: 0x%x\n",
          (uint32_t)&(PWRCTRL->MEMPWREN), (uint32_t)PWRCTRL->MEMPWREN);

  // For every memory that is powered in runmode, this register defines whether the specific
  // memory will remain powered during sleepmode.
  am_util_stdio_printf("PWRCTRL->MEMPWDINSLEEP: 0x%x: 0x%x\n",
          (uint32_t)&(PWRCTRL->MEMPWDINSLEEP), (uint32_t)PWRCTRL->MEMPWDINSLEEP);

  // Show the MISC register contents
  am_util_stdio_printf("PWRCTRL->MISC: 0x%x: 0x%x\n",
          (uint32_t)&(PWRCTRL->MISC), (uint32_t)PWRCTRL->MISC);

  am_util_stdio_printf("BLE Buck ON: %d\n", PWRCTRL->SUPPLYSTATUS_b.BLEBUCKON);

  // Note in Apollo3 manual: "The SIMO buck cannot be dynamically enabled/disabled after initial device reset."
  am_util_stdio_printf("SIMO Buck ON: %d\n", PWRCTRL->SUPPLYSTATUS_b.SIMOBUCKON);

  // The CLOCKENSTAT registers are important because they show you if there are any parts of the processor
  // that might be responsible for not letting the HFRC oscillator shut down.
  // Show the CLOCKENSTAT register contents
  am_util_stdio_printf("CLKGEN->CLOCKENSTAT: 0x%x: 0x%x\n",
          (uint32_t)&(CLKGEN->CLOCKENSTAT), (uint32_t)CLKGEN->CLOCKENSTAT);



  //DEBUG: try disabling tpiu to make sure that actually turns off the clock
  jv_itm_printf_disable();
  uint32_t tempval = (uint32_t)CLKGEN->CLOCKEN2STAT;
  jv_itm_printf_enable();

  // Show the CLOCKEN2STAT register contents
  am_util_stdio_printf("CLKGEN->CLOCKEN2STAT: 0x%x: 0x%x\n",
          //(uint32_t)&(CLKGEN->CLOCKEN2STAT), (uint32_t)CLKGEN->CLOCKEN2STAT);
          (uint32_t)&(CLKGEN->CLOCKEN2STAT), tempval);

  // Show the CLOCKEN3STAT register contents
  am_util_stdio_printf("CLKGEN->CLOCKEN3STAT: 0x%x: 0x%x\n",
          (uint32_t)&(CLKGEN->CLOCKEN3STAT), (uint32_t)CLKGEN->CLOCKEN3STAT);

}

// Will list off any peripherals
void jv_print_peripheral_pwr_status() {
    am_util_stdio_printf("=== Enabled Peripherals:\n");
    // List of peripherals in am_hal_pwrctrl.h
    // named like AM_HAL_PWRCTRL_PERIPH_{name}
    // Peripherals go from 0 (none) to AM_HAL_PWRCTRL_PERIPH_MAX-1
    char *periph_names[] = { "NONE", "IOS", "IOM0", "IOM1", "IOM2", "IOM3", "IOM4", "IOM5",
        "UART0", "UART1", "ADC", "SCARD", "MSPI", "PDM", "BLEL", "ERR:MAX" }; //MAX is out of bounds
    int num_enabled = 0;
    for (int i = 1; i < AM_HAL_PWRCTRL_PERIPH_MAX; i++) {
        uint32_t enabled = 0;
        int status = am_hal_pwrctrl_periph_enabled(i, &enabled);
        if(status != AM_HAL_STATUS_SUCCESS) {report(status);}
        if(enabled) {
            num_enabled++;
            am_util_stdio_printf("  %5s: %d\n", periph_names[i], enabled);
        }
        //am_util_stdio_printf("  %5s: %d\n", periph_names[i], enabled);
    }
    if(num_enabled == 0)
        { am_util_stdio_printf("- All peripherals are powered off\n"); }
}


bool jv_is_debugger_attached() {
    // See if a debugger is attached
    // Will still show up if debugger is removed, won't reset until power cycle
    uint32_t dhcsr = CoreDebug->DHCSR;
    bool debuggerAttached = dhcsr & CoreDebug_DHCSR_C_DEBUGEN_Msk;
    return debuggerAttached;
}


// Blinks LED N times, 50ms on, 50ms off
// BLOCKS
void jv_blink_n(int n) {
    if(n <= 0) { return; }

    set_leds(true);
    am_util_delay_ms(50);
    set_leds(false);

    // (Split into first blink/next blinks to avoid extra sleep at end)
    for(int i = 0; i < n-1; i++) { // this happens n-1 times
        am_util_delay_ms(50);
        set_leds(true);
        am_util_delay_ms(50);
        set_leds(false);
    }
}

// == Powers up LORA module using power switch IC, configures other pins
// JV_PIN_LORA_EN must be configured as output
// all other LORA pins get configured
// Doesn't affect SPI bus
// Returns true on success
bool jv_lora_poweron() {
    // am_hal_gpio_pinconfig(JV_PIN_LORA_RST,    g_AM_HAL_GPIO_OUTPUT);
    // am_hal_gpio_state_write(JV_PIN_LORA_RST, AM_HAL_GPIO_OUTPUT_SET);

    int status;
    status = am_hal_gpio_pinconfig(  JV_PIN_LORA_EN,   g_AM_HAL_GPIO_OUTPUT);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status); return false; }
    am_hal_gpio_state_write(JV_PIN_LORA_EN,   AM_HAL_GPIO_OUTPUT_CLEAR);

    // set DIO as input
    status = am_hal_gpio_pinconfig(JV_PIN_LORA_DI0,    g_AM_HAL_GPIO_INPUT);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status); return false; }

    // set LORA nRESET high
    status = am_hal_gpio_pinconfig(JV_PIN_LORA_RST,    g_AM_HAL_GPIO_OUTPUT);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status); return false; }
    am_hal_gpio_state_write(JV_PIN_LORA_RST,   AM_HAL_GPIO_OUTPUT_SET);

    // enable LORA power switch
    am_hal_gpio_state_write(JV_PIN_LORA_EN, AM_HAL_GPIO_OUTPUT_SET);

    // NOTE: this doesn't enable the SPI bus?

    //wait 10ms for chip to power up
    am_util_delay_ms(10);

    return true;
}

// == Powers down LORA module using power switch IC, disables other pins
// - JV_PIN_LORA_EN must be configured as output
// - all other LORA pins get disabled
// - Doesn't affect SPI buss
// returns true on success
bool jv_lora_poweroff() {
    int status;

    // disable LORA power switch
    am_hal_gpio_state_write(JV_PIN_LORA_EN, AM_HAL_GPIO_OUTPUT_CLEAR);

    // disable other LORA pins
    status = am_hal_gpio_pinconfig(JV_PIN_LORA_RST,    g_AM_HAL_GPIO_DISABLE);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status); return false; }
    status = am_hal_gpio_pinconfig(JV_PIN_LORA_DI0,    g_AM_HAL_GPIO_DISABLE);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status); return false; }

    // Note: this doesn't disable the SPI bus?
    // TODO

    return true;
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

//*********************************************
//
//              TESTPROG: LORA
//
//*********************************************
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

//*********************************************
//
//              TESTPROG: MORSE
//
//*********************************************

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

//*********************************************
//
//              TESTPROG: SLEEP
//
//*********************************************

struct tp_sleep_state {
    bool led_state;
} tp_sleep_state = {};

// v0: use a timer/IRQ to blink an LED
// v1: go to sleep and use a timer to wake up
void testprog_sleep() {
    am_util_stdio_printf("Starting sleep program:\n");


    // =====================================================
    // ==== SETUP RTC, ENABLE INTERRUPTS
    //  set RTC to use XTAL
    //am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
    //am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

    //OR: set RTC with LFRC??
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC, 0);
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



    // =====================================================
    // ====== CONFIGURE FLASH, SRAM, CACHE FOR DEEP SLEEP
    am_util_stdio_printf("Preparing to enter deep sleep\n");

    // Enable flash? only bottom 512K?
    if ( am_hal_pwrctrl_memory_enable(AM_HAL_PWRCTRL_MEM_FLASH_MIN) )
    { while(1); } //Example busy waits on error?


    // Note: the way these funcs work is AM_HAL_PWRCTRL_MEM_xxx masks off parts
    //       memory, and you can choose to deepsleep_powerdown()
    //       or deepsleep_retain() on those masks. You can use multiple call
    //       to e.g. retain all except the bottom 32k, since they just set
    //       or clear specific groups of bits in PWRCTRL->MEMPWDINSLEEP

    // FLASH: In deep sleep, disable all
    am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_MAX);

    // Cache: In deep sleep, disable
    am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);

    // SRAM: In deep sleep, retain all
    am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
    //// NOTE: BE CAREFUL WITH SRAM, need to do linker shenanigans if we want
    //// to ensure our RAM fits in bottom 32k
    //// Power down SRAM, retain only 32K
    //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
    //am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_32K_DTCM);


    // =====================================================
    // ==== Configure / disable peripherals


    // Setup SPI and LORA comms
    const int CONF_SPI_FREQ = 10000; //10KHz
    struct spi_bus* spi_bus_0;
    struct spi_device* lora_spi; // handle to the spi device for the lora module
    struct lora lora_storage = {};
    struct lora* lora_obj = &lora_storage; // handle for the lora module, wraps the spi device
    const uint32_t CONF_LORA_FREQ = 902300000u;
    const uint8_t CONF_LORA_SF = 7; // min 6, fast, noisy, max 12, slow, robust
    const uint8_t CONF_LORA_BW = 7;
    const uint8_t CONF_LORA_CODERATE = 4;

    spi_bus_0 = spi_bus_get_instance(SPI_BUS_0);
    lora_spi = spi_device_get_instance(spi_bus_0, SPI_CS_0, CONF_SPI_FREQ);

    // Enable spi bus, initialize LORA, configure settings
    jv_lora_poweron();

    if(!spi_bus_enable(spi_bus_0)) { report(-1); }
    if(!lora_init(lora_obj, lora_spi, CONF_LORA_FREQ, JV_PIN_LORA_DI0)) { report(-1); }
    if(!lora_set_spreading_factor(lora_obj, CONF_LORA_SF)) { report(-1); }
    lora_set_bandwidth(lora_obj, CONF_LORA_BW);
    lora_set_coding_rate(lora_obj, CONF_LORA_CODERATE);
    lora_sleep(lora_obj);
    spi_bus_sleep(spi_bus_0);

    // Send packet
    if(!spi_bus_enable(spi_bus_0)) { report(-1); }
    // Lora is configured, lora_send_packet should put it into standby
    uint8_t tx_buff[] = "Test packet!\n\0";
    int len = am_util_string_strlen((char *)tx_buff);
    int tx_bytes = (int)lora_send_packet(lora_obj, tx_buff, len);
    if (tx_bytes != len) { am_util_stdio_printf("Error: Only sent %d bytes, expected %d!\n", tx_bytes, len); }
    lora_sleep(lora_obj);
    spi_bus_sleep(spi_bus_0);

    //DEBUG: verify that enabling/disabling SPI bus actually works
    if(!spi_bus_enable(spi_bus_0)) { report(-1); }
    am_util_stdio_printf("enabled spi bus (for debugging)\n");
    spi_bus_sleep(spi_bus_0);
    am_util_stdio_printf("slept spi bus (for debugging)\n");


    // Print out clock source (need to sleep 2 seconds first)
    am_util_delay_ms(2000);
    jvutil_print_clkgen_state();


    jv_print_peripheral_pwr_status();

    // Disable LORA module
    am_util_stdio_printf("Disabling LORA\n");
    am_hal_gpio_state_write(JV_PIN_LORA_EN,   AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(JV_PIN_LORA_RST,  AM_HAL_GPIO_OUTPUT_CLEAR);

    jv_dump_clock_regs();

    // disable uart?
    am_util_stdio_printf("Deepsleep configuration complete\n");
    jv_itm_printf_disable(); //TODO: does this actually power down the interface?


    // ======= READY FOR SLEEP LOOP
    // setup to do different things each sleep, indicated by number of LED blinks
    int sleep_phase = 1;
    const int sleep_max_phase = 3; // phases go 1..max

    // Enter busy loop, sleep repeatedly
    // should wake for half a second, sleep for half a second
    while(1){
        //jv_itm_printf_enable();
        //am_util_stdio_printf("awoken\n");
        //jv_itm_printf_disable();

        jv_itm_printf_enable();

        jv_gpio_enable_led();

        // Blinks leds sleep_phase times
        jv_blink_n(sleep_phase);

        switch(sleep_phase) {
            case 1: {
                am_util_stdio_printf("Sleep phase 1: Disabling LORA, entering normal sleep\n");

                // disable LORA
                jv_lora_poweroff();

                //stay awake for 0.1s
                am_util_delay_ms(100);


            }
            break;
            case 2: {
                am_util_stdio_printf("Sleep phase 2: Enabling LORA stbdy, then LORA sleep, then deep sleeping\n");

                // enable LORA
                jv_lora_poweron();

                // Enable spi bus, initialize LORA, configure settings
                if(!spi_bus_enable(spi_bus_0)) { report(-1); }
                if(!lora_init(lora_obj, lora_spi, CONF_LORA_FREQ, JV_PIN_LORA_DI0)) { report(-1); }
                if(!lora_set_spreading_factor(lora_obj, CONF_LORA_SF)) { report(-1); }
                lora_set_bandwidth(lora_obj, CONF_LORA_BW);
                lora_set_coding_rate(lora_obj, CONF_LORA_CODERATE);

                // Put Lora to STNDBY for 200ms
                lora_standby(lora_obj);
                spi_bus_sleep(spi_bus_0);
                am_util_delay_ms(200);

                // Put Lora to sleep
                if(!spi_bus_enable(spi_bus_0)) { report(-1); }
                lora_sleep(lora_obj);
                spi_bus_sleep(spi_bus_0);

                // 200ms later, go to sleep
                am_util_delay_ms(200);

                // // Send packet
                // if(!spi_bus_enable(spi_bus_0)) { report(-1); }
                // // Lora is configured, lora_send_packet should put it into standby
                // char tx_buff[] = "Test packet!\n\0";
                // int len = am_util_string_strlen(tx_buff);
                // int tx_bytes = (int)lora_send_packet(lora_obj, tx_buff, len);
                // if (tx_bytes != len) { am_util_stdio_printf("Error: Only sent %d bytes, expected %d!\n", tx_bytes, len); }
                // lora_sleep();



            }
            break;

            case 3: {
                am_util_stdio_printf("Sleep phase 3: Sending LORA packet, then sleeping\n");

                // LORA should already be powered on / configured
                // lora_send_packet should put it into standby

                if(!spi_bus_enable(spi_bus_0)) { report(-1); }
                uint8_t tx_buff[] = "Test packet!\n\0";
                int len = am_util_string_strlen((char*)tx_buff);
                int tx_bytes = (int)lora_send_packet(lora_obj, tx_buff, len);
                if (tx_bytes != len) { am_util_stdio_printf("Error: Only sent %d bytes, expected %d!\n", tx_bytes, len); }
                lora_sleep(lora_obj);
                spi_bus_sleep(spi_bus_0);
            }

        }


        //jv_itm_printf_enable();
        //am_util_stdio_printf("going back to sleep\n");

        //Sleep debugging: wont deepsleep if TPIU enabled
        jv_itm_printf_disable();
        bool tpiu_was_disabled = MCUCTRL->TPIUCTRL_b.ENABLE == MCUCTRL_TPIUCTRL_ENABLE_DIS;
        if(!tpiu_was_disabled) {
            jv_itm_printf_enable();
            am_util_stdio_printf("Err: tpiu wasn't disabled properly, failed to deep sleep\n");
            jv_itm_printf_disable();
        }

        jv_itm_printf_enable();
        am_util_stdio_printf("Entering deep sleep\n");
        jv_itm_printf_disable();

        // Turn off all gpios (except keeping lora disabled)
        //jv_gpio_disable_most();

        //go to sleep


        if(sleep_phase == 1) {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        } else {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }

        sleep_phase++;
        if(sleep_phase > sleep_max_phase) { sleep_phase = 1;}
    } //busy loop

}

void am_rtc_isr() { //overrides the main isr
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    //am_util_stdio_printf("rtc ISR recieved\n");

    //tp_sleep_state.led_state = !tp_sleep_state.led_state;
    //set_leds(tp_sleep_state.led_state);
}


// Put LFRC 1kHz clock onto pin 0 (LED)
void testprog_lfrc_cal() {
    am_util_stdio_printf("Starting LFRC test program\n");
    am_util_stdio_printf("Switching pad 0 to clkout\n");

    // Configure pad0 as clkout
    const am_hal_gpio_pincfg_t gpio0_clkout =
    {
        .uFuncSel             = AM_HAL_PIN_0_CLKOUT,
        .eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eGPOutcfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    };
    int status;
    status = am_hal_gpio_pinconfig(0, gpio0_clkout);
    if(status != AM_HAL_STATUS_SUCCESS) { report(status);}

    // === Enable clkout from LFRC
    //status = am_hal_clkgen_clkout_enable(true, AM_HAL_CLKGEN_CLKOUT_LFRC_1024);
    //status = am_hal_clkgen_clkout_enable(true, AM_HAL_CLKGEN_CLKOUT_ULFRC_1); //1Hz

    // === OR Enable clkout from XTAL (need to enable XTAL manually)
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
    status = am_hal_clkgen_clkout_enable(true, AM_HAL_CLKGEN_CLKOUT_XTAL_1024);

    // See also: AM_HAL_CLKGEN_CLKOUT_ULFRC_1, ~1Hz, various other divisions, XTAL,

    if(status != AM_HAL_STATUS_SUCCESS) { report(status);}

    // === Print status and wrap up
    // takes 2 sec after selection of new RTC oscillator
    am_util_stdio_printf("Configured, waiting 2 sec...\n");

    am_hal_clkgen_status_t clk_stat = {};
    am_hal_clkgen_status_get(&clk_stat);

    am_util_stdio_printf("Current sysclk freq = %.dHz\n", clk_stat.ui32SysclkFreq);
    am_util_stdio_printf("RTC Oscillator:  %s\n",
            clk_stat.eRTCOSC == AM_HAL_CLKGEN_STATUS_RTCOSC_LFRC ? "LFRC":"XTAL");
    am_util_stdio_printf("bXtalFailure (or  LFRC is clkout): %s\n", clk_stat.bXtalFailure ? "true":"false");


    am_util_stdio_printf("Oscillating, entering busyloop\n");
    while(1){}
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

    //TEST! JUMP STRAIGHT TO SLEEP (DISABLE LORA)?
    //am_hal_gpio_pinconfig(JV_PIN_LED,  g_AM_HAL_GPIO_OUTPUT);
    //am_hal_gpio_pinconfig(  JV_PIN_LORA_EN,   g_AM_HAL_GPIO_OUTPUT);
    //am_hal_gpio_state_write(JV_PIN_LORA_EN,   AM_HAL_GPIO_OUTPUT_CLEAR);
    //testprog_sleep();

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

    // Start with LORA powered off
    // (other LORA pins get configure in jv_lora_poweron / poweroff
    am_hal_gpio_pinconfig( JV_PIN_LORA_EN,  g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(JV_PIN_LORA_EN,   AM_HAL_GPIO_OUTPUT_CLEAR);


    // //Setup ADP Pins to do nothing (DIS_SW low)
    // am_hal_gpio_pinconfig(  JV_PIN_ADP_PGOOD,  g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(  JV_PIN_ADP_DIS_SW, g_AM_HAL_GPIO_OUTPUT);
    // am_hal_gpio_state_write(JV_PIN_ADP_DIS_SW, AM_HAL_GPIO_OUTPUT_CLEAR);

    // // Setup ADC pins as inputs
    // am_hal_gpio_pinconfig(JV_PIN_ADC_VIN,     g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_VBAT,    g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_VBAT,    g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_DP0N,    g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_DP0P,    g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_DP1N,    g_AM_HAL_GPIO_INPUT);
    // am_hal_gpio_pinconfig(JV_PIN_ADC_DP1P,    g_AM_HAL_GPIO_INPUT);

    //*********************************************
    //             INITIALIZE UART
    //*********************************************

    // === Setup UART, send hello
    jv_itm_printf_enable();
    am_util_stdio_printf("\n\n\n==============\nInitialization complete\n");

    //*********************************************
    //          END OF INITIALIZATION
    //*********************************************


    if(jv_is_debugger_attached()) {
        am_util_stdio_printf("NOTE: Debugger detected, won't be able to deep sleep\n"
                          "(draws ~20-30mA when attached, and limits deep sleep to ~300uA\n"
                          "even after detached. Need to detach and power cycle\n");
        jv_blink_n(5);
    }

    am_util_id_t device_id;
    am_util_id_device(&device_id);
    am_util_stdio_printf("Device Info:\r\n"
            "\tPart number: 0x%08"PRIX32"\r\n"
            "\tChip ID0:    0x%08"PRIX32"\r\n"
            "\tChip ID1:    0x%08"PRIX32"\r\n"
            "\tRevision:    0x%08"PRIX32" (Rev%c%c)\r\n",
            device_id.sMcuCtrlDevice.ui32ChipPN,
            device_id.sMcuCtrlDevice.ui32ChipID0,
            device_id.sMcuCtrlDevice.ui32ChipID1,
            device_id.sMcuCtrlDevice.ui32ChipRev,
            device_id.ui8ChipRevMaj, device_id.ui8ChipRevMin );

    //am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World! (Over UART!!)\n==============\n");
    //am_hal_uart_tx_flush(phUART);

    // Go into one of our test program (these dont return!)

    //testprog_helloblinky();
    //testprog_lora();
    //testprog_morse();
    testprog_sleep();
    //testprog_lfrc_cal();
}
