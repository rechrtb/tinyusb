/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "sam.h"
#include "bsp/board.h"

#include "peripheral_clk_config.h"
#include "hpl/usart/hpl_usart_base.h"
#include "hpl/pmc/hpl_pmc.h"
#include "hal/include/hal_init.h"
#include "hal/include/hal_usart_async.h"
#include "hal/include/hal_gpio.h"


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define UsbPowerSwitchPin     GPIO(GPIO_PORTC, 6)
#define UsbModePin            GPIO(GPIO_PORTC, 20)
#define UsbDetectPin          GPIO(GPIO_PORTC, 19)


#define TriggerPin            GPIO(GPIO_PORTC, 22)

#define UART_TX_PIN           GPIO(GPIO_PORTB, 4)
#define UART_RX_PIN           GPIO(GPIO_PORTA, 21)

//------------- IMPLEMENTATION -------------//
void board_init(void)
{
  init_mcu();

  /* Disable Watchdog */
  hri_wdt_set_MR_WDDIS_bit(WDT);

  _pmc_enable_periph_clock(ID_PIOC);

  // UsbPowerSwitchPin
  gpio_set_pin_level(UsbPowerSwitchPin, false);
  gpio_set_pin_direction(UsbPowerSwitchPin, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(UsbPowerSwitchPin, GPIO_PIN_FUNCTION_OFF);

  // UsbModePin
  gpio_set_pin_level(UsbModePin, false);
  gpio_set_pin_direction(UsbModePin, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(UsbModePin, GPIO_PIN_FUNCTION_OFF);

  // UsbDetectPin
  gpio_set_pin_direction(UsbDetectPin, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(UsbDetectPin, GPIO_PULL_OFF);
  gpio_set_pin_function(UsbDetectPin, GPIO_PIN_FUNCTION_OFF);

  // TriggerPin
  gpio_set_pin_level(TriggerPin, false);
  gpio_set_pin_direction(TriggerPin, GPIO_DIRECTION_OUT);
  gpio_set_pin_function(TriggerPin, GPIO_PIN_FUNCTION_OFF);

#if CFG_TUSB_OS  == OPT_OS_NONE
  // 1ms tick timer (samd SystemCoreClock may not correct)
  SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

//// USB_HOST_INSTANCE_CLOCK_init
  hri_pmc_write_SCER_reg(PMC, PMC_SCER_USBCLK);
  _pmc_enable_periph_clock(ID_USBHS);
}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USBHS_Handler(void)
{
#if CFG_TUD_ENABLED
  tud_int_handler(0);
#endif

#if CFG_TUH_ENABLED
  tuh_int_handler(0);
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_trigger_pin(bool state)
{
  gpio_set_pin_level(TriggerPin, state);
}

void board_usb_set_host(bool host)
{
  gpio_set_pin_level(UsbModePin, host);
}

bool board_usb_detect(void)
{
  return gpio_get_pin_level(UsbDetectPin);
}

void board_usb_enable(bool state)
{
  gpio_set_pin_level(UsbPowerSwitchPin, state);
}

void board_led_write(bool state)
{
}

int board_uart_read(uint8_t* buf, int len)
{
  return len;
}

int board_uart_write(void const * buf, int len)
{
  return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}
