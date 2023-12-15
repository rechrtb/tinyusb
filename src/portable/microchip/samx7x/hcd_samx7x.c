/*
* The MIT License (MIT)
*
* Copyright (c) 2018, hathach (tinyusb.org)
* Copyright (c) 2021, HiFiPhile
* Copyright (c) 2023, rechrtb
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
* This file is part of the TinyUSB stack.
*/

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_SAMX7X

#include "sam.h"
#include "host/hcd.h"
#include "common_usb_regs.h"

bool hcd_init(uint8_t rhport)
{
  (void) rhport;
  hcd_int_disable(rhport);

  USB_REG->CTRL = CTRL_UIMOD | CTRL_USBE;
#if TUD_OPT_HIGH_SPEED
  USB_REG->DEVCTRL &= ~DEVCTRL_SPDCONF;
#else
  USB_REG->DEVCTRL |= DEVCTRL_SPDCONF_LOW_POWER;
#endif

  return false;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{

}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    return false;
}

void hcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ((IRQn_Type) ID_USBHS);
}

void hcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ((IRQn_Type) ID_USBHS);
}

void hcd_port_reset(uint8_t rhport)
{
}

void hcd_port_reset_end(uint8_t rhport)
{
}

bool hcd_port_connect_status(uint8_t rhport)
{
    return false;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
  switch (USB_REG->SR & SR_SPEED) {
  case SR_SPEED_FULL_SPEED:
  default:
    return TUSB_SPEED_FULL;
  case SR_SPEED_HIGH_SPEED:
    return TUSB_SPEED_HIGH;
  case SR_SPEED_LOW_SPEED:
    return TUSB_SPEED_LOW;
  }
}

uint32_t hcd_frame_number(uint8_t rhport)
{
    return 0;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
    return false;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
    return false;
}

void hcd_int_handler(uint8_t rhport)
{

}

#endif