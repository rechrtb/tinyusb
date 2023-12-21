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

#include "host/hcd.h"
#include "sam.h"
#include "common_usb_regs.h"

void breakpoint(void)
{
	volatile int a = 0;
	a++;
}

static void hcd_reset_pipes(uint8_t rhport)
{
	// TODO: implement
}


bool hcd_init(uint8_t rhport)
{
	(void) rhport;
	hcd_int_disable(rhport);

//// system_init -> USB_HOST_INSTANCE_init -> USB_HOST_INSTANCE_CLOCK_init
	// Note: already done in board_init()

//// system_init -> USB_HOST_INSTANCE_init -> usb_h_init ->  _usb_h_init
// 	struct _usb_h_prvt *pd = (struct _usb_h_prvt *)prvt;

// 	ASSERT(drv && hw && prvt && pd->pipe_pool && pd->pipe_pool_size);
// 	if (hri_usbhs_get_CTRL_reg(hw, USBHS_CTRL_USBE)) {
// 		return USB_H_DENIED;
// 	}

// #if CONF_USB_H_VBUS_CTRL
// 	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, false);
// #endif

// 	drv->sof_cb = (usb_h_cb_sof_t)_dummy_func_no_return;
// 	drv->rh_cb  = (usb_h_cb_roothub_t)_dummy_func_no_return;

// 	drv->prvt = prvt;
// 	drv->hw   = hw;

// 	_usb_h_reset_pipes(drv, false);
	hcd_reset_pipes(rhport);

// 	pd->suspend_start   = 0;
// 	pd->resume_start    = 0;
// 	pd->pipes_unfreeze  = 0;
// 	pd->n_ctrl_req_user = 0;
// 	pd->n_sof_user      = 0;
// 	pd->dpram_used      = 0;

// 	_usb_h_dev = drv;

// 	NVIC_EnableIRQ(USBHS_IRQn);
	// Note: done in tuh_init, after this call returns

// #define USBHS_CTRL_UIDE 0x01000000 /* UOTGID pin enable */
// 	hri_usbhs_clear_CTRL_reg(hw, USBHS_CTRL_UIMOD | USBHS_CTRL_UIDE);
	USB_REG->CTRL &= ~(CTRL_UIMOD | CTRL_UID);

// 	hri_usbhs_set_CTRL_reg(hw, USBHS_CTRL_FRZCLK);
	USB_REG->CTRL |= CTRL_FRZCLK;

// #if CONF_USBHS_SRC == CONF_SRC_USB_48M
// 	hri_usbhs_write_HSTCTRL_SPDCONF_bf(hw, 1);
// #else
// 	hri_usbhs_clear_HSTCTRL_reg(hw, USBHS_HSTCTRL_SPDCONF_Msk);
// #endif
	// TODO: detect if set to use high speed
	USB_REG->HSTCTRL &= ~HSTCTRL_SPDCONF;

// 	/* Force re-connection on initialization */
// 	hri_usbhs_write_HSTIFR_reg(drv->hw, USBHS_HSTIMR_DDISCIE | USBHS_HSTIMR_HWUPIE);
	// TODO: check if this is really correct
	USB_REG->HSTIFR |= HSTIMR_DDISCIE | HSTIMR_HWUPIE;

// 	return USB_H_OK;

//// usbhc_start -> usb_h_enable -> _usb_h_enable
// 	ASSERT(drv && drv->hw);
// 	hri_usbhs_set_CTRL_reg(drv->hw, USBHS_CTRL_USBE);
	USB_REG->CTRL = CTRL_USBE;
// 	/* Check USB clock */
// 	hri_usbhs_clear_CTRL_reg(drv->hw, USBHS_CTRL_FRZCLK);
	// TODO: Check if this can be unfreezed on interrupt handling like the device implementation.
	// USB_REG->CTRL &= ~CTRL_FRZCLK;
// 	while (!hri_usbhs_get_SR_reg(drv->hw, USBHS_SR_CLKUSABLE));
	// while (!(USB_REG->SR & SR_CLKUSABLE));

// 	/* Clear all interrupts that may have been set by a previous host mode */
// 	hri_usbhs_write_HSTICR_reg(drv->hw,
// 	                           USBHS_HSTICR_DCONNIC | USBHS_HSTICR_HSOFIC | USBHS_HSTICR_RSMEDIC | USBHS_HSTICR_RSTIC
// 	                               | USBHS_HSTICR_RXRSMIC);

	USB_REG->HSTICR |= (HSTICR_DCONNIC | HSTICR_HSOFIC | HSTICR_RSMEDIC | HSTICR_RSTIC | HSTICR_RXRSMIC);

// 	/* VBus Hardware Control! */
// 	hri_usbhs_set_CTRL_reg(drv->hw, 1u << 8);
	USB_REG->CTRL |= CTRL_VBUSHWC; // datasheet indicates must be set to 1
// #if CONF_USB_H_VBUS_CTRL
// 	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, true);
// #endif

// 	/* Enable interrupts to detect connection */
// 	hri_usbhs_set_HSTIMR_reg(drv->hw,
// 	                         USBHS_HSTIMR_DCONNIE | USBHS_HSTIMR_RSTIE | USBHS_HSTIMR_HSOFIE | USBHS_HSTIMR_HWUPIE);
	USB_REG->HSTIER |= HSTIER_HWUPIES;

	return true;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
//  --- _usb_h_disable ---
// 	ASSERT(drv && drv->hw);
// 	hri_usbhs_set_CTRL_reg(drv->hw, USBHS_CTRL_FRZCLK);
// 	hri_usbhs_clear_CTRL_reg(drv->hw, USBHS_CTRL_USBE);
// #if CONF_USB_H_VBUS_CTRL
// 	CONF_USB_H_VBUS_CTRL_FUNC(drv, 1, false);
// #endif
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
	breakpoint();
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
	// breakpoint();
	NVIC_DisableIRQ((IRQn_Type) ID_USBHS);
}

void hcd_port_reset(uint8_t rhport)
{
	// Enable reset sent interrupt
	USB_REG->HSTIER |= HSTIER_RSTIES;

	// Send reset
	USB_REG->HSTCTRL |= HSTCTRL_RESET;
}

void hcd_port_reset_end(uint8_t rhport)
{
	breakpoint();
}

bool hcd_port_connect_status(uint8_t rhport)
{
	breakpoint();
	return false;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
	breakpoint();
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
	return (USB_REG->HSTFNUM & HSTFNUM_FNUM) >> HSTFNUM_FNUM_Pos;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
	breakpoint();
	return false;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
	breakpoint();
	return false;
}


void hcd_int_handler(uint8_t rhport)
{
	volatile uint32_t isr = USB_REG->HSTISR;

	/* Low speed, switch to low power mode to use 48MHz clock */
	// TODO: check handling transition from low speed -> high speed
	volatile tusb_speed_t speed = hcd_port_speed_get(rhport);
	if (speed == TUSB_SPEED_FULL || speed == TUSB_SPEED_LOW)
	{
		if (!(USB_REG->HSTCTRL & USBHS_HSTCTRL_SPDCONF_Msk))
		{
			USB_REG->HSTCTRL|= HSTCTRL_SPDCONF_LOW_POWER;
		}
	}

	if (isr & HSTISR_HWUPI)
	{
		// USB_REG->CTRL &= ~CTRL_FRZCLK;
		// while (!(USB_REG->SR & SR_CLKUSABLE));

		// Disable HWUPI interrupt
		USB_REG->HSTIDR |= HSTIDR_HWUPIEC;

		// Enable VBUS
		USB_REG->SFR |= SFR_VBUSRQS;

		USB_REG->HSTICR |= HSTICR_HWUPIC;
		USB_REG->HSTIDR |= HSTIDR_HWUPIEC;

		// Enable connect interrupt
		USB_REG->HSTIER |= HSTIER_DCONNIES;
	}

	if (isr & HSTISR_DCONNI)
	{
		// USB_REG->CTRL &= ~CTRL_FRZCLK;
		// while (!(USB_REG->SR & SR_CLKUSABLE));

		USB_REG->HSTICR |= HSTICR_DCONNIC;
		USB_REG->HSTIDR |= HSTISR_DCONNI;

		// Enable disconnection interrupt
		USB_REG->HSTIER |= HSTIER_DDISCIES;

		// Enable SOF
		USB_REG->HSTCTRL |= HSTCTRL_SOFE;

		// Notify tinyUSB that device attached to initiate next states
		hcd_event_device_attach(rhport, true);
	}

}

#endif