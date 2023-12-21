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

static void hcd_enable_sof_int(uint8_t rhport)
{
	// TODO: implement and fint where used
	USB_REG->HSTIER |= HSTIMR_HSOFIE;
}

static void hcd_handle_sof(uint8_t rhport)
{
	// TODO: check if one use of handling SOF interrupts
	// is to correctly implement hcd_frame_number on high speed, which seems to be
	// used for computing time measurement in milliseconds on non-OS builds
}

static void hcd_handle_pipe(uint8_t rhport)
{

}

static void hcd_handle_dma(uint8_t rhport)
{

}

static void hcd_handle_rh_change(uint8_t rhport, uint32_t isr)
{
	uint32_t imr = USB_REG->HSTIMR;

	if (isr & imr & HSTISR_DCONNI)
	{
		// Enable disconnection interrupt
		USB_REG->HSTIDR |= HSTISR_DCONNI;
		USB_REG->HSTICR |= HSTISR_DDISCI;
		USB_REG->HSTIER |= HSTISR_DDISCI;

		// /* Enable SOF */
		// USB_REG->CTRL |= HSTCTRL_SOFE;

		USB_REG->CTRL |= HSTCTRL_RESET;
	}




	if ((isr & HSTISR_HWUPI) && (imr & HSTISR_DCONNI))
	{
		// Check USB clock ready
		while (!(USB_REG->SR & SR_CLKUSABLE));

		// Unfreeze the clock
		USB_REG->CTRL &= ~CTRL_FRZCLK;

		// Disable HWUPI interrupt	
		USB_REG->HSTIDR |= HSTIDR_HWUPIEC;

		// Enable VBUS	
		USB_REG->SFR |= SFR_VBUSRQS;
	}

	if (isr & (HSTISR_HWUPI | HSTISR_RXRSMI | HSTISR_RSMEDI)) {
		// Check USB clock ready
		while (!(USB_REG->SR & SR_CLKUSABLE));

		// Unfreeze the clock
		USB_REG->CTRL &= ~CTRL_FRZCLK;

		USB_REG->HSTICR |= (HSTICR_HWUPIC | HSTICR_RSMEDIC | HSTICR_RXRSMIC);
		USB_REG->HSTIDR |= (HSTIDR_HWUPIEC | HSTIDR_RSMEDIEC | HSTIDR_RXRSMIEC);
		USB_REG->HSTIER |= (HSTIER_RSTIES | HSTIER_DDISCIES );

		// /* Enable SOF */
		// USB_REG->CTRL |= HSTCTRL_SOFE;

		/* Reset */
		USB_REG->DEVCTRL &= ~DEVCTRL_SPDCONF;
		// USB_REG->HSTIER |= (HSTIER_HSOFIES);
	}

	// 	if (!(isr & USBHS_HSTISR_RSMEDI) && !(isr & USBHS_HSTISR_DDISCI)) {
	// 		/* It is a upstream resume
	// 		 * Note: When the CPU exits from a deep sleep mode, the event
	// 		 * USBHS_HSTISR_RXRSMI can be not detected
	// 		 * because the USB clock are not available.
	// 		 *
	// 		 * In High speed mode a downstream resume must be sent
	// 		 * after a upstream to avoid a disconnection.
	// 		 */
	// 		if (hri_usbhs_get_SR_reg(drv->hw, USBHS_SR_SPEED_Msk) == USBHS_SR_SPEED_HIGH_SPEED) {
	// 			hri_usbhs_set_HSTCTRL_RESUME_bit(drv->hw);
	// 		}
	// 	}
	// 	/* Wait 50ms before restarting transfer */
	// 	_usb_h_set_resume(drv, pd, 50);
	// }
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
	USB_REG->DEVCTRL &= ~DEVCTRL_SPDCONF;

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
	USB_REG->CTRL &= ~CTRL_FRZCLK;
// 	while (!hri_usbhs_get_SR_reg(drv->hw, USBHS_SR_CLKUSABLE));
	while (!(USB_REG->SR & SR_CLKUSABLE));

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
	USB_REG->HSTIER |= (HSTIER_DCONNIES | HSTIER_RSTIES /*| HSTIER_HSOFIES */ |  HSTIER_HWUPIES);

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
	// breakpoint();
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
	breakpoint();
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
	breakpoint();
	return 0;
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

	// /* Low speed, switch to low power mode to use 48MHz clock */
	// // TODO: check handling transition from low speed -> high speed
	// volatile tusb_speed_t speed = hcd_port_speed_get(rhport);
	// if (speed == TUSB_SPEED_FULL || speed == TUSB_SPEED_LOW)
	// {
	// 	if (!(USB_REG->CTRL & USBHS_HSTCTRL_SPDCONF_Msk)) {
	// 		USB_REG->CTRL |= DEVCTRL_SPDCONF_LOW_POWER;
	// 	}
	// }

	/* SOF */
	// if (isr & HSTISR_HSOFI) {
	// 	hcd_handle_sof(rhport);
	// 	return;
	// }

	/* Pipe interrupts */
	if (isr & HSTIMR_PEP_) {
		hcd_handle_pipe(rhport);
		return;
	}

	// /* DMA interrupts */
	// TODO: handle dma as needed
	// if (isr & HSTISR_DMA_) {
	// 	hcd_handle_dma(rhport);
	// 	return;
	// }

	/* Reset sent, connect/disconnect, wake up */
	if (isr & (HSTISR_RSTI | HSTISR_DCONNI | HSTISR_DDISCI | HSTISR_HWUPI | HSTISR_RXRSMI | HSTISR_RXRSMI)) {
		hcd_handle_rh_change(rhport, isr);
		return;
	}
}

#endif