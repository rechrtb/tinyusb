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

static bool ready = false;

static const uint16_t psize_2_size[] = {8, 16, 32, 64, 128, 256, 512, 1024};

#define EP_GET_FIFO_PTR(ep, scale) (((TU_XSTRCAT(TU_STRCAT(uint, scale),_t) (*)[0x8000 / ((scale) / 8)])FIFO_RAM_ADDR)[(ep)])


typedef struct 
{
	uint8_t len;
	bool in;

	uint8_t address;
	uint8_t endpoint;
} xfer_ctl_t;

static xfer_ctl_t pipes[EP_MAX];

void breakpoint(void)
{
	volatile int a = 0;
	a++;
}

static uint8_t hw_pipe_get_endpoint(uint8_t rhport, uint8_t pipe)
{
	(void) rhport;
	return USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PEPNUM;
}

static uint8_t hw_pipe_get_address(uint8_t rhport, uint8_t pipe)
{
	(void) rhport;
	uint8_t index = pipe >> 2;
	uint8_t pos = (pipe & 0x3) << 3;
	uint32_t reg = (&USB_REG->HSTADDR1)[index];
	return (reg & (0x7f << pos)) >> pos;
}

static void hw_set_token(uint8_t rhport, uint8_t pipe, uint8_t token)
{
	(void) rhport;
	uint32_t tmp = USB_REG->HSTPIPCFG[pipe];
	tmp &= ~HSTPIPCFG_PTOKEN;
	tmp |= HSTPIPCFG_PTOKEN & ((uint32_t)(token) << HSTPIPCFG_PTOKEN_Pos);
	USB_REG->HSTPIPCFG[pipe] = tmp;
}

static uint8_t hw_pipe_find(uint8_t rhport, uint8_t address, uint8_t endpoint)
{
	(void) rhport;
	for (uint8_t i = 1; i < EP_MAX; i++)
	{
		if (hw_pipe_get_address(rhport, i) == address &&
			hw_pipe_get_endpoint(rhport, i) == endpoint)
		{
			return i;
		}
	}
	return EP_MAX;
}

static uint8_t hw_pipe_find_free(uint8_t rhport)
{
	(void) rhport;
	for (uint8_t i = 1; i < EP_MAX; i++)
	{
		if (!hw_pipe_get_address(rhport, i))
		{
			return i;
		}
	}
	return EP_MAX;
}

static void hw_pipe_reset(uint8_t rhport, uint8_t pipe)
{
	(void) rhport;
	memset(&pipes[pipe], 0, sizeof(pipes[pipe]));

	USB_REG->HSTPIP |= ((1 << pipe) << HSTPIP_PRST_Pos) & HSTPIP_PRST; // put pipe in reset
	USB_REG->HSTPIP &= ~(((1 << pipe) << HSTPIP_PRST_Pos) & HSTPIP_PRST); // remove pipe from reset
}

static void hw_pipes_reset(uint8_t rhport)
{
	(void) rhport;
	for (uint8_t i = 0; i < EP_MAX; i++)
	{
		hw_pipe_reset(rhport, i);
	}
}

static uint16_t compute_psize(uint16_t size)
{
	uint8_t i;
	for (i = 0; i < sizeof(psize_2_size) / sizeof(uint16_t); i++) {
		/* Size should be exactly PSIZE values */
		if (size <= psize_2_size[i]) {
			return i;
		}
	}
	return 7;
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
	hw_pipes_reset(rhport);

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

	ready = false;

	return true;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
	uint8_t pipe = 0;

	// if (dev_addr)
	// {
	// 	pipe = hw_pipe_find(dev_addr, 0xFF);
	// 	if (pipe >= EP_MAX)
	// 	{
	// 		return false;
	// 	}
	// }

	USB_REG->HSTPIP |= ((1 << pipe) << HSTPIP_PRST_Pos) & HSTPIP_PRST; // put pipe in reset
	USB_REG->HSTPIP &= ~(((1 << pipe) << HSTPIP_PRST_Pos) & HSTPIP_PRST); // remove pipe from reset
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
	uint8_t pipe = 0;

	if (dev_addr)
	{
		pipe = hw_pipe_find(rhport, dev_addr, 0);
		if (pipe >= EP_MAX)
		{
			return false;
		}
	}

	// hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_SETUP_Val);
	hw_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_SETUP_Val);
	// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXSTPI);
	USB_REG->HSTPIPICR[pipe] = HSTPIPICR_CTRL_TXSTPIC;
	// for (i = 0; i < 8; i++) {
	// 	*dst8++ = *src8++;
	// }
	const uint8_t *src = setup_packet;
	uint8_t *dst = EP_GET_FIFO_PTR(pipe, 8);
	for (size_t i = 0; i < 8; i++)
	{
		*dst++ = *src++;
	}
	// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_TXSTPE);
	USB_REG->HSTPIPIER[pipe] = HSTPIPIER_CTRL_TXSTPES;
	// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_FIFOCON | USBHS_HSTPIPIMR_PFREEZE);
	USB_REG->HSTPIPIDR[pipe] = HSTPIPIMR_FIFOCON | HSTPIPIMR_PFREEZE;

	pipes[pipe].len = (setup_packet[7] << 8) | setup_packet[6];
	pipes[pipe].in = (setup_packet[0] & TUSB_DIR_IN_MASK);

	return true;
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
	(void) rhport;
}

bool hcd_port_connect_status(uint8_t rhport)
{
	return ready;
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
	return (USB_REG->HSTFNUM & HSTFNUM_FNUM) >> HSTFNUM_FNUM_Pos;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
	// Reset the pipe
	uint8_t pipe = 0; // TODO: find available pipe

	if (dev_addr)
	{
		pipe = hw_pipe_find_free(rhport);
		if (pipe >= EP_MAX)
		{
			return false;
		}
	}

	hw_pipe_reset(rhport, pipe);

	// Configure the pipe
	USB_REG->HSTPIP |= ((1 << pipe) << HSTPIP_PEN_Pos) & HSTPIP_PEN; // enable pipe

	uint32_t cfg = 0;

	tusb_xfer_type_t type = ep_desc->bmAttributes.xfer;
	cfg |= (HSTPIPCFG_PTYPE & ((uint32_t)type << HSTPIPCFG_PTYPE_Pos));

	tusb_dir_t dir  = ep_desc->bEndpointAddress & TUSB_DIR_IN_MASK;
	cfg |= (type == TUSB_XFER_CONTROL ? 0 :
					(dir == TUSB_DIR_OUT ? USBHS_HSTPIPCFG_PTOKEN_OUT : USBHS_HSTPIPCFG_PTOKEN_IN));

	uint16_t interval = ep_desc->bInterval;
	tusb_speed_t speed = hcd_port_speed_get(rhport);
	if (speed == TUSB_SPEED_HIGH && (type == TUSB_XFER_ISOCHRONOUS || type == TUSB_XFER_INTERRUPT))
	{
		uint16_t ms = interval > 16 ? 16 : 2 << (interval - 1);
		interval = (ms > 0xFF) ? 0xFF : (uint8_t)ms;
	} else
	{
		if (type == TUSB_XFER_BULK && dir == TUSB_DIR_OUT && interval < 1)
		{
			interval = 1;
		}
	}
	cfg |= (HSTPIPCFG_INTFRQ & ((uint32_t)interval << HSTPIPCFG_INTFRQ_Pos));

	bool ping = (speed == TUSB_SPEED_HIGH) && (type == TUSB_XFER_CONTROL || (type == TUSB_XFER_BULK && dir == TUSB_DIR_OUT));
	cfg |= (ping ? HSTPIPCFG_CTRL_BULK_PINGEN : 0);

	cfg |= (HSTPIPCFG_PEPNUM & ((uint32_t)(ep_desc->bEndpointAddress & 0xF) << HSTPIPCFG_PEPNUM_Pos));

	uint16_t size = ep_desc->wMaxPacketSize & 0x3FF;
	cfg |= (HSTPIPCFG_PSIZE & ((uint32_t)compute_psize(size) << HSTPIPCFG_PSIZE_Pos));

	uint8_t bank = ((size >> 11) & 0x3) + 1;
	cfg |= (HSTPIPCFG_PBK & ((uint32_t)(bank - 1) << HSTPIPCFG_PBK_Pos));

	cfg |= HSTPIPCFG_ALLOC;

	USB_REG->HSTPIP |= USBHS_HSTPIP_PEN0 << pipe;
	USB_REG->HSTPIPCFG[pipe] = cfg;

	if (USB_REG->HSTPIPISR[pipe] & HSTPIPISR_CFGOK) // check if config is correct
	{
		// Setup pipe address
		uint8_t reg_i = pipe >> 2;
		uint8_t pos = (pipe & 0x3) << 3;
		uint32_t reg = (&USB_REG->HSTADDR1)[reg_i];
		reg &= ~(0x7F << pos);
		reg |= (dev_addr & 0x7F) << pos;
		(&USB_REG->HSTADDR1)[reg_i] = reg;

		// Configure pipe-related interrupts
		USB_REG->HSTPIPICR[pipe] = HSTPIPICR_CTRL_RXSTALLDIC | HSTPIPICR_OVERFIC | HSTPIPISR_PERRI | HSTPIPICR_INTRPT_UNDERFIC;
		USB_REG->HSTPIPIER[pipe] = HSTPIPIER_CTRL_RXSTALLDES | HSTPIPIER_OVERFIES | HSTPIPIER_PERRES;
		USB_REG->HSTIER |= (HSTISR_PEP_0 | (HSTISR_DMA_0 >> 1)) << pipe;

		pipes[pipe].address = dev_addr;
		pipes[pipe].endpoint = ep_desc->bEndpointAddress;

		return true;
	}

	USB_REG->HSTPIP &= ~(((1 << pipe) << HSTPIP_PEN_Pos) & HSTPIP_PEN); // disable pipe
	return false;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
	uint8_t pipe = 0;

	if (dev_addr)
	{
		uint8_t ep_num = ep_addr & (TUSB_DIR_IN_MASK - 1);
		pipe = hw_pipe_find(rhport, dev_addr, ep_num == 0 ? ep_num : ep_addr);
		if (pipe >= EP_MAX)
		{
			return false;
		}
	}

	if (ep_addr & TUSB_DIR_IN_MASK)
	{
		// to_read++;
		// if (n_rx < max_pkt_size) {
		// 	shortpkt = true;
		// }
		// if (n_rx) {
		// 	_usb_h_load_x_param(pipe, &dst, &size, &count);
		// 	n_remain = size - count;
		// 	src      = (uint8_t *)&_usbhs_get_pep_fifo_access(pi, 8);
		// 	dst      = &dst[count];
		// 	if (n_rx >= n_remain) {
		// 		n_rx = n_remain;
		// 		full = true;
		// 	}
		// 	count += n_rx;
		// 	for (i = 0; i < n_rx; i++) {
		// 		*dst++ = *src++;
		// 	}
		// 	_usb_h_save_x_param(pipe, count);
		// }

		uint8_t *data =  EP_GET_FIFO_PTR(pipe, 8);

		memcpy(buffer, data, buflen);

		// /* Clear FIFO status */
		// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIDR_FIFOCONC);
		USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_FIFOCONC;

		// /* Reset timeout for control pipes */
		// if (pipe->type == 0) {
		// 	pipe->x.ctrl.pkt_timeout = USB_CTRL_DPKT_TIMEOUT;
		// }
		// /* Finish on error or short packet */
		// if (full || shortpkt) {
		// 	hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIDR_SHORTPACKETIEC | USBHS_HSTPIPIDR_RXINEC);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_SHORTPACKETIEC | HSTPIPIDR_RXINEC;
		// 	if (pipe->type == 0) { /* Control transfer: DatI -> StatO */
		// 		pipe->x.ctrl.state       = USB_H_PIPE_S_STATO;
		// 		pipe->x.ctrl.pkt_timeout = USB_CTRL_STAT_TIMEOUT;
		// 		_usb_h_out_zlp_ex(pipe);

		// hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_OUT_Val);
		uint32_t tmp = USB_REG->HSTPIPCFG[pipe];
		tmp &= ~HSTPIPCFG_PTOKEN;
		tmp |= HSTPIPCFG_PTOKEN & ((uint32_t)(HSTPIPCFG_PTOKEN_OUT_Val) << HSTPIPCFG_PTOKEN_Pos);
		USB_REG->HSTPIPCFG[pipe] = tmp;

		// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
		USB_REG->HSTPIPICR[pipe] = HSTPIPISR_TXOUTI;
		// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_TXOUTE);
		USB_REG->HSTPIPIER[pipe] = HSTPIPIER_TXOUTES;
		// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_FIFOCON | USBHS_HSTPIPIMR_PFREEZE);
		USB_REG->HSTPIPIDR[pipe] = HSTPIPIMR_FIFOCON | HSTPIPIMR_PFREEZE;

		// 	} else {
		// 		hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_PFREEZES);
		// 		hri_usbhs_write_HSTPIPINRQ_reg(drv->hw, pi, 0);
		// 		_usb_h_end_transfer(pipe, USB_H_OK);
		// 	}
		// } else if (!hri_usbhs_read_HSTPIPINRQ_reg(drv->hw, pi)
		// 		&& hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE)) {
		// 	/* Unfreeze if request packet by packet */
		// 	hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE);
		// } else {
		// 	/* Just wait another packet */
		// }
		return true;
	}
	else
	{
		breakpoint();
		return true;
	}
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
		USB_REG->CTRL &= ~CTRL_FRZCLK;
		while (!(USB_REG->SR & SR_CLKUSABLE));

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
		USB_REG->CTRL &= ~CTRL_FRZCLK;
		while (!(USB_REG->SR & SR_CLKUSABLE));

		USB_REG->HSTICR |= HSTICR_DCONNIC;
		USB_REG->HSTIDR |= HSTISR_DCONNI;

		// Enable disconnection interrupt
		USB_REG->HSTIER |= HSTIER_DDISCIES;

		// Enable SOF
		USB_REG->HSTCTRL |= HSTCTRL_SOFE;

		// Notify tinyUSB that device attached to initiate next states
		hcd_event_device_attach(rhport, true);
	}

	if (isr & HSTISR_RSTI)
	{
		USB_REG->CTRL &= ~CTRL_FRZCLK;
		while (!(USB_REG->SR & SR_CLKUSABLE));
		USB_REG->HSTICR |= HSTICR_RSTIC;
		USB_REG->HSTIDR |= HSTIDR_RSTIEC;

		ready = true;
	}

	if (isr & HSTISR_PEP_)
	{
		USB_REG->CTRL &= ~CTRL_FRZCLK;
		while (!(USB_REG->SR & SR_CLKUSABLE));
		uint8_t pipe = 23 - __CLZ(isr & USBHS_HSTISR_PEP__Msk);
		uint32_t pipisr = USB_REG->HSTPIPISR[pipe];

		if (pipisr & HSTPIPISR_CTRL_TXSTPI)
		{
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXSTPI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_CTRL_TXSTPIC;
			// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXSTPI);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPICR_CTRL_TXSTPIC;

			uint8_t *setup =  EP_GET_FIFO_PTR(pipe, 8);

			if (pipes[pipe].in || pipes[pipe].len == 0)
			{
				// _usb_h_ctrl_in_req(p);

				// hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_IN_Val);
				uint32_t tmp = USB_REG->HSTPIPCFG[pipe];
				tmp &= ~HSTPIPCFG_PTOKEN;
				tmp |= HSTPIPCFG_PTOKEN & ((uint32_t)(HSTPIPCFG_PTOKEN_IN_Val) << HSTPIPCFG_PTOKEN_Pos);
				USB_REG->HSTPIPCFG[pipe] = tmp;

				// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_RXINI | USBHS_HSTPIPISR_SHORTPACKETI);
				USB_REG->HSTPIPICR[pipe] = HSTPIPISR_RXINI | HSTPIPISR_SHORTPACKETI;
				// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_RXINES);
				USB_REG->HSTPIPIER[pipe] = HSTPIPIER_RXINES;
				// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_FIFOCON | USBHS_HSTPIPIMR_PFREEZE);
				USB_REG->HSTPIPIDR[pipe] = HSTPIPIMR_FIFOCON | HSTPIPIMR_PFREEZE;
			}
			else
			{
				//TODO 
				// 		/* Start OUT */
				// 		hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_OUT_Val);
				// 		hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_TXOUTE);
				// 		_usb_h_out(p);
			}
		}

		/* RXIN: Full packet received */
		if (pipisr & HSTPIPISR_RXINI)
		{
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_RXINI | USBHS_HSTPIPISR_SHORTPACKETI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_RXINIC | HSTPIPICR_SHORTPACKETIC;

			// /* In case of low USB speed and with a high CPU frequency,
			//  * a ACK from host can be always running on USB line
			//  * then wait end of ACK on IN pipe.
			//  */
			// if (!hri_usbhs_read_HSTPIPINRQ_reg(drv->hw, pi)) {
			// 	while (!hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE)) {
			// 	}
			// }
			if (!(USB_REG->HSTPIPINRQ[pipe]))
			{
				while(!(USB_REG->HSTPIPIMR[pipe] & HSTPIPIMR_PFREEZE)) {}
			}

			// _usb_h_in(p);
			// uint8_t            pi  = _usb_h_pipe_i(pipe);
			// struct usb_h_desc *drv = (struct usb_h_desc *)pipe->hcd;
			// uint8_t *          src, *dst;
			// uint32_t           size, count, i;
			// uint32_t           n_rx = 0;
			// uint32_t           n_remain;
			// uint16_t           max_pkt_size = pipe->type == 0 ? pipe->x.ctrl.pkt_size : pipe->max_pkt_size;
			// bool               shortpkt = false, full = false;

			if (pipes[pipe].len)
			{
				// /* Read byte count */
				// n_rx = hri_usbhs_read_HSTPIPISR_PBYCT_bf(drv->hw, pi);
				uint16_t to_read = (USB_REG->HSTPIPISR[pipe] & USBHS_HSTPIPISR_PBYCT_Msk) >> USBHS_HSTPIPISR_PBYCT_Pos;
				hcd_event_xfer_complete(hw_pipe_get_address(rhport, pipe), 0, to_read, XFER_RESULT_SUCCESS, true);
			}
			else
			{
				// 	/* Control status : ZLP IN done */
				// 	hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_PFREEZES);
				USB_REG->HSTPIPIER[pipe] = HSTPIPIER_PFREEZES;

				// 	hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIDR_SHORTPACKETIEC | USBHS_HSTPIPIDR_RXINEC);
				USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_SHORTPACKETIEC | HSTPIPIDR_RXINEC;

				// 	hri_usbhs_write_HSTPIPINRQ_reg(drv->hw, pi, 0);
				USB_REG->HSTPIPINRQ[pipe] = 0;

				hcd_event_xfer_complete(hw_pipe_get_address(rhport, pipe), 0, 0, XFER_RESULT_SUCCESS, true);
				hcd_event_xfer_complete(hw_pipe_get_address(rhport, pipe), 0, 0, XFER_RESULT_SUCCESS, true);

			// 	_usb_h_end_transfer(pipe, USB_H_OK);
			// 	return;
			}
		}

		if (pipisr & USBHS_HSTPIPISR_TXOUTI)
		{
			// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_PFREEZES);
			USB_REG->HSTPIPIER[pipe] =  HSTPIPIER_PFREEZES;

			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_TXOUTIC;

			// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPISR_TXOUTI;

			hcd_event_xfer_complete(hw_pipe_get_address(rhport, pipe), 0, 0, XFER_RESULT_SUCCESS, true);
			hcd_event_xfer_complete(hw_pipe_get_address(rhport, pipe), 0, 0, XFER_RESULT_SUCCESS, true);
			return;
		}
	}
}

#endif