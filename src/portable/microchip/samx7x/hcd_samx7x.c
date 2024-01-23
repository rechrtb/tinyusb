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

#define EP_GET_FIFO_PTR(ep, scale) (((TU_XSTRCAT(TU_STRCAT(uint, scale),_t) (*)[0x8000 / ((scale) / 8)])FIFO_RAM_ADDR)[(ep)])

typedef struct 
{
	uint8_t *buf;
	uint16_t buflen;
	uint16_t proclen;
	bool dma;
} hw_pipe_t;

static inline void hw_enter_critical(volatile uint32_t *atomic)
{
	*atomic = __get_PRIMASK();
	__disable_irq();
	__DMB();
}

static inline void hw_exit_critical(volatile uint32_t *atomic)
{
	__DMB();
	__set_PRIMASK(*atomic);
}


TU_ATTR_ALWAYS_INLINE static inline void CleanInValidateCache(uint32_t *addr, int32_t size)
{
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    SCB_CleanInvalidateDCache_by_Addr(addr, size);
  }
  else
  {
    __DSB();
    __ISB();
  }
}

static hw_pipe_t pipes[EP_MAX];

static const uint16_t psize_2_size[] = {8, 16, 32, 64, 128, 256, 512, 1024};

void breakpoint(void)
{
	volatile int a = 0;
	a++;
}

static uint32_t evts[1000];
static uint32_t evti = 0;

static inline void add_evt(uint32_t num)
{
	if (evti == 0)
	{
		memset(&evts, 0, sizeof(evts));
	}

	if (evti < 1000)
	{
		++evti;
		evts[evti - 1] = num;
	}
}

static inline uint8_t hw_pipe_get_endpoint(uint8_t rhport, uint8_t pipe)
{
	(void) rhport;
	uint8_t ep_num = ((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PEPNUM) >> HSTPIPCFG_PEPNUM_Pos);
	if (ep_num)
	{
		bool in = ((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PTOKEN) >> HSTPIPCFG_PTOKEN_Pos) == HSTPIPCFG_PTOKEN_IN_Val;
		return ep_num | (in ? TUSB_DIR_IN_MASK : 0x00);
	}
	return ep_num;
}

static inline uint8_t hw_pipe_get_address(uint8_t rhport, uint8_t pipe)
{
	(void) rhport;
	uint8_t index = pipe >> 2;
	uint8_t pos = (pipe & 0x3) << 3;
	uint32_t reg = (&USB_REG->HSTADDR1)[index];
	return ((reg & (0x7f << pos)) >> pos);
}

static inline void hw_pipe_set_token(uint8_t rhport, uint8_t pipe, uint32_t token)
{
	(void) rhport;
	uint32_t tmp = USB_REG->HSTPIPCFG[pipe];
	tmp &= ~HSTPIPCFG_PTOKEN;
	tmp |= HSTPIPCFG_PTOKEN & (token);
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

static void hw_pipe_abort(uint8_t rhport, uint8_t pipe)
{
	// hri_usbhs_set_HSTPIP_reg(drv->hw, USBHS_HSTPIP_PRST0 << pi);
	// hri_usbhs_clear_HSTPIP_reg(drv->hw, USBHS_HSTPIP_PRST0 << pi);

	USB_REG->HSTPIP |= HSTPIP_PRST0 << pipe;
	USB_REG->HSTPIP &= ~(HSTPIP_PRST0 << pipe);

	// /* Disable interrupts */
	// hri_usbhs_write_HSTPIPIDR_reg(drv->hw,
	// 							pi,
	// 							USBHS_HSTPIPIMR_RXINE | USBHS_HSTPIPIMR_TXOUTE | USBHS_HSTPIPIMR_TXSTPE
	// 								| USBHS_HSTPIPIMR_RXSTALLDE | USBHS_HSTPIPIMR_SHORTPACKETIE);

	USB_REG->HSTPIPIDR[pipe] = HSTPIPIMR_RXINE | HSTPIPIMR_TXOUTE | HSTPIPIMR_CTRL_TXSTPE | HSTPIPIMR_BLK_RXSTALLDE | HSTPIPIMR_SHORTPACKETIE;
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
	add_evt(12);
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
	USB_REG->HSTCTRL |= HSTCTRL_SPDCONF;
	// USB_REG->HSTCTRL &= ~HSTCTRL_SPDCONF;

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
	add_evt(11);
	// Reset every pipe associated with the device
	for (uint8_t i = 0; i < EP_MAX; i++)
	{
		if (hw_pipe_get_address(rhport, i) == dev_addr)
		{
			hw_pipe_reset(rhport, i);
			USB_REG->HSTPIP &= ~(((1 << i) << HSTPIP_PEN_Pos) & HSTPIP_PEN); // disable pipe
		}
	}
}

static void hcd_dma_handler(uint8_t rhport, uint8_t pipe_ix)
{
	uint32_t status = USB_REG->HSTDMA[pipe_ix - 1].HSTDMASTATUS;
	if (status & HSTDMASTATUS_CHANN_ENB)
	{
		return; // Ignore EOT_STA interrupt
	}
	// Disable DMA interrupt
	USB_REG->HSTIDR = HSTIDR_DMA_1 << (pipe_ix - 1);

	uint16_t count = pipes[pipe_ix].buflen - ((status & HSTDMASTATUS_BUFF_COUNT) >> HSTDMASTATUS_BUFF_COUNT_Pos);
    uint8_t ep_addr = hw_pipe_get_endpoint(rhport, pipe_ix);
    uint8_t dev_addr = hw_pipe_get_address(rhport, pipe_ix);
	hcd_event_xfer_complete(dev_addr, ep_addr, count, XFER_RESULT_SUCCESS, true);
}

static bool hw_pipe_setup_dma(uint8_t rhport, uint8_t pipe, bool end)
{
	uint32_t flags = 0;
    // _usb_h_dma(pipe, false);
    // uint8_t            pi   = _usb_h_pipe_i(pipe);
    // uint8_t            dmai = pi - 1;
    // struct usb_h_desc *drv  = (struct usb_h_desc *)pipe->hcd;
    // uint16_t           mps  = psize_2_size[hri_usbhs_read_HSTPIPCFG_PSIZE_bf(drv->hw, pi)];
    // hal_atomic_t       flags;
    // uint8_t *          buf;
    // uint32_t           size, count;
    // uint32_t           n_next, n_max;
    // uint32_t           dma_ctrl = 0;
    // bool               dir      = pipe->ep & 0x80;

    // if (pipe->x.general.state != USB_H_PIPE_S_DATO && pipe->x.general.state != USB_H_PIPE_S_DATI) {
    //     return; /* No DMA is running */
    // }


    // _usb_h_load_x_param(pipe, &buf, &size, &count);

    uint8_t endpoint = hw_pipe_get_endpoint(rhport, pipe);
    uint16_t max_size  = psize_2_size[(USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PSIZE) >> HSTPIPCFG_PSIZE_Pos];

	uint32_t nextlen = pipes[pipe].buflen - pipes[pipe].proclen;
	uint32_t maxlen = 0x10000;

	if (endpoint & TUSB_DIR_IN_MASK)
	{
		if (256 * max_size < maxlen)
		{
			maxlen = 256 * max_size;
		}
	}

	if (maxlen < nextlen)
	{
		nextlen = maxlen;
	}

    uint32_t dma_ctrl = USBHS_HSTDMACONTROL_BUFF_LENGTH((nextlen == 0x10000) ? 0 : nextlen);

    if (endpoint & TUSB_DIR_IN_MASK)
    {
        dma_ctrl |= HSTDMACONTROL_END_TR_IT | HSTDMACONTROL_END_TR_EN;
    }
    else
    {
        if ((pipes[pipe].buflen & (max_size - 1)) != 0)
        {
            dma_ctrl |= HSTDMACONTROL_END_B_EN;
        }
    }

	add_evt(10000);

    USB_REG->HSTDMA[pipe - 1].HSTDMAADDRESS = (uint32_t)&pipes[pipe].buf[pipes[pipe].proclen];
    dma_ctrl |= HSTDMACONTROL_END_BUFFIT | HSTDMACONTROL_CHANN_ENB;

	hw_enter_critical(&flags);
	add_evt(10001);
	if (!(USB_REG->HSTDMA[pipe - 1].HSTDMASTATUS & HSTDMASTATUS_END_TR_ST))
	{
		if (endpoint & TUSB_DIR_IN_MASK)
		{
			add_evt(10002);
			USB_REG->HSTPIPINRQ[pipe] = (nextlen + max_size - 1) / max_size - 1;
			add_evt(10003);
		}
		add_evt(10004);
		USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_NBUSYBKEC | HSTPIPIDR_PFREEZEC;
		add_evt(10005);
		pipes[pipe].proclen += nextlen;
		USB_REG->HSTDMA[pipe - 1].HSTDMACONTROL = dma_ctrl;
		add_evt(10006);
		hw_exit_critical(&flags);
		return true;
	}
	hw_exit_critical(&flags);

    // if (count < size && !end) {
    //     /* Need to send or receive other data */
    //     n_next = size - count;
    //     n_max  = USBHS_DMA_TRANS_MAX;
    //     if (dir) {
    //         /* 256 is the maximum of IN requests via UPINRQ */
    //         if (256L * mps < n_max) {
    //             n_max = 256L * mps;
    //         }
    //     }
    //     if (n_max < n_next) {
    //         /* HW maximum transfer size */
    //         n_next = n_max;
    //     }
    //     /* Set 0 to transfer the maximum */
    //     dma_ctrl = USBHS_HSTDMACONTROL_BUFF_LENGTH((n_next == USBHS_DMA_TRANS_MAX) ? 0 : n_next);
    //     if (dir) {
    //         /* Enable short packet reception */
    //         dma_ctrl |= USBHS_HSTDMACONTROL_END_TR_IT | USBHS_HSTDMACONTROL_END_TR_EN;
    //     } else if ((n_next & (mps - 1)) != 0) {
    //         /* Enable short packet option
    //         * else the DMA transfer is accepted
    //         * and interrupt DMA valid but nothing is sent.
    //         */
    //         dma_ctrl |= USBHS_HSTDMACONTROL_END_B_EN;
    //         /* No need to request another ZLP */
    //         pipe->zlp = 0;
    //     }
    //     /* Start USB DMA to fill or read FIFO of the selected endpoint */
    //     hri_usbhs_write_HSTDMAADDRESS_reg(drv->hw, dmai, (uint32_t)&buf[count]);
    //     dma_ctrl |= USBHS_HSTDMACONTROL_END_BUFFIT | USBHS_HSTDMACONTROL_CHANN_ENB;
    //     /* Disable IRQs to have a short sequence
    //     * between read of EOT_STA and DMA enable
    //     */
    //     atomic_enter_critical(&flags);
    //     if (!hri_usbhs_get_hstdmastatus_reg(drv->hw, dmai, usbhs_hstdmastatus_end_tr_st)) {
    //         if (dir) {
    //             hri_usbhs_write_HSTPIPINRQ_reg(drv->hw, pi, (n_next + mps - 1) / mps - 1);
    //         }
    //         if (pipe->periodic_start) {
    //             /* Still packets in FIFO, just start */
    //             if (hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_NBUSYBKE)) {
    //                 pipe->periodic_start = 0;
    //                 hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_NBUSYBKE);
    //             } else {
    //                 /* Wait SOF to start */
    //                 _usb_h_add_sof_user(drv); /* SOF User: periodic start */
    //             }
    //         } else {
    //             /* Just start */
    //             hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_NBUSYBKE | USBHS_HSTPIPIMR_PFREEZE);
    //         }
    //         /* Start DMA */
    //         hri_usbhs_write_HSTDMACONTROL_reg(drv->hw, dmai, dma_ctrl);
    //         count += n_next;
    //         _usb_h_save_x_param(pipe, count);
    //         atomic_leave_critical(&flags);
    //         return;
    //     }
    //     atomic_leave_critical(&flags);
    // }

    // /* OUT pipe */
    // if ((pipe->ep & 0x80) == 0) {
    //     if (pipe->zlp) {
    //         /* Need to send a ZLP (No possible with USB DMA)
    //         * enable interrupt to wait a free bank to sent ZLP
    //         */
    //         hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
    //         if (hri_usbhs_get_HSTPIPISR_reg(drv->hw, pi, USBHS_HSTPIPISR_RWALL)) {
    //             /* Force interrupt in case of pipe already free */
    //             hri_usbhs_write_HSTPIPIFR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
    //         }
    //         hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_TXOUTE);
    //     } else {
    //         /* Wait that all banks are free to freeze clock of OUT endpoint
    //         * and call callback except ISO. */
    //         /* For ISO out, start another DMA transfer since no ACK needed */
    //         hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_NBUSYBKES);
    //         if (pipe->type != 1) {
    //             /* Callback on BE transfer done */
    //             return;
    //         }
    //     }
    // }
    // /* Finish transfer */
    // _usb_h_end_transfer(pipe, USB_H_OK);
    // return USB_H_OK;
	add_evt(10010);

    return false;
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
	add_evt(6);

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
	hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_SETUP);
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
	USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_FIFOCONC | HSTPIPIDR_PFREEZEC;

	add_evt(100);

	return true;
}

void hcd_int_enable(uint8_t rhport)
{
	if (ready)
	{
		add_evt(4);
	}
	(void) rhport;
	NVIC_EnableIRQ((IRQn_Type) ID_USBHS);
}

void hcd_int_disable(uint8_t rhport)
{
	if (ready)
	{
		add_evt(5);
	}
	(void) rhport;
	NVIC_DisableIRQ((IRQn_Type) ID_USBHS);
}

void hcd_port_reset(uint8_t rhport)
{
	(void) rhport;
	// Enable reset sent interrupt
	USB_REG->HSTIER |= HSTIER_RSTIES;
	// Send reset
	USB_REG->HSTCTRL |= HSTCTRL_RESET;
}

void hcd_port_reset_end(uint8_t rhport)
{
	add_evt(7);
	(void) rhport;
}

bool hcd_port_connect_status(uint8_t rhport)
{
	add_evt(8);
	(void) rhport;
	return ready;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
	add_evt(9);
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
	// if (evti > 200)
	// {
	// 	add_evt(10);
	// }
	return (USB_REG->HSTFNUM & HSTFNUM_FNUM) >> HSTFNUM_FNUM_Pos;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
	add_evt(3);

	uint8_t pipe = 0;

	if (dev_addr)
	{
		pipe = hw_pipe_find_free(rhport);
		if (pipe >= EP_MAX)
		{
			return false;
		}
	}

	// Reset the pipe
	hw_pipe_reset(rhport, pipe);

	// Configure the pipe
	USB_REG->HSTPIP |= ((1 << pipe) << HSTPIP_PEN_Pos) & HSTPIP_PEN; // enable pipe

	uint32_t cfg = 0;

	tusb_xfer_type_t type = ep_desc->bmAttributes.xfer;
	cfg |= (HSTPIPCFG_PTYPE & ((uint32_t)type << HSTPIPCFG_PTYPE_Pos));

	tusb_dir_t dir  = ep_desc->bEndpointAddress & TUSB_DIR_IN_MASK;
	cfg |= (type == TUSB_XFER_CONTROL ? 0 :
					(dir == TUSB_DIR_OUT ? HSTPIPCFG_PTOKEN_OUT : HSTPIPCFG_PTOKEN_IN));

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

	bool use_dma = EP_DMA_SUPPORT(pipe) && type != TUSB_XFER_CONTROL;

	if (use_dma)
	{
		// /* Start DMA */
		// hri_usbhs_set_HSTPIPCFG_AUTOSW_bit(pipe->hcd->hw, pi);
		USB_REG->HSTPIPCFG[pipe] |= HSTPIPCFG_AUTOSW;
	}

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

		pipes[pipe].dma = use_dma;

		return true;
	}

	USB_REG->HSTPIP &= ~(((1 << pipe) << HSTPIP_PEN_Pos) & HSTPIP_PEN); // disable pipe
	return false;
}


bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
	add_evt(2);

	uint8_t pipe = 0;
	uint8_t ep_num = ep_addr & ~TUSB_DIR_IN_MASK;

	if (dev_addr)
	{
		pipe = hw_pipe_find(rhport, dev_addr, ep_num == 0 ? ep_num : ep_addr);
		if (pipe >= EP_MAX)
		{
			return false;
		}
	}

	pipes[pipe].buf = buffer;
	pipes[pipe].buflen = buflen;
	pipes[pipe].proclen = 0;

	if (pipes[pipe].dma)
	{
		CleanInValidateCache((uint32_t*) tu_align((uint32_t) buffer, 4), buflen + 31);
		// pipe->periodic_start = (!dir) && (iso_pipe || int_pipe);


        //_usb_h_dma(pipe, false);
        return hw_pipe_setup_dma(rhport, pipe, false);
	}
	else
	{
		if (ep_addr & TUSB_DIR_IN_MASK)
		{
			add_evt(200);

			// hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_IN_Val);
			hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_IN);
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_RXINI | USBHS_HSTPIPISR_SHORTPACKETI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_RXINIC | HSTPIPICR_SHORTPACKETIC;
			// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_RXINES);
			USB_REG->HSTPIPIER[pipe] = HSTPIPIER_RXINES;
			USB_REG->HSTPIPINRQ[pipe] = 0;
			USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_PFREEZEC;
			add_evt(201);
		}
		else
		{
			add_evt(300);

			volatile uint8_t *dst = EP_GET_FIFO_PTR(pipe, 8);
			volatile uint8_t *src = buffer;
			for (size_t i = 0; i < buflen; i++)
			{
				*dst++ = *src++;
			}
			// hri_usbhs_write_HSTPIPCFG_PTOKEN_bf(drv->hw, pi, USBHS_HSTPIPCFG_PTOKEN_OUT_Val);
			hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_OUT);
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPISR_TXOUTI;
			// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_TXOUTE);
			USB_REG->HSTPIPIER[pipe] = HSTPIPIER_TXOUTES;
			// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIMR_FIFOCON | USBHS_HSTPIPIMR_PFREEZE);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_FIFOCONC | HSTPIPIDR_PFREEZEC;
		}
	}

	return true;
}


void hcd_int_handler(uint8_t rhport)
{
	add_evt(1);

	volatile uint32_t isr = USB_REG->HSTISR;

	/* Low speed, switch to low power mode to use 48MHz clock */
	// TODO: check handling transition from low speed -> high speed
	if (hcd_port_speed_get(rhport) == TUSB_SPEED_LOW)
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

		return;
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

		return;
	}

	if (isr & HSTISR_RSTI)
	{
		USB_REG->CTRL &= ~CTRL_FRZCLK;
		while (!(USB_REG->SR & SR_CLKUSABLE));
		USB_REG->HSTICR |= HSTICR_RSTIC;
		USB_REG->HSTIDR |= HSTIDR_RSTIEC;
		ready = true;
		return;
	}

	if (isr & HSTISR_PEP_)
	{
		add_evt(5);

		uint8_t pipe = 23 - __CLZ(isr & USBHS_HSTISR_PEP__Msk);
		uint32_t pipisr = USB_REG->HSTPIPISR[pipe];

		uint8_t address = hw_pipe_get_address(rhport, pipe);
		uint8_t endpoint = hw_pipe_get_endpoint(rhport, pipe);

		if (pipisr & HSTPIPISR_CTRL_RXSTALLDI)
		{
			add_evt(88);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_CTRL_RXSTALLDIC;
			USB_REG->HSTPIPIER[pipe] = HSTPIPIER_RSTDTS;
			hw_pipe_abort(rhport, pipe);
			hcd_event_xfer_complete(address, endpoint, 0, XFER_RESULT_STALLED, true);
			return;
		}

		if (pipisr & HSTPIPISR_PERRI)
		{
			add_evt(89);
			xfer_result_t res = (USB_REG->HSTPIPERR[pipe] & HSTPIPERR_TIMEOUT) ? XFER_RESULT_TIMEOUT : XFER_RESULT_FAILED;
			hw_pipe_abort(rhport, pipe);
			hcd_event_xfer_complete(address, endpoint, 0, res, true);
			return;
		}

		if (pipisr & HSTPIPISR_CTRL_TXSTPI)
		{
			add_evt(10);
			USB_REG->HSTPIPIER[pipe] = HSTPIPIER_PFREEZES;
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXSTPI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_CTRL_TXSTPIC;
			// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXSTPI);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_CTRL_TXSTPEC;
			hcd_event_xfer_complete(address, endpoint, 8, XFER_RESULT_SUCCESS, true);
			add_evt(11);
			return;
		}

		/* RXIN: Full packet received */
		if (pipisr & HSTPIPISR_RXINI || pipisr & HSTPIPISR_SHORTPACKETI)
		{
			USB_REG->HSTPIPIER[pipe] = HSTPIPIER_PFREEZES;
			add_evt(20);

			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_RXINI | USBHS_HSTPIPISR_SHORTPACKETI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_RXINIC | HSTPIPICR_SHORTPACKETIC;

			/* In case of low USB speed and with a high CPU frequency,
			 * a ACK from host can be always running on USB line
			 * then wait end of ACK on IN pipe.
			 */
			// if (!hri_usbhs_read_HSTPIPINRQ_reg(drv->hw, pi)) {
			// 	while (!hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE)) {
			// 	}
			// }
			if (!(USB_REG->HSTPIPINRQ[pipe]))
			{
				while(!(USB_REG->HSTPIPIMR[pipe] & HSTPIPIMR_PFREEZE)) {}
			}

			// _usb_h_in(p);
			if (pipes[pipe].buflen)
			{
				add_evt(21);
				// /* Read byte count */
				// n_rx = hri_usbhs_read_HSTPIPISR_PBYCT_bf(drv->hw, pi);
				uint16_t rx = (USB_REG->HSTPIPISR[pipe] & USBHS_HSTPIPISR_PBYCT_Msk) >> USBHS_HSTPIPISR_PBYCT_Pos;
				volatile uint8_t *src = EP_GET_FIFO_PTR(pipe, 8);
				volatile uint8_t *dst = pipes[pipe].buf + pipes[pipe].proclen;
				for (size_t i = 0; i < rx; i++)
				{
					*dst++ = *src++;
				}

				USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_FIFOCONC;

				pipes[pipe].proclen += rx;

				if (pipes[pipe].proclen >= pipes[pipe].buflen)
				{
					add_evt(2300);
					hcd_event_xfer_complete(address, endpoint, pipes[pipe].proclen, XFER_RESULT_SUCCESS, true);
					add_evt(2301);
				}
				else
				{
					USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_PFREEZEC;
				}
			}
			else
			{
				// Zero-length packet
				add_evt(25);
				// // 	hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_PFREEZES);
				// USB_REG->HSTPIPIER[pipe] = HSTPIPIER_PFREEZES;
				// // 	hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPIDR_SHORTPACKETIEC | USBHS_HSTPIPIDR_RXINEC);
				// USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_SHORTPACKETIEC | HSTPIPIDR_RXINEC;
				// // 	hri_usbhs_write_HSTPIPINRQ_reg(drv->hw, pi, 0);
				// USB_REG->HSTPIPINRQ[pipe] = 0;
				USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_FIFOCONC;
				hcd_event_xfer_complete(address, endpoint, 0, XFER_RESULT_SUCCESS, true);
				add_evt(26);
			}
			return;
		}

		if (pipisr & HSTPIPISR_TXOUTI)
		{
			add_evt(30);
			// hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIER_PFREEZES);
			USB_REG->HSTPIPIER[pipe] =  HSTPIPIER_PFREEZES;
			// hri_usbhs_write_HSTPIPICR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
			USB_REG->HSTPIPICR[pipe] = HSTPIPICR_TXOUTIC;
			// hri_usbhs_write_HSTPIPIDR_reg(drv->hw, pi, USBHS_HSTPIPISR_TXOUTI);
			USB_REG->HSTPIPIDR[pipe] = HSTPIPIDR_TXOUTEC;

			hcd_event_xfer_complete(address, endpoint, pipes[pipe].buflen, XFER_RESULT_SUCCESS, true);
			add_evt(31);
			return;
		}

		add_evt(pipisr);
	}

	/* DMA interrupts */
	if (isr & HSTISR_DMA_)
	{
		add_evt(1000);

    //     int8_t              pi  = 7 - __CLZ(isr & imr & USBHS_HSTISR_DMA__Msk);
		uint8_t pipe = 7 - __CLZ(isr & USBHS_HSTISR_DMA__Msk);

		uint8_t address = hw_pipe_get_address(rhport, pipe);
		uint8_t endpoint = hw_pipe_get_endpoint(rhport, pipe);
    //     struct _usb_h_prvt *pd = (struct _usb_h_prvt *)drv->prvt;
    //     struct usb_h_pipe * p;
    //     uint32_t            imr = hri_usbhs_read_HSTIMR_reg(drv->hw);

    //     uint32_t dmastat;
    //     uint8_t *buf;
    //     uint32_t size, count;
    //     uint32_t n_remain;
    //     if (pi < 0) {
    //         return;
    //     }

    //     dmastat = hri_usbhs_read_HSTDMASTATUS_reg(drv->hw, pi - 1);
        uint32_t stat = USB_REG->HSTDMA[pipe - 1].HSTDMASTATUS;
    //     if (dmastat & USBHS_HSTDMASTATUS_CHANN_ENB) {
    //         return; /* Ignore EOT_STA interrupt */
    //     }
        if (stat & HSTDMASTATUS_CHANN_ENB)
        {
			add_evt(1001);
            return;
        }

    //     p = &pd->pipe_pool[pi];
    // #if _HPL_USB_H_HBW_SP
    //     if (p->high_bw_out) {
    //         /* Streaming out, no ACK, assume everything sent */
    //         _usb_h_ll_dma_out(p, _usb_h_ll_get(pi, p->bank));
    //         return;
    //     }
    // #endif

		if (pipe == 2)
		{
			breakpoint();
		}

    //     /* Save number of data no transfered */
    //     n_remain = (dmastat & USBHS_HSTDMASTATUS_BUFF_COUNT_Msk) >> USBHS_HSTDMASTATUS_BUFF_COUNT_Pos;
        uint16_t remaining = (stat & HSTDMASTATUS_BUFF_COUNT) >> HSTDMASTATUS_BUFF_COUNT_Pos;
		pipes[pipe].proclen -= remaining;


		if (pipes[pipe].proclen >= pipes[pipe].buflen)
		{
			add_evt(1010);
			pipes[pipe].buf = 0;
			hcd_event_xfer_complete(address, endpoint, pipes[pipe].proclen, XFER_RESULT_SUCCESS, true);
			add_evt(1011);
		}
		// else
		// {
		// 	hw_pipe_setup_dma(rhport, pipe, remaining);
		// }

    //     if (n_remain) {
    //         _usb_h_load_x_param(p, &buf, &size, &count);
    //         (void)buf;
    //         (void)size;
    //         /* Transfer no complete (short packet or ZLP) then:
    //         * Update number of transfered data
    //         */
    //         count -= n_remain;
    //         _usb_h_save_x_param(p, count);
    //     }

    //     /* Pipe IN: freeze status may delayed */
    //     if (p->ep & 0x80) {
    //         if (!hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE)) {
    //             /* Pipe is not frozen in case of :
    //             * - incomplete transfer when the request number INRQ is not
    //             *   complete.
    //             * - low USB speed and with a high CPU frequency,
    //             *   a ACK from host can be always running on USB line.
    //             */
    //             if (n_remain) {
    //                 /* Freeze pipe in case of incomplete transfer */
    //                 hri_usbhs_write_HSTPIPIER_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE);
    //             } else {
    //                 /* Wait freeze in case of ACK on going */
    //                 while (!hri_usbhs_get_HSTPIPIMR_reg(drv->hw, pi, USBHS_HSTPIPIMR_PFREEZE)) {
    //                 }
    //             }
    //         }
    //     }
    //     _usb_h_dma(p, (bool)n_remain);

		return;
	}

	add_evt(isr);
}


void HardFault_Handler(void)
{
	breakpoint();
}


#endif