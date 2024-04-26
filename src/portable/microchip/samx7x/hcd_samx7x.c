/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 * Copyright (c) 2021, HiFiPhile
 * Copyright (c) 2024, Duet3D
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

#include <assert.h>

#include "host/hcd.h"
#include "sam.h"
#include "common_usb_regs.h"
#include "common_usb_defs.h"

#define DEBUG 1

#if DEBUG
extern bool board_trigger_pin(bool set);

static inline void toggle_trigger(void)
{
  static bool trigger = false;
  trigger = !trigger;
  board_trigger_pin(trigger);
}

void breakpoint(void)
{
  volatile int a = 0;
  a++;
}

#define MAX_EVENTS 1000

static uint32_t evts[MAX_EVENTS];
static uint32_t evti = 0;

static inline void add_evt(uint32_t num)
{
  if (evti == 0)
  {
    memset(&evts, 0, sizeof(evts));
  }

  if (evti < MAX_EVENTS)
  {
    ++evti;
    evts[evti - 1] = num;
  }
}
#endif

#ifndef USE_DUAL_BANK
#  if TUD_OPT_HIGH_SPEED
#    define USE_DUAL_BANK   0
#  else
#    define USE_DUAL_BANK   1
#  endif
#endif

static bool ready = false;

typedef struct
{
  uint8_t *buffer;
  uint16_t total;
  uint16_t processed;
  uint16_t out;
  bool dma;
} hw_pipe_xfer_t;

static hw_pipe_xfer_t pipe_xfers[EP_MAX];

static inline bool hw_pipe_enabled(uint8_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (USB_REG->HSTPIP & (HSTPIP_PEN0 << pipe));
}

static inline void hw_pipe_clear_reg(uint8_t rhport, uint8_t pipe, uint32_t mask)
{
  (void) rhport;
  USB_REG->HSTPIPICR[pipe] = mask;
}

static inline void hw_pipe_disable_reg(uint8_t rhport, uint8_t pipe, uint32_t mask)
{
  (void) rhport;
  USB_REG->HSTPIPIDR[pipe] = mask;
}

static inline uint16_t hw_pipe_bytes(uint8_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (USB_REG->HSTPIPISR[pipe] & USBHS_HSTPIPISR_PBYCT_Msk) >> USBHS_HSTPIPISR_PBYCT_Pos;
}

static inline void hw_pipe_enable_reg(uint8_t rhport, uint8_t pipe, uint32_t mask)
{
  (void) rhport;
  USB_REG->HSTPIPIER[pipe] = mask;
}

static inline uint8_t hw_pipe_get_ep_addr(uint8_t rhport, uint8_t pipe)
{
  (void)rhport;
  uint8_t ep_num = ((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PEPNUM) >> HSTPIPCFG_PEPNUM_Pos);
  if (ep_num)
  {
    bool in = ((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PTOKEN) >> HSTPIPCFG_PTOKEN_Pos) == HSTPIPCFG_PTOKEN_IN_Val;
    return ep_num | (in ? TUSB_DIR_IN_MASK : 0x00);
  }
  return ep_num; // ep0 in and out on the same pipe
}

static inline uint8_t hw_pipe_get_dev_addr(uint8_t rhport, uint8_t pipe)
{
  (void)rhport;
  uint8_t index = pipe >> 2;
  uint8_t pos = (pipe & 0x3) << 3;
  uint32_t reg = (&USB_REG->HSTADDR1)[index];
  return ((reg & (0x7F << pos)) >> pos);
}

static inline void hw_pipe_set_token(uint8_t rhport, uint8_t pipe, uint32_t token)
{
  (void)rhport;
  uint32_t tmp = USB_REG->HSTPIPCFG[pipe];
  tmp &= ~HSTPIPCFG_PTOKEN;
  tmp |= HSTPIPCFG_PTOKEN & (token);
  USB_REG->HSTPIPCFG[pipe] = tmp;
}

static uint8_t hw_pipe_find(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr)
{
  (void)rhport;

  if (!(ep_addr & (TUSB_DIR_IN_MASK - 1))) // check if ep0, which has in and out on same pipe
  {
    ep_addr = 0;
  }

  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    if (hw_pipe_enabled(rhport, i) &&
        hw_pipe_get_dev_addr(rhport, i) == dev_addr &&
        hw_pipe_get_ep_addr(rhport, i) == ep_addr)
    {
      return i;
    }
  }

  return EP_MAX;
}

static uint8_t hw_pipe_find_free(uint8_t rhport)
{
  (void)rhport;
  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    if (!hw_pipe_enabled(rhport, i))
    {
      return i;
    }
  }
  return EP_MAX;
}

static inline void hw_pipe_enable(uint8_t rhport, uint8_t pipe, bool enable)
{
  uint32_t mask = HSTPIP_PEN0 << pipe;
  if (enable)
  {
    USB_REG->HSTPIP |= mask;
  }
  else
  {
    USB_REG->HSTPIP &= ~mask;
  }
}

static void hw_pipe_reset(uint8_t rhport, uint8_t pipe)
{
  (void)rhport;
  memset(&pipe_xfers[pipe], 0, sizeof(pipe_xfers[pipe]));
  uint32_t mask = HSTPIP_PRST0 << pipe;
  USB_REG->HSTPIP |= mask;    // put pipe in reset
  USB_REG->HSTPIP &= ~mask; // remove pipe from reset
}

static void hw_pipes_reset(uint8_t rhport)
{
  (void)rhport;
  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    hw_pipe_reset(rhport, i);
  }
}

static void hw_pipe_abort(uint8_t rhport, uint8_t pipe)
{
  // hri_usbhs_set_HSTPIP_reg(drv->hw, USBHS_HSTPIP_PRST0 << pi);
  // hri_usbhs_clear_HSTPIP_reg(drv->hw, USBHS_HSTPIP_PRST0 << pi);
  hw_pipe_reset(rhport, pipe);

  // /* Disable interrupts */
  // hri_usbhs_write_HSTPIPIDR_reg(drv->hw,
  // 							pi,
  // 							USBHS_HSTPIPIMR_RXINE | USBHS_HSTPIPIMR_TXOUTE | USBHS_HSTPIPIMR_TXSTPE
  // 								| USBHS_HSTPIPIMR_RXSTALLDE | USBHS_HSTPIPIMR_SHORTPACKETIE);

  hw_pipe_disable_reg(rhport, pipe, HSTPIPIMR_RXINE | HSTPIPIMR_TXOUTE |
    HSTPIPIMR_CTRL_TXSTPE | HSTPIPIMR_BLK_RXSTALLDE | HSTPIPIMR_SHORTPACKETIE);
}

static uint16_t hw_compute_psize(uint16_t size)
{
  static const uint16_t sizes[] = {8, 16, 32, 64, 128, 256, 512, 1024};
  uint8_t i;
  for (i = 0; i < sizeof(sizes) / sizeof(uint16_t); i++)
  {
    /* Size should be exactly PSIZE values */
    if (size <= sizes[i])
    {
      return i;
    }
  }
  return 7;
}

static uint16_t hw_pipe_get_size(uint16_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (1 << (((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PSIZE) >> HSTPIPCFG_PSIZE_Pos) + 3));
}

static bool hw_pipe_prepare_out(uint8_t rhport, uint8_t pipe)
{
  if (pipe_xfers[pipe].total)
  {
    uint16_t remain = - hw_pipe_bytes(rhport, pipe);
    pipe_xfers[pipe].processed += pipe_xfers[pipe].out - remain;

    if (pipe_xfers[pipe].processed < pipe_xfers[pipe].total)
    {
      uint16_t pipe_left = hw_pipe_get_size(rhport, pipe) - remain;
      uint16_t buffer_left = pipe_xfers[pipe].total - pipe_xfers[pipe].processed;
      pipe_xfers[pipe].out = pipe_left < buffer_left ? pipe_left : buffer_left;

      uint8_t *dst = PEP_GET_FIFO_PTR(pipe, 8) + remain;
      uint8_t *src = pipe_xfers[pipe].buffer + pipe_xfers[pipe].processed;
      for (size_t i = 0; i < pipe_xfers[pipe].out; i++)
      {
        *dst++ = *src++;
      }
      return true;
    }
  }
  return false;
}

static void hw_handle_pipe_int(uint8_t rhport, uint32_t isr)
{
  uint8_t pipe = 23 - __CLZ(isr & HSTISR_PEP_);
  uint32_t pipisr = USB_REG->HSTPIPISR[pipe];

  uint8_t dev_addr = hw_pipe_get_dev_addr(rhport, pipe);
  uint8_t ep_addr = hw_pipe_get_ep_addr(rhport, pipe);

  if (pipisr & HSTPIPISR_CTRL_RXSTALLDI)
  {
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_RXSTALLDIC);
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_RSTDTS);
    hw_pipe_abort(rhport, pipe);
    hcd_event_xfer_complete(dev_addr, ep_addr, 0, XFER_RESULT_STALLED, true);
    return;
  }

  if (pipisr & HSTPIPISR_PERRI)
  {
    xfer_result_t res = (USB_REG->HSTPIPERR[pipe] & HSTPIPERR_TIMEOUT)
                        ? XFER_RESULT_TIMEOUT : XFER_RESULT_FAILED;
    hw_pipe_abort(rhport, pipe);
    hcd_event_xfer_complete(dev_addr, ep_addr, 0, res, true);
    return;
  }

  if (pipisr & HSTPIPISR_CTRL_TXSTPI)
  {
    // Clear and disable setup packet interrupt
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_TXSTPIC);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_CTRL_TXSTPEC);
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
    // Notify USB stack of setup transmit success
    hcd_event_xfer_complete(dev_addr, ep_addr, 8, XFER_RESULT_SUCCESS, true);
    return;
  }

  if (pipisr & HSTPIPISR_RXINI || pipisr & HSTPIPISR_SHORTPACKETI)
  {
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC | HSTPIPICR_SHORTPACKETIC);

    // In case of low USB speed and with a high CPU frequency,
    // a ACK from host can be always running on USB line
    // then wait end of ACK on IN pipe.
    if (!(USB_REG->HSTPIPINRQ[pipe]))
    {
      while (!(USB_REG->HSTPIPIMR[pipe] & HSTPIPIMR_PFREEZE));
    }

    if (pipe_xfers[pipe].total)
    {
      uint32_t rx = hw_pipe_bytes(rhport, pipe);
      uint8_t *src = PEP_GET_FIFO_PTR(pipe, 8);
      uint8_t *dst = pipe_xfers[pipe].buffer + pipe_xfers[pipe].processed;
      for (size_t i = 0; i < rx; i++)
      {
        *dst++ = *src++;
      }
      pipe_xfers[pipe].processed += rx;
      if (pipe_xfers[pipe].processed < pipe_xfers[pipe].total)
      {
        hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC | HSTPIPIDR_FIFOCONC);
        return;
      }
    }
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_SHORTPACKETIEC | HSTPIPIDR_RXINEC);
    hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].total, XFER_RESULT_SUCCESS, true);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);
    return;
  }

  if (pipisr & HSTPIPISR_TXOUTI)
  {
    // Freeze the pipe
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
    // Clear transmit interrupt
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_TXOUTIC);

    if(hw_pipe_prepare_out(rhport, pipe))
    {
      // Still more data to send
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC | HSTPIPIDR_FIFOCONC);
      return;
    }
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_TXOUTEC);
    // Notify the USB stack
    hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].total, XFER_RESULT_SUCCESS, true);
    return;
  }
}

static void hw_handle_dma_int(uint8_t rhport, uint32_t isr)
{
  uint8_t pipe = 7 - __CLZ(isr & HSTISR_DMA_);
  uint8_t channel = pipe - 1;

  uint32_t stat = USB_REG->HSTDMA[channel].HSTDMASTATUS;

  if (stat & HSTDMASTATUS_CHANN_ENB)
  {
    return; // ignore EOT_STA interrupt
  }

  USB_REG->HSTIDR |= HSTIDR_DMA_0 << channel;
  uint16_t remain = (stat & HSTDMASTATUS_BUFF_COUNT) >> HSTDMASTATUS_BUFF_COUNT_Pos;
  pipe_xfers[pipe].processed = pipe_xfers[pipe].total - remain;

  uint8_t dev_addr = hw_pipe_get_dev_addr(rhport, pipe);
  uint8_t ep_addr = hw_pipe_get_ep_addr(rhport, pipe);
  hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].processed, XFER_RESULT_SUCCESS, true);
}

static bool hw_pipe_setup_dma(uint8_t rhport, uint8_t pipe, bool end)
{
  uint8_t ep_addr = hw_pipe_get_ep_addr(rhport, pipe);

  uint32_t dma_ctrl = USBHS_HSTDMACONTROL_BUFF_LENGTH(pipe_xfers[pipe].total);
  // if (ep_addr & TUSB_DIR_IN_MASK)
  // {
  //   dma_ctrl |= HSTDMACONTROL_END_TR_IT | HSTDMACONTROL_END_TR_EN;
  // }
  // else
  {
    if ((pipe_xfers[pipe].total & (hw_pipe_get_size(rhport, pipe) - 1)) != 0)
    {
      dma_ctrl |= HSTDMACONTROL_END_B_EN;
    }
  }

  uint8_t channel = pipe - 1;
  USB_REG->HSTDMA[channel].HSTDMAADDRESS = (uint32_t)(&(pipe_xfers[pipe].buffer));
  dma_ctrl |= HSTDMACONTROL_END_BUFFIT | HSTDMACONTROL_CHANN_ENB;

  uint32_t flags = 0;
  hw_enter_critical(&flags);
  if (!(USB_REG->HSTDMA[channel].HSTDMASTATUS & HSTDMASTATUS_END_TR_ST))
  {
    USB_REG->HSTIER |= HSTIER_DMA_0 << channel;
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_NBUSYBKEC | HSTPIPIDR_PFREEZEC);
    USB_REG->HSTDMA[channel].HSTDMACONTROL = dma_ctrl;
    hw_exit_critical(&flags);
    return true;
  }
  hw_exit_critical(&flags);
  return false;
}

static void hw_handle_rh_int(uint8_t rhport, uint32_t isr)
{
  // Device reset
  if (isr & HSTISR_RSTI)
  {
    USB_REG->HSTICR |= HSTICR_RSTIC;
    USB_REG->HSTIDR |= HSTIDR_RSTIEC;
    ready = true;
    return;
  }

  // Device disconnection
  if (isr & HSTISR_DDISCI)
  {
    USB_REG->HSTICR = HSTICR_DDISCIC | HSTICR_DCONNIC;
    USB_REG->HSTIDR = HSTIDR_DDISCIEC | HSTIDR_HWUPIEC | HSTIDR_RSMEDIEC | HSTIDR_RXRSMIEC;
    USB_REG->HSTCTRL &= ~HSTCTRL_RESET;
    USB_REG->HSTICR = HSTICR_DCONNIC | HSTICR_HWUPIC | HSTICR_RSMEDIC | HSTICR_RXRSMIC;
    USB_REG->HSTIER = HSTIER_DCONNIES | HSTIER_HWUPIES | HSTIER_RSMEDIES | HSTIER_RXRSMIES;
    hcd_event_device_remove(rhport, true);
    return;
  }

  // Device connection
  if (isr & HSTISR_DCONNI)
  {
    USB_REG->HSTICR |= HSTICR_DCONNIC;
    USB_REG->HSTIDR |= HSTISR_DCONNI;
    USB_REG->HSTIER |= HSTIER_DDISCIES;
    USB_REG->HSTCTRL |= HSTCTRL_SOFE;
    hcd_event_device_attach(rhport, true);
    return;
  }

  // Host wakeup
  if (isr & HSTISR_HWUPI)
  {
    USB_REG->CTRL &= ~CTRL_FRZCLK;
    while (!(USB_REG->SR & SR_CLKUSABLE));

    USB_REG->HSTIDR |= HSTIDR_HWUPIEC;
    USB_REG->SFR |= SFR_VBUSRQS;
    USB_REG->HSTICR |= HSTICR_HWUPIC;
    USB_REG->HSTIDR |= HSTIDR_HWUPIEC;
    USB_REG->HSTIER |= HSTIER_DCONNIES;
    return;
  }
}

bool hcd_init(uint8_t rhport)
{
  (void)rhport;
  hcd_int_disable(rhport);
  hw_pipes_reset(rhport);

  USB_REG->CTRL &= ~(CTRL_UIMOD | CTRL_UID);
  USB_REG->CTRL |= CTRL_FRZCLK;

  USB_REG->HSTCTRL &= ~HSTCTRL_SPDCONF;

  // Force re-connection on initialization
  // TODO: check if this is really correct
  USB_REG->HSTIFR |= HSTIMR_DDISCIE | HSTIMR_HWUPIE;

  USB_REG->CTRL = CTRL_USBE;
  USB_REG->HSTICR |= (HSTICR_DCONNIC | HSTICR_HSOFIC | HSTICR_RSMEDIC | HSTICR_RSTIC | HSTICR_RXRSMIC);
  USB_REG->CTRL |= CTRL_VBUSHWC; // datasheet indicates must be set to 1
  USB_REG->HSTIER |= HSTIER_DCONNIES | HSTIMR_RSTIE | HSTIER_HWUPIES;

  ready = false;

  return true;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  // Reset every pipe associated with the device
  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    if (hw_pipe_enabled(rhport, i) && hw_pipe_get_dev_addr(rhport, i) == dev_addr)
    {
      hw_pipe_reset(rhport, i);
      hw_pipe_enable(rhport, i, false);
    }
  }
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
  uint8_t pipe = 0;
  pipe = hw_pipe_find(rhport, dev_addr, 0);
  if (pipe >= EP_MAX)
  {
    return false;
  }
  // Set pipe token to setup
  hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_SETUP);
  hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC);
  // Clear setup token interrupt
  hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_TXSTPIC);
  // Enable setup token interrupt
  hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_CTRL_TXSTPES);
  // Disable pipe freeze and control clear
  // Copy setup data to USB buffer
  const uint8_t *src = setup_packet;
  uint8_t *dst = PEP_GET_FIFO_PTR(pipe, 8);
  for (size_t i = 0; i < 8; i++)
  {
    *dst++ = *src++;
  }
  // Disable pipe freeze and control clear
  __DSB();
  __ISB();
  hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);
  return true;
}

void hcd_int_enable(uint8_t rhport)
{
  (void)rhport;
  NVIC_EnableIRQ((IRQn_Type)ID_USBHS);
}

void hcd_int_disable(uint8_t rhport)
{
  (void)rhport;
  NVIC_DisableIRQ((IRQn_Type)ID_USBHS);
}

void hcd_port_reset(uint8_t rhport)
{
  (void)rhport;
  // Enable reset sent interrupt
  USB_REG->HSTIER |= HSTIER_RSTIES;
  // Send reset
  USB_REG->HSTCTRL |= HSTCTRL_RESET;
}

void hcd_port_reset_end(uint8_t rhport)
{
  (void)rhport;
}

bool hcd_port_connect_status(uint8_t rhport)
{
  (void)rhport;
  return ready;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
  return hw_port_speed_get();
}

uint32_t hcd_frame_number(uint8_t rhport)
{
  return (USB_REG->HSTFNUM & HSTFNUM_FNUM) >> HSTFNUM_FNUM_Pos;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const *ep_desc)
{
  uint8_t pipe = 0;
  pipe = hw_pipe_find_free(rhport);
  if (pipe >= EP_MAX)
  {
    return false;
  }

  // Configure the pipe
  hw_pipe_reset(rhport, pipe); // reset the pipe

  // Prepare pipe configuration
  uint32_t cfg = 0;

  static_assert(TUSB_XFER_CONTROL == HSTPIPCFG_PTYPE_CTRL_Val && // check that tusb_xfer_type_t has
                TUSB_XFER_ISOCHRONOUS == HSTPIPCFG_PTYPE_ISO_Val && // the same numerical value as
                TUSB_XFER_BULK == HSTPIPCFG_PTYPE_BLK_Val && // the one the peripheral expects, to
                TUSB_XFER_INTERRUPT == HSTPIPCFG_PTYPE_INTRPT_Val); // avoid a switch statement/ if-else
  tusb_xfer_type_t type = ep_desc->bmAttributes.xfer;
  cfg |= (uint32_t)type << HSTPIPCFG_PTYPE_Pos;

  bool in = ep_desc->bEndpointAddress & TUSB_DIR_IN_MASK; // configure the token, for control pipes
  cfg |= (type == TUSB_XFER_CONTROL ? HSTPIPCFG_PTOKEN_SETUP : // set it to setup token; since it's
          in ? HSTPIPCFG_PTOKEN_IN : HSTPIPCFG_PTOKEN_OUT); // going to be set to this on setup anyways

  // ep_addr number, without dir bit
  cfg |= (HSTPIPCFG_PEPNUM & ((uint32_t)(ep_desc->bEndpointAddress) << HSTPIPCFG_PEPNUM_Pos));

  uint16_t size = ep_desc->wMaxPacketSize & (PIPE_MAX_PACKET_SIZE - 1); // mask with max packet size the
  cfg |= (HSTPIPCFG_PSIZE & ((uint32_t)hw_compute_psize(size) << HSTPIPCFG_PSIZE_Pos)); // hardware supports

  cfg |= HSTPIPCFG_PBK_1_BANK;
#if USE_DUAL_BANK
  if (type == TUSB_XFER_ISOCHRONOUS || type == TUSB_XFER_BULK)
  {
    cfg |= HSTPIPCFG_PBK_2_BANK;
  }
#endif

  bool dma = EP_DMA_SUPPORT(pipe) && type != TUSB_XFER_CONTROL;

  if (dma)
  {
    cfg |= HSTPIPCFG_AUTOSW; // needed for dma
  }

  cfg |= HSTPIPCFG_ALLOC; // alloc dpram for pipe

  hw_pipe_enable(rhport, pipe, true);
  USB_REG->HSTPIPCFG[pipe] = cfg; // write prepared configuration

  if (USB_REG->HSTPIPISR[pipe] & HSTPIPISR_CFGOK) // check if pipe enabling succeeded with ok configuration
  {
    // setup device dev_addr for pipe
    uint8_t reg_i = pipe >> 2;
    uint8_t pos = (pipe & 0x3) << 3;
    uint32_t reg = (&USB_REG->HSTADDR1)[reg_i];
    reg &= ~(0x7F << pos);
    reg |= (dev_addr & 0x7F) << pos;
    (&USB_REG->HSTADDR1)[reg_i] = reg;

    // configure pipe-related interrupts
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_RXSTALLDIC | HSTPIPICR_OVERFIC | HSTPIPISR_PERRI | HSTPIPICR_INTRPT_UNDERFIC);
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_CTRL_RXSTALLDES | HSTPIPIER_OVERFIES | HSTPIPIER_PERRES);
    USB_REG->HSTIER |= (HSTISR_PEP_0 | (HSTISR_DMA_0 >> 1)) << pipe;

    pipe_xfers[pipe].dma = dma;

    return true;
  }

  hw_pipe_enable(rhport, pipe, false);
  return false;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
  uint8_t pipe = 0;
  pipe = hw_pipe_find(rhport, dev_addr, ep_addr);
  if (pipe >= EP_MAX)
  {
    return false;
  }

  pipe_xfers[pipe].buffer = buffer;
  pipe_xfers[pipe].total = buflen;
  pipe_xfers[pipe].processed = 0;
  pipe_xfers[pipe].out = 0;

  if (pipe_xfers[pipe].dma)
  {
    hw_cache_invalidate((uint32_t *)tu_align((uint32_t)buffer, 4), buflen + 31);
    return hw_pipe_setup_dma(rhport, pipe, false);
  }
  else
  {
    if (ep_addr & TUSB_DIR_IN_MASK)
    {
      hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_IN);
      hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC | HSTPIPICR_SHORTPACKETIC);
      hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_RXINES);
      __DSB();
      __ISB();
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC);
    }
    else
    {
      hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_OUT);
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC);
      hw_pipe_clear_reg(rhport, pipe, HSTPIPISR_TXOUTI);
      hw_pipe_prepare_out(rhport, pipe);
      hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_TXOUTES);
      __DSB();
      __ISB();
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);
    }
  }

  return true;
}

void hcd_int_handler(uint8_t rhport)
{
  uint32_t isr = USB_REG->HSTISR;

  // Change to low power mode to only use the 48 MHz clock on LS
  if (hcd_port_speed_get(rhport) == TUSB_SPEED_LOW)
  {
    if (!(USB_REG->HSTCTRL & USBHS_HSTCTRL_SPDCONF_Msk))
    {
      USB_REG->HSTCTRL |= HSTCTRL_SPDCONF_LOW_POWER;
    }
  }

  // Pipe processing & exception interrupts
  if (isr & HSTISR_PEP_)
  {
    hw_handle_pipe_int(rhport, isr);
    return;
  }

  // DMA processing interrupts
  if (isr & HSTISR_DMA_)
  {
    hw_handle_dma_int(rhport, isr);
    return;
  }

  // Host global (root hub) processing interrupts
  if (isr & (HSTISR_RSTI | HSTISR_DCONNI | HSTISR_DDISCI | HSTISR_HWUPI | HSTISR_RXRSMI))
  {
    hw_handle_rh_int(rhport, isr);
    return;
  }
}

#endif
