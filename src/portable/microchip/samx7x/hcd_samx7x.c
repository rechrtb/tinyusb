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

// Check that tusb_xfer_type_t has the same numerical value as the
// peripheral definitions. This way, casting can be used instead of conditionals
// or switches.
static_assert(TUSB_XFER_CONTROL == HSTPIPCFG_PTYPE_CTRL_Val &&
              TUSB_XFER_ISOCHRONOUS == HSTPIPCFG_PTYPE_ISO_Val &&
              TUSB_XFER_BULK == HSTPIPCFG_PTYPE_BLK_Val &&
              TUSB_XFER_INTERRUPT == HSTPIPCFG_PTYPE_INTRPT_Val);

#ifndef USE_DUAL_BANK
#  if TUD_OPT_HIGH_SPEED
#    define USE_DUAL_BANK   0
#  else
#    define USE_DUAL_BANK   1
#  endif
#endif

#define RET_IF_TRUE(fn)      if (fn) { return; }

typedef enum {
  HCD_IDLE,
  HCD_ATTACHING,
  HCD_RESETTING,
  HCD_CONNECTED
} hcd_status_t;

static volatile hcd_status_t status[1] = { HCD_IDLE };
typedef struct
{
  uint8_t *buffer;
  uint16_t total;
  uint16_t done;
  bool dma;
} hw_pipe_xfer_t;

static hw_pipe_xfer_t pipe_xfers[EP_MAX];

static volatile uint8_t events[1500];
static volatile uint16_t events_idx = 0;

static inline bool hw_pipe_enabled(uint8_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (USB_REG->HSTPIP & (HSTPIP_PEN0 << pipe));
}

static inline uint8_t hw_pipe_interrupt(uint8_t rhport)
{
  return __builtin_ctz((((USB_REG->HSTISR) & (USB_REG->HSTIMR)) >> 8) | (1 << EP_MAX));
}

static inline uint8_t hw_pipe_dma_interrupt(uint8_t rhport)
{
	return (__builtin_ctz((((USB_REG->HSTISR) & (USB_REG->HSTIMR)) >> 25) | (1 << (EP_MAX-1))) + 1);
}

static inline bool hw_pipe_frozen(uint8_t rhport, uint8_t pipe)
{
  return (USB_REG->HSTPIPIMR[pipe] & HSTPIPIMR_PFREEZE);
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

static uint16_t hw_pipe_get_size(uint16_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (1 << (((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PSIZE) >> HSTPIPCFG_PSIZE_Pos) + 3));
}

static uint8_t hw_pipe_find_free(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc)
{
  (void)rhport;
  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    // Somewhat naive approach: find a pipe that's currently disabled, and has either not been allocated before
    // or has an allocation fulfilling the size requested, if previously allocated. Perhaps in the future
    // some form of algorithm to detect whether DPRAM can be deallocated is needed.
    size_t sz = hw_pipe_get_size(rhport, i) * (((USB_REG->HSTPIPCFG[i] & HSTPIPCFG_PBK) >> HSTPIPCFG_PBK_Pos) + 1);
    if (!hw_pipe_enabled(rhport, i) && (!(USB_REG->HSTPIPCFG[i] & HSTPIPCFG_ALLOC) || ep_desc->wMaxPacketSize <= sz))
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

  // Reset pipe to stop transfer
  uint32_t mask = HSTPIP_PRST0 << pipe;
  USB_REG->HSTPIP |= mask;    // put pipe in reset
  USB_REG->HSTPIP &= ~mask; // remove pipe from reset

  // Disable pipe interrupts
  hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_RXINEC | HSTPIPIDR_SHORTPACKETIEC |
    HSTPIPIDR_TXOUTEC | HSTPIPIDR_CTRL_TXSTPEC | HSTPIPIDR_BLK_RXSTALLDEC);

  // Clear pipe context
  memset(&pipe_xfers[pipe], 0, sizeof(pipe_xfers[pipe]));
}

static void hw_pipes_reset(uint8_t rhport)
{
  (void)rhport;
  for (int8_t i = 0; i < EP_MAX; i++) // go from high to low endpoints
  {
    hw_pipe_reset(rhport, i);
  }
}

static uint16_t hw_compute_psize(uint16_t size)
{
  static const uint16_t sizes[] = {8, 16, 32, 64, 128, 256, 512, 1024};
  uint8_t i = 0;
  for (; i < sizeof(sizes) / sizeof(uint16_t); i++)
  {
    /* Size should be exactly PSIZE values */
    if (size <= sizes[i])
    {
      return i;
    }
  }
  return 7;
}


static tusb_xfer_type_t hw_pipe_get_type(uint16_t rhport, uint8_t pipe)
{
  (void) rhport;
  return (tusb_xfer_type_t)((USB_REG->HSTPIPCFG[pipe] & HSTPIPCFG_PTYPE) >> HSTPIPCFG_PTYPE_Pos);
}

static bool hw_pipe_fifo_copy_out(uint8_t rhport, uint8_t pipe)
{
  uint32_t remain = pipe_xfers[pipe].total - pipe_xfers[pipe].done;
  uint16_t pipe_size = hw_pipe_get_size(rhport, pipe);

  uint32_t next = remain < pipe_size ? remain : pipe_size;

  if (next)
  {
    uint8_t *dst = PEP_GET_FIFO_PTR(pipe, 8);
    uint8_t *src = pipe_xfers[pipe].buffer + pipe_xfers[pipe].done;
    memcpy(dst, src, next);
    pipe_xfers[pipe].done += next;
  }

  // Returns if last segment to transmit: the last segment has been copied to
  // the FIFO or the current segment is short.
  return pipe_xfers[pipe].done >= pipe_xfers[pipe].total || next < pipe_size;
}

static bool hw_pipe_fifo_copy_in(uint8_t rhport, uint8_t pipe)
{
  uint32_t recieved = hw_pipe_bytes(rhport, pipe);

  if (recieved)
  {
    // Copy data from FIFO to buffer
    uint8_t *src = PEP_GET_FIFO_PTR(pipe, 8);
    uint8_t *dst = pipe_xfers[pipe].buffer + pipe_xfers[pipe].done;
    memcpy(dst, src, recieved);
    pipe_xfers[pipe].done += recieved;
  }

  return pipe_xfers[pipe].done >= pipe_xfers[pipe].total || recieved < hw_pipe_get_size(rhport, pipe);
}

static bool hw_handle_fifo_pipe_int(uint8_t rhport, uint8_t pipe, uint8_t dev_addr, uint8_t ep_addr)
{
  if ((((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_RXINI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_RXINE)))
  {
    //events[events_idx++] = 50;
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC);

    if (hw_pipe_fifo_copy_in(rhport, pipe))
    {
      //events[events_idx++] = 51;
      hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_RXINEC);
      USB_REG->HSTPIPINRQ[pipe] &= ~HSTPIPINRQ_INMODE;
      USB_REG->HSTIDR = ((HSTISR_PEP_0) << pipe);
      hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].done, XFER_RESULT_SUCCESS, true);
    }

    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);
    return true;
  }

  if (((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_TXOUTI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_TXOUTE))
  {
    //events[events_idx++] = 52;
    // Clear transmit interrupt
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_TXOUTIC);
    bool last = hw_pipe_fifo_copy_out(rhport, pipe);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);

    if (last) // on last segment, wait for banks to be empty if not isochronous
    {
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_TXOUTEC);
      if (hw_pipe_get_type(rhport, pipe) == TUSB_XFER_ISOCHRONOUS) // for isochronous pipes, no need for ack
      {
        hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
        USB_REG->HSTIDR = ((HSTISR_PEP_0) << pipe);
        hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].done, XFER_RESULT_SUCCESS, true);
      }
      else
      {
        //events[events_idx++] = 53;
        hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_NBUSYBKES);
      }
    }
    return true;
  }

  if ((((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_NBUSYBK) == 0) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_NBUSYBKE))
  {
    //events[events_idx++] = 54;
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_NBUSYBKEC);

    if (hw_pipe_get_type(rhport, pipe) != TUSB_XFER_ISOCHRONOUS) // for isochornous pipes, this was done on tx out handling
    {
      hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
      USB_REG->HSTIDR = ((HSTISR_PEP_0) << pipe);
      hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].done, XFER_RESULT_SUCCESS, true);
    }
    return true;
  }

  return false;
}

bool hw_handle_ctrl_pipe_int(uint8_t rhport, uint8_t pipe, uint8_t dev_addr, uint8_t ep_addr)
{
  static_assert(HSTPIPISR_CTRL_TXSTPI == HSTPIPISR_BLK_TXSTPI);
  static_assert(HSTPIPIMR_CTRL_TXSTPE == HSTPIPIMR_BLK_TXSTPE);
  if (((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_CTRL_TXSTPI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_CTRL_TXSTPE))
  {
    //events[events_idx++] = 40;
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
    // Clear and disable setup packet interrupt
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_CTRL_TXSTPEC);
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_TXSTPIC);
    // Notify TinyUSB of setup transmit success
    hcd_event_xfer_complete(dev_addr, ep_addr, 8, XFER_RESULT_SUCCESS, true);
    return true;
  }

  if ((((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_RXINI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_RXINE)))
  {
    //events[events_idx++] = 41;
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_RXINEC);

    // In case of low USB speed and with a high CPU frequency,
    // a ACK from host can be always running on USB line
    // then wait end of ACK on IN pipe.
    while (!(USB_REG->HSTPIPIMR[pipe] & HSTPIPIMR_PFREEZE));

    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC);

    hw_pipe_fifo_copy_in(rhport, pipe);
    hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].done, XFER_RESULT_SUCCESS, true);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC);
    return true;
  }

  if (((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_TXOUTI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_TXOUTE))
  {
    //events[events_idx++] = 42;
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
    // Clear transmit interrupt
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_TXOUTIC);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_TXOUTEC);
    hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].done, XFER_RESULT_SUCCESS, true);
    return true;
  }

  return false;
}

static bool hw_handle_dma_pipe_int(uint8_t rhport, uint8_t pipe, uint8_t dev_addr, uint8_t ep_addr)
{
  if (((((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_NBUSYBK) >> HSTPIPISR_NBUSYBK_Pos) == 0) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_NBUSYBKE))
  {
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
    hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_NBUSYBKEC);

    if (hw_pipe_get_type(rhport, pipe) != TUSB_XFER_ISOCHRONOUS)
    {
      // For OUT isochronous pipes, already sent transfer complete event
      // during DMA interrupt handling.
      hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].total, XFER_RESULT_SUCCESS, true);
    }

    return true;
  }

  return false;
}

static bool hw_handle_pipe_int(uint8_t rhport)
{
  uint8_t pipe = hw_pipe_interrupt(rhport);
  //events[events_idx++] = 30;

  if (pipe < EP_MAX)
  {
    //events[events_idx++] = 31;
    uint8_t dev_addr = hw_pipe_get_dev_addr(rhport, pipe);
    uint8_t ep_addr = hw_pipe_get_ep_addr(rhport, pipe);

    bool handled = hw_pipe_get_type(rhport, pipe) == TUSB_XFER_CONTROL ?
                   //events[events_idx++] = 32 &&
                   hw_handle_ctrl_pipe_int(rhport, pipe, dev_addr, ep_addr) : pipe_xfers[pipe].dma ?
                   //events[events_idx++] = 33 &&
                   hw_handle_dma_pipe_int(rhport, pipe, dev_addr, ep_addr) :
                   hw_handle_fifo_pipe_int(rhport, pipe, dev_addr, ep_addr);

    if (!handled)
    {
      //events[events_idx++] = 34;

      static_assert(HSTPIPISR_CTRL_RXSTALLDI == HSTPIPISR_BLK_RXSTALLDI);
      static_assert(HSTPIPISR_CTRL_RXSTALLDI == HSTPIPISR_INTRPT_RXSTALLDI);
      static_assert(HSTPIPIMR_CTRL_RXSTALLDE == HSTPIPIMR_BLK_RXSTALLDE);
      static_assert(HSTPIPIMR_CTRL_RXSTALLDE == HSTPIPIMR_INTRPT_RXSTALLDE);
      if (((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_CTRL_RXSTALLDI) & ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_CTRL_RXSTALLDE))
      {
        //events[events_idx++] = 35;
        hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_RXSTALLDIC);
        hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_RSTDTS);
        hw_pipe_reset(rhport, pipe);
        hcd_event_xfer_complete(dev_addr, ep_addr, 0, XFER_RESULT_STALLED, true);
        return true;
      }

      if (((USB_REG->HSTPIPISR[pipe]) & HSTPIPISR_PERRI) && ((USB_REG->HSTPIPIMR[pipe]) & HSTPIPIMR_PERRE))
      {
        //events[events_idx++] = 36;
        xfer_result_t res = (USB_REG->HSTPIPERR[pipe] & HSTPIPERR_TIMEOUT)
                            ? XFER_RESULT_TIMEOUT : XFER_RESULT_FAILED;
        hw_pipe_reset(rhport, pipe);
        hcd_event_xfer_complete(dev_addr, ep_addr, 0, res, true);
        return true;
      }
    }

    return handled;
  }

  return false;
}


static bool hw_handle_dma_int(uint8_t rhport)
{
  uint8_t pipe = hw_pipe_dma_interrupt(rhport);

  if (pipe < EP_MAX)
  {
    uint8_t channel = pipe - 1;

    uint32_t stat = USB_REG->HSTDMA[channel].HSTDMASTATUS;
    if (stat & HSTDMASTATUS_CHANN_ENB)
    {
      return true; // ignore EOT_STA interrupt
    }

    uint8_t dev_addr = hw_pipe_get_dev_addr(rhport, pipe);
    uint8_t ep_addr = hw_pipe_get_ep_addr(rhport, pipe);

    if (ep_addr & TUSB_DIR_IN_MASK)
    {
      uint16_t remaining = (stat & HSTDMASTATUS_BUFF_COUNT) >> HSTDMASTATUS_BUFF_COUNT_Pos;
      if (!hw_pipe_frozen(rhport, pipe))
      {
        // Pipe is not frozen in case of :
        // - incomplete transfer when the request number INRQ is not complete.
        // - low USB speed and with a high CPU frequency,
        // a ACK from host can be always running on USB line.
        if (remaining)
        {
          hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_PFREEZES);
        }
        else
        {
          while(!hw_pipe_frozen(rhport, pipe));
        }
      }

      uint32_t xfered = pipe_xfers[pipe].total - remaining;
      if (ep_addr & TUSB_DIR_IN_MASK)
      {
        hw_cache_invalidate(pipe_xfers[pipe].buffer, xfered);
      }
      hcd_event_xfer_complete(dev_addr, ep_addr, xfered, XFER_RESULT_SUCCESS, true);
    }
    else
    {
      // Turn on busy bank interrupt. For pipes other than isochronous OUT,
      // the transfer complete event is handled there.
      hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_NBUSYBKES);
      if (hw_pipe_get_type(rhport, pipe) == TUSB_XFER_ISOCHRONOUS)
      {
        hcd_event_xfer_complete(dev_addr, ep_addr, pipe_xfers[pipe].total, XFER_RESULT_SUCCESS, true);
      }
    }
    return true;
  }
  return false;
}

static bool hw_handle_rh_int(uint8_t rhport)
{
  // Device reset
  if (((USB_REG->HSTISR) & HSTISR_RSTI) && ((USB_REG->HSTIMR) & HSTIMR_RSTIE))
  {
    //events[events_idx++] = 1;

    status[rhport] = HCD_CONNECTED;

    // Acknowledge device reset interrupt
    USB_REG->HSTICR = HSTICR_RSTIC;
    USB_REG->HSTIDR = HSTIDR_RSTIEC;

    return true;
  }

  // Wait for USB clock to be connected on asynchronous interrupt
  while (!(USB_REG->SR & SR_CLKUSABLE));

  // Device disconnection
  if (((USB_REG->HSTISR) & HSTISR_DDISCI) && ((USB_REG->HSTIMR) & HSTIMR_DDISCIE))
  {
    //events[events_idx++] = 2;

    // Acknowledge disconnection interrupt
    USB_REG->HSTICR = HSTICR_DDISCIC | HSTICR_HSOFIC;
    USB_REG->HSTIDR = HSTIDR_DDISCIEC | HSTIDR_HSOFIEC;

    // Disable reset, in case of disconnection during reset
    USB_REG->HSTCTRL &= ~HSTCTRL_RESET;

    // Restore host speed detection in case the disconnected LS device
    USBHS->USBHS_HSTCTRL &= ~USBHS_HSTCTRL_SPDCONF_Msk;
    USBHS->USBHS_HSTCTRL |= USBHS_HSTCTRL_SPDCONF_NORMAL;

    USB_REG->HSTICR = HSTICR_DCONNIC;
    USB_REG->HSTIER = HSTIER_DCONNIES;

    status[rhport] = HCD_IDLE;

    hw_pipes_reset(rhport);

    hcd_event_device_remove(rhport, true);
    return true;
  }

  // Device connection
  if (((USB_REG->HSTISR) & HSTISR_DCONNI) && ((USB_REG->HSTIMR) & HSTIMR_DCONNIE))
  {
    events_idx = 0;
    memset((void*)(&events), 0, sizeof(events));

    //events[events_idx++] = 3;

    // Acknowledge connection interrupt
    USB_REG->HSTICR = HSTICR_DCONNIC;
    USB_REG->HSTIDR = HSTIDR_DCONNIEC;

    // // Enable SOF generation and interrupts
    USB_REG->HSTCTRL |= HSTCTRL_SOFE;
    USB_REG->HSTIER = HSTIER_HSOFIES;

    // Prepare for disconnection interrupt
    USB_REG->HSTICR = HSTICR_DDISCIC;
    USB_REG->HSTIER = HSTIER_DDISCIES;

    status[rhport] = HCD_ATTACHING;
    return true;
  }

  return false;
}

bool hcd_init(uint8_t rhport)
{
  (void)rhport;
  hcd_int_disable(rhport);

  // Set host mode
  USB_REG->CTRL &= ~(CTRL_UIMOD | CTRL_UID);

  // Freeze USB clock for now
  USB_REG->CTRL |= CTRL_FRZCLK;

  // Set USB to switch to high speed if necessary
  USBHS->USBHS_HSTCTRL &= ~USBHS_HSTCTRL_SPDCONF_Msk;
  USBHS->USBHS_HSTCTRL |= USBHS_HSTCTRL_SPDCONF_NORMAL;

  // // Force re-connection on initialization
  // USB_REG->HSTIFR |= HSTIMR_DDISCIE;

  // Enable USB
  USB_REG->CTRL = CTRL_USBE;

	// Clear all interrupts that may have been set by a previous host mode
  USBHS->USBHS_HSTICR = USBHS_HSTICR_DCONNIC | USBHS_HSTICR_DDISCIC
      | USBHS_HSTICR_HSOFIC  | USBHS_HSTICR_HWUPIC
      | USBHS_HSTICR_RSMEDIC | USBHS_HSTICR_RSTIC
      | USBHS_HSTICR_RXRSMIC;

  USB_REG->CTRL |= CTRL_VBUSHWC; // datasheet indicates must be set to 1
  USB_REG->SFR |= SFR_VBUSRQS;

  USB_REG->HSTIDR = HSTIDR_HSOFIEC; // interrupts not used, just count registers

  USB_REG->HSTIER = HSTIER_DCONNIES | HSTIER_RSTIES;
  status[rhport] = HCD_IDLE;
  return true;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  // Reset every pipe associated with the device
  for (uint8_t i = 0; i < EP_MAX; i++)
  {
    if (hw_pipe_enabled(rhport, i) && hw_pipe_get_dev_addr(rhport, i) == dev_addr)
    {
      hw_pipe_enable(rhport, i, false);
      // Dealloc if this pipe is last, or next pipe is not allocated. If next pipe
      // is allocated, do not deallocate this one as that will cause that pipe's FIFO
      // area to slide - which may lead to data loss.
      if (i == EP_MAX - 1 || !(USB_REG->HSTPIPCFG[i + 1] & HSTPIPCFG_ALLOC))
      {
        USB_REG->HSTPIPCFG[i] &= ~HSTPIPCFG_ALLOC;
      }
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

  //events[events_idx++] = 10;

  // Set pipe token to setup
  hw_pipe_set_token(rhport, pipe, HSTPIPCFG_PTOKEN_SETUP);
  // Clear setup token interrupt
  hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_CTRL_TXSTPIC);
  // Copy setup data to USB buffer
  const uint8_t *src = setup_packet;
  uint8_t *dst = PEP_GET_FIFO_PTR(pipe, 8);
  for (int i = 0; i < 8; i++)
  {
    *dst++ = *src++;
  }
  // Enable setup token sent interrupt
  hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_CTRL_TXSTPES);
  __DSB();
  __ISB();
  // Disable pipe freeze and control clear
  hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_FIFOCONC | HSTPIPIDR_PFREEZEC);
  return true;
}

static void hw_pipe_ctrl_xfer(uint8_t rhport, uint8_t pipe, bool in)
{
  hw_pipe_set_token(rhport, pipe, in ? HSTPIPCFG_PTOKEN_IN : HSTPIPCFG_PTOKEN_OUT); // control pipes are bi-directional, set correct token

  if (in)
  {
    //events[events_idx++] = 25;
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_RXINIC);
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_RXINES);
  }
  else
  {
    //events[events_idx++] = 26;
    hw_pipe_clear_reg(rhport, pipe, HSTPIPICR_TXOUTIC);
    hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_TXOUTES);
    hw_pipe_fifo_copy_out(rhport, pipe);
  }
  hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC | HSTPIPIDR_FIFOCONC);
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
  //events[events_idx++] = 4;
  // Enable reset sent interrupt
  USB_REG->HSTIER = HSTIER_RSTIES;
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
  return status[rhport] == HCD_CONNECTED;
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
  pipe = hw_pipe_find_free(rhport, ep_desc);
  if (pipe >= EP_MAX)
  {
    return false;
  }

  // Configure the pipe
  hw_pipe_reset(rhport, pipe); // reset the pipe

  // Prepare pipe configuration
  uint32_t cfg = 0;

  tusb_xfer_type_t type = ep_desc->bmAttributes.xfer;
  cfg |= (uint32_t)type << HSTPIPCFG_PTYPE_Pos;

  bool in = ep_desc->bEndpointAddress & TUSB_DIR_IN_MASK; // configure the token, for control pipes
  cfg |= (type == TUSB_XFER_CONTROL ? HSTPIPCFG_PTOKEN_SETUP : // set it to setup token; since it's
          in ? HSTPIPCFG_PTOKEN_IN : HSTPIPCFG_PTOKEN_OUT); // going to be set to this on setup anyways

  // ep_addr number, without dir bit
  cfg |= (HSTPIPCFG_PEPNUM & ((uint32_t)(ep_desc->bEndpointAddress) << HSTPIPCFG_PEPNUM_Pos));

  uint16_t size = ep_desc->wMaxPacketSize & (PIPE_MAX_PACKET_SIZE - 1); // mask with max packet size the
  cfg |= (HSTPIPCFG_PSIZE & ((uint32_t)hw_compute_psize(size) << HSTPIPCFG_PSIZE_Pos)); // hardware supports

  uint8_t interval = ep_desc->bInterval;

  if (hcd_port_speed_get(rhport) == TUSB_SPEED_HIGH && (type == TUSB_XFER_INTERRUPT || type == TUSB_XFER_ISOCHRONOUS))
  {
    if (interval > 16)
    {
      interval = 16;
    }
    uint16_t ms = 1 << (interval - 1);
    interval = ms > 0xFF ? 0xFF : ms;
  }

  cfg |= interval << HSTPIPCFG_INTFRQ_Pos;

  cfg |= HSTPIPCFG_PBK_1_BANK;
#if USE_DUAL_BANK
  if (type == TUSB_XFER_ISOCHRONOUS || type == TUSB_XFER_BULK)
  {
    cfg |= HSTPIPCFG_PBK_2_BANK;
  }
#endif

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
    USB_REG->HSTIER = (HSTISR_PEP_0 | (HSTISR_DMA_0 >> 1)) << pipe;

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
  pipe_xfers[pipe].done = 0;

  // Use DMA for non-ZLP out transfers on supported, non-control pipes.
  pipe_xfers[pipe].dma = (pipe_xfers[pipe].total || (ep_addr & TUSB_DIR_IN_MASK))
                         && EP_DMA_SUPPORT(pipe) &&
                         hw_pipe_get_type(rhport, pipe) != TUSB_XFER_CONTROL;

  if (pipe_xfers[pipe].dma)
  {
    uint16_t pipe_size = hw_pipe_get_size(rhport, pipe);
    uint32_t size_max  = (ep_addr & TUSB_DIR_IN_MASK) ? pipe_size * PIPINRQ_MAX : DMA_TRANS_MAX;

    if (pipe_xfers[pipe].total <= size_max) // TODO: implement support for very large buffers
    {
      USB_REG->HSTPIPCFG[pipe] |= HSTPIPCFG_AUTOSW;

      uint32_t dma_ctrl = USBHS_HSTDMACONTROL_BUFF_LENGTH(pipe_xfers[pipe].total);
      if (ep_addr & TUSB_DIR_IN_MASK)
      {
        hw_cache_invalidate_prepare(pipe_xfers[pipe].buffer, pipe_xfers[pipe].total);
        if (hw_pipe_get_type(rhport, pipe) != TUSB_XFER_ISOCHRONOUS ||
            pipe_xfers[pipe].total <= pipe_size)
        {
          // Enable short packet reception
          dma_ctrl |= HSTDMACONTROL_END_TR_IT | HSTDMACONTROL_END_TR_EN;
        }
      }
      else
      {
        hw_cache_flush(pipe_xfers[pipe].buffer, pipe_xfers[pipe].total);
        if (pipe_xfers[pipe].total % pipe_size != 0)
        {
          dma_ctrl |= HSTDMACONTROL_END_B_EN;
        }
      }

      uint8_t channel = pipe - 1;
      USB_REG->HSTDMA[channel].HSTDMAADDRESS = (uint32_t)(pipe_xfers[pipe].buffer);
      dma_ctrl |= HSTDMACONTROL_END_BUFFIT | HSTDMACONTROL_CHANN_ENB;

      uint32_t flags = 0;
      hw_enter_critical(&flags);
      if (!(USB_REG->HSTDMA[channel].HSTDMASTATUS & HSTDMASTATUS_END_TR_ST))
      {
        if (ep_addr & TUSB_DIR_IN_MASK)
        {
          USB_REG->HSTPIPINRQ[pipe] = HSTPIPINRQ_INRQ &
            ((((pipe_xfers[pipe].total + (pipe_size - 1)) / pipe_size) - 1) << HSTPIPINRQ_INRQ_Pos);
        }
        hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_NBUSYBKEC | HSTPIPIDR_PFREEZEC);
        USB_REG->HSTDMA[channel].HSTDMACONTROL = dma_ctrl;
        hw_exit_critical(&flags);
      }
      else
      {
        return false;
        hw_exit_critical(&flags);
      }
    }
    else
    {
      return false;
    }
  }
  else
  {
    bool in = ep_addr & TUSB_DIR_IN_MASK;

    if (hw_pipe_get_type(rhport, pipe) == TUSB_XFER_CONTROL)
    {
      //events[events_idx++] = 21;
      hw_pipe_ctrl_xfer(rhport, pipe, in);
    }
    else
    {
      //events[events_idx++] = 22;
      hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_PFREEZEC);
      if (ep_addr & TUSB_DIR_IN_MASK)
      {
        //events[events_idx++] = 23;
        USB_REG->HSTPIPINRQ[pipe] |= HSTPIPINRQ_INMODE;
        hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_RXINES);
      }
      else
      {
        //events[events_idx++] = 24;
        hw_pipe_enable_reg(rhport, pipe, HSTPIPIER_TXOUTES);
        hw_pipe_disable_reg(rhport, pipe, HSTPIPIDR_NBUSYBKEC);
      }
      USB_REG->HSTIER = (HSTISR_PEP_0) << pipe;
    }
  }

  return true;
}
void hcd_int_handler(uint8_t rhport)
{
  // Change to low power mode to only use the 48 MHz clock during low-speed
  if (hcd_port_speed_get(rhport) == TUSB_SPEED_LOW &&
      (!(USB_REG->HSTCTRL & USBHS_HSTCTRL_SPDCONF_Msk)))
  {
    USB_REG->HSTCTRL |= HSTCTRL_SPDCONF_LOW_POWER;
  }

  // Pipe processing & exception interrupts
  if (((USB_REG->HSTISR) & HSTISR_HSOFI) && ((USB_REG->HSTIMR) & HSTIMR_HSOFIE))
  {
    USB_REG->HSTICR = HSTICR_HSOFIC;

    // Do not handle micro-SOFs
    if((USB_REG->HSTFNUM & HSTFNUM_MFNUM) >> HSTFNUM_MFNUM_Pos)
    {
      return;
    }

    if (status[rhport] == HCD_ATTACHING && hcd_frame_number(rhport) > 10)
    {
      //events[events_idx++] = 5;
      USB_REG->HSTIDR = HSTIDR_HSOFIEC;
      status[rhport] =  HCD_RESETTING;
      hcd_event_device_attach(rhport, true);
    }
    return;
  }

  // Pipe processing & exception interrupts
  if ((USB_REG->HSTISR) & HSTISR_PEP_)
  {
    RET_IF_TRUE(hw_handle_pipe_int(rhport));
  }

  // DMA processing interrupts
  if ((USB_REG->HSTISR) & HSTISR_DMA_)
  {
    RET_IF_TRUE(hw_handle_dma_int(rhport));
  }

  // Host global (root hub) processing interrupts
  if ((USB_REG->HSTISR) & (HSTISR_RSTI | HSTISR_DCONNI | HSTISR_DDISCI))
  {
    RET_IF_TRUE(hw_handle_rh_int(rhport));
  }

  assert(false); // error condition
}

#endif
