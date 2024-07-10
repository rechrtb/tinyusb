 /*
* The MIT License (MIT)
*
* Copyright (c) 2019 Microchip Technology Inc.
* Copyright (c) 2018, hathach (tinyusb.org)
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
#ifndef _COMMON_USB_DEFS_H_
#define _COMMON_USB_DEFS_H_

#include <stdint.h>

#include "common/tusb_types.h"
#include "common/tusb_common.h"
#include "common_usb_regs.h"

#define PEP_GET_FIFO_PTR(ep, scale) (((TU_XSTRCAT(TU_STRCAT(uint, scale),_t) (*)[0x8000 / ((scale) / 8)])FIFO_RAM_ADDR)[(ep)])
#define PIPE_MAX_PACKET_SIZE 1024


TU_ATTR_ALWAYS_INLINE static inline void hw_enter_critical(volatile uint32_t *atomic)
{
	*atomic = __get_PRIMASK();
	__disable_irq();
	__DMB();
}

TU_ATTR_ALWAYS_INLINE static inline void hw_exit_critical(volatile uint32_t *atomic)
{
	__DMB();
	__set_PRIMASK(*atomic);
}


TU_ATTR_ALWAYS_INLINE static inline void hw_dcache_invalidate_prepare(uint8_t *addr, int32_t size)
{
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    int32_t op_size = size;
    uint32_t op_addr = (uint32_t) addr;
    uint32_t addr1 = op_addr;
    uint32_t addr2 = op_addr + op_size;
    if (addr1 & 0x1F || addr2 & 0x1F) {
      addr1 &= ~0x1F;
      addr2 &= ~0x1F;
      __ISB();
      __DSB();
      SCB->DCCIMVAC = addr1;
      SCB->DCCIMVAC = addr2;
      __DSB();
      __ISB();
    }
  }
  else
  {
    __DSB();
    __ISB();
  }
}

TU_ATTR_ALWAYS_INLINE static inline void hw_dcache_invalidate(uint8_t *addr, int32_t size)
{
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    int32_t op_size = size;
    uint32_t op_addr = (uint32_t)addr;
    int32_t linesize = 32U;                // in Cortex-M7 size of cache line is fixed to 8 words (32 bytes)
    if (op_addr & 0x1F) {
      op_size += op_addr & 0x1F;
      op_addr &= ~0x1F;
    }
    __ISB();
    __DSB();

    while (op_size > 0) {
      /* D-Cache Invalidate by MVA to PoC */
      (*(volatile uint32_t *)(0xE000EF5C)) = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    __DSB();
    __ISB();
  }
  else
  {
    __DSB();
    __ISB();
  }
}

TU_ATTR_ALWAYS_INLINE static inline void hw_dcache_flush(uint8_t *addr, int32_t size)
{
  if (SCB->CCR & SCB_CCR_DC_Msk)
  {
    int32_t op_size = size;
    uint32_t op_addr = (uint32_t) addr;
    int32_t linesize = 32U;                // in Cortex-M7 size of cache line is fixed to 8 words (32 bytes)
    if (op_addr & 0x1F) {
      op_size += op_addr & 0x1F;
      op_addr &= ~0x1F;
    }
    __ISB();
    __DSB();

    while (op_size > 0) {
      SCB->DCCMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    __DSB();
    __ISB();
  }
  else
  {
    __DSB();
    __ISB();
  }
}

TU_ATTR_ALWAYS_INLINE static inline tusb_speed_t hw_port_speed_get(void)
{
  switch (USB_REG->SR & SR_SPEED)
  {
  case SR_SPEED_FULL_SPEED:
  default:
    return TUSB_SPEED_FULL;
  case SR_SPEED_HIGH_SPEED:
    return TUSB_SPEED_HIGH;
  case SR_SPEED_LOW_SPEED:
    return TUSB_SPEED_LOW;
  };
}

#endif