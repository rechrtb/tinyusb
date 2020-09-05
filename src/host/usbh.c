/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "common/tusb_common.h"

#if TUSB_OPT_HOST_ENABLED

#ifndef CFG_TUH_TASK_QUEUE_SZ
#define CFG_TUH_TASK_QUEUE_SZ   16
#endif

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "tusb.h"
#include "hub.h"
#include "usbh_hcd.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= 2
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif

static host_class_driver_t const usbh_class_drivers[] =
{
  #if CFG_TUH_CDC
    {
      DRIVER_NAME("CDC")
      .class_code = TUSB_CLASS_CDC,
      .init       = cdch_init,
      .open       = cdch_open,
      .xfer_cb    = cdch_xfer_cb,
      .close      = cdch_close
    },
  #endif

  #if CFG_TUH_MSC
    {
      DRIVER_NAME("MSC")
      .class_code = TUSB_CLASS_MSC,
      .init       = msch_init,
      .open       = msch_open,
      .xfer_cb    = msch_xfer_cb,
      .close      = msch_close
    },
  #endif

  #if HOST_CLASS_HID
    {
      DRIVER_NAME("HID")
      .class_code = TUSB_CLASS_HID,
      .init       = hidh_init,
      .open       = hidh_open_subtask,
      .xfer_cb    = hidh_xfer_cb,
      .close      = hidh_close
    },
  #endif

  #if CFG_TUH_HUB
    {
      DRIVER_NAME("HUB")
      .class_code = TUSB_CLASS_HUB,
      .init       = hub_init,
      .open       = hub_open,
      .xfer_cb    = hub_xfer_cb,
      .close      = hub_close
    },
  #endif

  #if CFG_TUH_VENDOR
    {
      DRIVER_NAME("VENDOR")
      .class_code = TUSB_CLASS_VENDOR_SPECIFIC,
      .init       = cush_init,
      .open       = cush_open_subtask,
      .xfer_cb    = cush_isr,
      .close      = cush_close
    }
  #endif
};

enum { USBH_CLASS_DRIVER_COUNT = TU_ARRAY_SIZE(usbh_class_drivers) };

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+

// including zero-address
CFG_TUSB_MEM_SECTION usbh_device_t _usbh_devices[CFG_TUSB_HOST_DEVICE_MAX+1];

// Event queue
// role device/host is used by OS NONE for mutex (disable usb isr)
OSAL_QUEUE_DEF(OPT_MODE_HOST, _usbh_qdef, CFG_TUH_TASK_QUEUE_SZ, hcd_event_t);
static osal_queue_t _usbh_q;

CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4) static uint8_t _usbh_ctrl_buf[CFG_TUSB_HOST_ENUM_BUFFER_SIZE];

//------------- Helper Function Prototypes -------------//
static inline uint8_t get_new_address(void);

//--------------------------------------------------------------------+
// PUBLIC API (Parameter Verification is required)
//--------------------------------------------------------------------+
tusb_device_state_t tuh_device_get_state (uint8_t const dev_addr)
{
  TU_ASSERT( dev_addr <= CFG_TUSB_HOST_DEVICE_MAX, TUSB_DEVICE_STATE_UNPLUG);
  return (tusb_device_state_t) _usbh_devices[dev_addr].state;
}

void osal_task_delay(uint32_t msec)
{
  (void) msec;

  const uint32_t start = hcd_frame_number(TUH_OPT_RHPORT);
  while ( ( hcd_frame_number(TUH_OPT_RHPORT) - start ) < msec ) {}
}

//--------------------------------------------------------------------+
// CLASS-USBD API (don't require to verify parameters)
//--------------------------------------------------------------------+
bool tuh_init(void)
{
  tu_memclr(_usbh_devices, sizeof(usbh_device_t)*(CFG_TUSB_HOST_DEVICE_MAX+1));

  //------------- Enumeration & Reporter Task init -------------//
  _usbh_q = osal_queue_create( &_usbh_qdef );
  TU_ASSERT(_usbh_q != NULL);

  //------------- Semaphore, Mutex for Control Pipe -------------//
  for(uint8_t i=0; i<CFG_TUSB_HOST_DEVICE_MAX+1; i++) // including address zero
  {
    usbh_device_t * const dev = &_usbh_devices[i];

    dev->control.sem_hdl = osal_semaphore_create(&dev->control.sem_def);
    TU_ASSERT(dev->control.sem_hdl != NULL);

    memset(dev->itf2drv, 0xff, sizeof(dev->itf2drv)); // invalid mapping
    memset(dev->ep2drv , 0xff, sizeof(dev->ep2drv )); // invalid mapping
  }

  // Class drivers init
  for (uint8_t drv_id = 0; drv_id < USBH_CLASS_DRIVER_COUNT; drv_id++)
  {
    TU_LOG2("%s init\r\n", usbh_class_drivers[drv_id].name);
    usbh_class_drivers[drv_id].init();
  }

  TU_ASSERT(hcd_init());
  hcd_int_enable(TUH_OPT_RHPORT);

  return true;
}

//------------- USBH control transfer -------------//
bool usbh_control_xfer (uint8_t dev_addr, tusb_control_request_t* request, uint8_t* data)
{
  usbh_device_t* dev = &_usbh_devices[dev_addr];
  const uint8_t rhport = dev->rhport;

  dev->control.request = *request;
  dev->control.pipe_status = 0;

  // Setup Stage
  hcd_setup_send(rhport, dev_addr, (uint8_t*) &dev->control.request);
  TU_VERIFY(osal_semaphore_wait(dev->control.sem_hdl, OSAL_TIMEOUT_NORMAL));

  // Data stage : first data toggle is always 1
  if ( request->wLength )
  {
    hcd_edpt_xfer(rhport, dev_addr, tu_edpt_addr(0, request->bmRequestType_bit.direction), data, request->wLength);
    TU_VERIFY(osal_semaphore_wait(dev->control.sem_hdl, OSAL_TIMEOUT_NORMAL));
  }

  // Status : data toggle is always 1
  hcd_edpt_xfer(rhport, dev_addr, tu_edpt_addr(0, 1-request->bmRequestType_bit.direction), NULL, 0);
  TU_VERIFY(osal_semaphore_wait(dev->control.sem_hdl, OSAL_TIMEOUT_NORMAL));

  if ( XFER_RESULT_STALLED == dev->control.pipe_status ) return false;
  if ( XFER_RESULT_FAILED == dev->control.pipe_status ) return false;

  return true;
}

bool usbh_pipe_control_open(uint8_t dev_addr, uint8_t max_packet_size)
{
  osal_semaphore_reset( _usbh_devices[dev_addr].control.sem_hdl );
  //osal_mutex_reset( usbh_devices[dev_addr].control.mutex_hdl );
      
  tusb_desc_endpoint_t ep0_desc =
  {
    .bLength          = sizeof(tusb_desc_endpoint_t),
    .bDescriptorType  = TUSB_DESC_ENDPOINT,
    .bEndpointAddress = 0,
    .bmAttributes     = { .xfer = TUSB_XFER_CONTROL },
    .wMaxPacketSize   = { .size = max_packet_size },
    .bInterval        = 0
  };

  return hcd_edpt_open(_usbh_devices[dev_addr].rhport, dev_addr, &ep0_desc);
}

bool usbh_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
  bool ret = hcd_edpt_open(rhport, dev_addr, ep_desc);

  if (ret)
  {
    usbh_device_t* dev = &_usbh_devices[dev_addr];

    // new endpoints belongs to latest interface (last valid value)
    uint8_t drvid = 0xff;
    for(uint8_t i=0; i < sizeof(dev->itf2drv); i++)
    {
      if ( dev->itf2drv[i] == 0xff ) break;
      drvid = dev->itf2drv[i];
    }
    TU_ASSERT(drvid < USBH_CLASS_DRIVER_COUNT);

    uint8_t const ep_addr = ep_desc->bEndpointAddress;
    dev->ep2drv[tu_edpt_number(ep_addr)][tu_edpt_dir(ep_addr)] = drvid;
  }

  return ret;
}

//--------------------------------------------------------------------+
// HCD Event Handler
//--------------------------------------------------------------------+

void hcd_event_handler(hcd_event_t const* event, bool in_isr)
{
  switch (event->event_id)
  {
    default:
      osal_queue_send(_usbh_q, event, in_isr);
    break;
  }
}

// interrupt caused by a TD (with IOC=1) in pipe of class class_code
void hcd_event_xfer_complete(uint8_t dev_addr, uint8_t ep_addr, uint32_t xferred_bytes, xfer_result_t result, bool in_isr)
{
  usbh_device_t* dev = &_usbh_devices[ dev_addr ];

  if (0 == tu_edpt_number(ep_addr))
  {
    dev->control.pipe_status = result;
//    usbh_devices[ pipe_hdl.dev_addr ].control.xferred_bytes = xferred_bytes; not yet neccessary
    osal_semaphore_post( dev->control.sem_hdl, true ); // FIXME post within ISR
  }
  else
  {
    hcd_event_t event =
    {
      .rhport = 0,
      .event_id = HCD_EVENT_XFER_COMPLETE,
      .dev_addr = dev_addr,
      .xfer_complete =
      {
        .ep_addr = ep_addr,
        .result = result,
        .len = xferred_bytes
      }
    };

    hcd_event_handler(&event, in_isr);
  }
}

void hcd_event_device_attach(uint8_t rhport, bool in_isr)
{
  hcd_event_t event =
  {
    .rhport = rhport,
    .event_id = HCD_EVENT_DEVICE_ATTACH
  };

  event.connection.hub_addr = 0;
  event.connection.hub_port = 0;

  hcd_event_handler(&event, in_isr);
}

void hcd_event_device_remove(uint8_t hostid, bool in_isr)
{
  hcd_event_t event =
  {
    .rhport = hostid,
    .event_id = HCD_EVENT_DEVICE_REMOVE
  };

  event.connection.hub_addr = 0;
  event.connection.hub_port = 0;

  hcd_event_handler(&event, in_isr);
}


// a device unplugged on hostid, hub_addr, hub_port
// return true if found and unmounted device, false if cannot find
static void usbh_device_unplugged(uint8_t rhport, uint8_t hub_addr, uint8_t hub_port)
{
  //------------- find the all devices (star-network) under port that is unplugged -------------//
  for (uint8_t dev_addr = 0; dev_addr <= CFG_TUSB_HOST_DEVICE_MAX; dev_addr ++)
  {
    usbh_device_t* dev = &_usbh_devices[dev_addr];

    // TODO Hub multiple level
    if (dev->rhport == rhport   &&
        (hub_addr == 0 || dev->hub_addr == hub_addr) && // hub_addr == 0 & hub_port == 0 means roothub
        (hub_port == 0 || dev->hub_port == hub_port) &&
        dev->state    != TUSB_DEVICE_STATE_UNPLUG)
    {
      // Invoke callback before close driver
      if (tuh_umount_cb) tuh_umount_cb(dev_addr);

      // Close class driver
      for (uint8_t drv_id = 0; drv_id < USBH_CLASS_DRIVER_COUNT; drv_id++)
      {
        TU_LOG2("%s close\r\n", usbh_class_drivers[drv_id].name);
        usbh_class_drivers[drv_id].close(dev_addr);
      }

      memset(dev->itf2drv, 0xff, sizeof(dev->itf2drv)); // invalid mapping
      memset(dev->ep2drv , 0xff, sizeof(dev->ep2drv )); // invalid mapping

      hcd_device_close(rhport, dev_addr);

      dev->state = TUSB_DEVICE_STATE_UNPLUG;
    }
  }
}

//--------------------------------------------------------------------+
// ENUMERATION TASK
//--------------------------------------------------------------------+
static bool parse_configuration_descriptor(uint8_t dev_addr, tusb_desc_configuration_t const* desc_cfg)
{
  usbh_device_t* dev = &_usbh_devices[dev_addr];
  uint8_t const* p_desc = (uint8_t const*) desc_cfg;
  p_desc = tu_desc_next(p_desc);

  TU_LOG2_MEM(desc_cfg, desc_cfg->wTotalLength, 0);

  // parse each interfaces
  while( p_desc < _usbh_ctrl_buf + desc_cfg->wTotalLength )
  {
    // skip until we see interface descriptor
    if ( TUSB_DESC_INTERFACE != tu_desc_type(p_desc) )
    {
      p_desc = tu_desc_next(p_desc); // skip the descriptor, increase by the descriptor's length
    }else
    {
      tusb_desc_interface_t const* desc_itf = (tusb_desc_interface_t const*) p_desc;

      // Check if class is supported
      uint8_t drv_id;
      for (drv_id = 0; drv_id < USBH_CLASS_DRIVER_COUNT; drv_id++)
      {
        if ( usbh_class_drivers[drv_id].class_code == desc_itf->bInterfaceClass ) break;
      }

      if( drv_id >= USBH_CLASS_DRIVER_COUNT )
      {
        // skip unsupported class
        p_desc = tu_desc_next(p_desc);
      }
      else
      {
        // Interface number must not be used already TODO alternate interface
        TU_ASSERT( dev->itf2drv[desc_itf->bInterfaceNumber] == 0xff );
        dev->itf2drv[desc_itf->bInterfaceNumber] = drv_id;

        if (desc_itf->bInterfaceClass == TUSB_CLASS_HUB && dev->hub_addr != 0)
        {
          // TODO Attach hub to Hub is not currently supported
          // skip this interface
          p_desc = tu_desc_next(p_desc);
        }
        else
        {
          uint16_t itf_len = 0;

          TU_LOG2("%s open\r\n", usbh_class_drivers[drv_id].name);
          TU_ASSERT( usbh_class_drivers[drv_id].open(dev->rhport, dev_addr, desc_itf, &itf_len) );
          TU_ASSERT( itf_len >= sizeof(tusb_desc_interface_t) );
          p_desc += itf_len;
        }
      }
    }
  }

  return true;
}

bool enum_task(hcd_event_t* event)
{
  enum {
    POWER_STABLE_DELAY = 100,
    RESET_DELAY        = 500, // 200 USB specs say only 50ms but many devices require much longer
  };

  usbh_device_t* dev0 = &_usbh_devices[0];
  tusb_control_request_t request;

  dev0->rhport   = event->rhport; // TODO refractor integrate to device_pool
  dev0->hub_addr = event->connection.hub_addr;
  dev0->hub_port = event->connection.hub_port;
  dev0->state    = TUSB_DEVICE_STATE_UNPLUG;

  //------------- connected/disconnected directly with roothub -------------//
  if ( dev0->hub_addr == 0)
  {
    // wait until device is stable. Increase this if the first 8 bytes is failed to get
    osal_task_delay(POWER_STABLE_DELAY);

    // device unplugged while delaying
    if ( !hcd_port_connect_status(dev0->rhport) ) return true;

    hcd_port_reset( dev0->rhport ); // port must be reset to have correct speed operation
    osal_task_delay(RESET_DELAY);

    dev0->speed = hcd_port_speed_get( dev0->rhport );
  }
#if CFG_TUH_HUB
  //------------- connected/disconnected via hub -------------//
  else
  {
    // TODO wait for PORT reset change instead
    osal_task_delay(POWER_STABLE_DELAY);

    hub_port_status_response_t port_status;
    TU_VERIFY_HDLR( hub_port_get_status(dev0->hub_addr, dev0->hub_port, &port_status), hub_status_pipe_queue( dev0->hub_addr) );

    // device unplugged while delaying
    if ( !port_status.status.connection ) return true;

    dev0->speed = (port_status.status.high_speed) ? TUSB_SPEED_HIGH :
                  (port_status.status.low_speed ) ? TUSB_SPEED_LOW  : TUSB_SPEED_FULL;

    // Acknowledge Port Reset Change
    if (port_status.change.reset)
    {
      hub_port_clear_feature(dev0->hub_addr, dev0->hub_port, HUB_FEATURE_PORT_RESET_CHANGE);
    }
  }
#endif // CFG_TUH_HUB

  TU_ASSERT( usbh_pipe_control_open(0, 8) );

  //------------- Get first 8 bytes of device descriptor to get Control Endpoint Size -------------//
  TU_LOG2("Get 8 byte of Device Descriptor\r\n");
  request = (tusb_control_request_t ) {
        .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_IN },
        .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_DEVICE << 8,
        .wIndex = 0,
        .wLength = 8
  };
  bool is_ok = usbh_control_xfer(0, &request, _usbh_ctrl_buf);

  //------------- Reset device again before Set Address -------------//
  TU_LOG2("Port reset \r\n");

  if (dev0->hub_addr == 0)
  {
    TU_ASSERT(is_ok);

    // connected directly to roothub
    hcd_port_reset( dev0->rhport ); // reset port after 8 byte descriptor
    osal_task_delay(RESET_DELAY);
  }
#if CFG_TUH_HUB
  else
  {
    // connected via a hub
    TU_VERIFY_HDLR(is_ok, hub_status_pipe_queue( dev0->hub_addr) ); // TODO hub refractor

    if ( hub_port_reset(dev0->hub_addr, dev0->hub_port) )
    {
      osal_task_delay(RESET_DELAY);

      // Acknowledge Port Reset Change if Reset Successful
      hub_port_clear_feature(dev0->hub_addr, dev0->hub_port, HUB_FEATURE_PORT_RESET_CHANGE);
    }

    (void) hub_status_pipe_queue( dev0->hub_addr ); // done with hub, waiting for next data on status pipe
  }
#endif // CFG_TUH_HUB

  //------------- Set new address -------------//
  TU_LOG2("Set Address \r\n");
  uint8_t const new_addr = get_new_address();
  TU_ASSERT(new_addr <= CFG_TUSB_HOST_DEVICE_MAX); // TODO notify application we reach max devices

  request = (tusb_control_request_t ) {
        .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_OUT },
        .bRequest = TUSB_REQ_SET_ADDRESS,
        .wValue = new_addr,
        .wIndex = 0,
        .wLength = 0
  };
  TU_ASSERT(usbh_control_xfer(0, &request, NULL));

  //------------- update port info & close control pipe of addr0 -------------//
  usbh_device_t* new_dev = &_usbh_devices[new_addr];
  new_dev->rhport  = dev0->rhport;
  new_dev->hub_addr = dev0->hub_addr;
  new_dev->hub_port = dev0->hub_port;
  new_dev->speed    = dev0->speed;

  hcd_device_close(dev0->rhport, 0); // close device 0
  dev0->state = TUSB_DEVICE_STATE_UNPLUG;

  // open control pipe for new address
  TU_ASSERT ( usbh_pipe_control_open(new_addr, ((tusb_desc_device_t*) _usbh_ctrl_buf)->bMaxPacketSize0 ) );

  //------------- Get full device descriptor -------------//
  TU_LOG2("Get Device Descriptor \r\n");
  request = (tusb_control_request_t ) {
        .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_IN },
        .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_DEVICE << 8,
        .wIndex = 0,
        .wLength = 18
  };
  TU_ASSERT(usbh_control_xfer(new_addr, &request, _usbh_ctrl_buf));

  // update device info  TODO alignment issue
  tusb_desc_device_t const * desc_device = (tusb_desc_device_t const*) _usbh_ctrl_buf;

  if (tuh_attach_cb) tuh_attach_cb((tusb_desc_device_t*) _usbh_ctrl_buf);

  new_dev->vendor_id  = desc_device->idVendor;
  new_dev->product_id = desc_device->idProduct;
  TU_ASSERT(desc_device->bNumConfigurations > 0);

  enum { CONFIG_NUM = 1 }; // default to use configuration 1

  //------------- Get 9 bytes of configuration descriptor -------------//
  TU_LOG2("Get 9 bytes of Configuration Descriptor\r\n");
  request = (tusb_control_request_t ) {
        .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_IN },
        .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = (TUSB_DESC_CONFIGURATION << 8) | (CONFIG_NUM - 1),
        .wIndex = 0,
        .wLength = 9
  };
  TU_ASSERT( usbh_control_xfer(new_addr, &request, _usbh_ctrl_buf));

  // TODO not enough buffer to hold configuration descriptor
  TU_ASSERT( CFG_TUSB_HOST_ENUM_BUFFER_SIZE >= ((tusb_desc_configuration_t*)_usbh_ctrl_buf)->wTotalLength );

  //------------- Get full configuration descriptor -------------//
  TU_LOG2("Get full Configuration Descriptor\r\n");
  request.wLength = ((tusb_desc_configuration_t*)_usbh_ctrl_buf)->wTotalLength; // full length
  TU_ASSERT( usbh_control_xfer( new_addr, &request, _usbh_ctrl_buf ) );

  // update configuration info
  new_dev->interface_count = ((tusb_desc_configuration_t*) _usbh_ctrl_buf)->bNumInterfaces;

  //------------- Set Configure -------------//
  TU_LOG2("Set Configuration Descriptor\r\n");
  request = (tusb_control_request_t ) {
        .bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_DEVICE, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_OUT },
        .bRequest = TUSB_REQ_SET_CONFIGURATION,
        .wValue = CONFIG_NUM,
        .wIndex = 0,
        .wLength = 0
  };
  TU_ASSERT(usbh_control_xfer( new_addr, &request, NULL ));

  TU_LOG2("Device configured\r\n");
  new_dev->state = TUSB_DEVICE_STATE_CONFIGURED;

  //------------- TODO Get String Descriptors -------------//

  // Parse configuration & set up drivers
  parse_configuration_descriptor(new_addr, (tusb_desc_configuration_t*) _usbh_ctrl_buf);

  // Invoke callback if available
  if (tuh_mount_cb) tuh_mount_cb(new_addr);

  return true;
}

/* USB Host Driver task
 * This top level thread manages all host controller event and delegates events to class-specific drivers.
 * This should be called periodically within the mainloop or rtos thread.
 *_usbh_devices[dev_addr].
   @code
    int main(void)
    {
      application_init();
      tusb_init();

      while(1) // the mainloop
      {
        application_code();

        tuh_task(); // tinyusb host task
      }
    }
    @endcode
 */
void tuh_task(void)
{
  // Skip if stack is not initialized
  if ( !tusb_inited() ) return;

  // Loop until there is no more events in the queue
  while (1)
  {
    hcd_event_t event;
    if ( !osal_queue_receive(_usbh_q, &event) ) return;

    switch (event.event_id)
    {
      case HCD_EVENT_DEVICE_ATTACH:
        TU_LOG2("USBH DEVICE ATTACH\r\n");
        enum_task(&event);
      break;

      case HCD_EVENT_DEVICE_REMOVE:
        TU_LOG2("USBH DEVICE REMOVED\r\n");
        usbh_device_unplugged(event.rhport, event.connection.hub_addr, event.connection.hub_port);

        #if CFG_TUH_HUB
        // TODO remove
        if ( event.connection.hub_addr != 0)
        {
          // done with hub, waiting for next data on status pipe
          (void) hub_status_pipe_queue( event.connection.hub_addr );
        }
        #endif
      break;

      case HCD_EVENT_XFER_COMPLETE:
      {
        usbh_device_t* dev = &_usbh_devices[event.dev_addr];
        uint8_t const ep_addr = event.xfer_complete.ep_addr;
        uint8_t const epnum   = tu_edpt_number(ep_addr);
        uint8_t const ep_dir  = tu_edpt_dir(ep_addr);

        TU_LOG2("on EP %02X with %u bytes\r\n", ep_addr, (unsigned int) event.xfer_complete.len);

        if ( 0 == epnum )
        {
          // TODO control transfer
        }else
        {
          uint8_t drv_id = dev->ep2drv[epnum][ep_dir];
          TU_ASSERT(drv_id < USBH_CLASS_DRIVER_COUNT, );

          TU_LOG2("%s xfer callback\r\n", usbh_class_drivers[drv_id].name);
          usbh_class_drivers[drv_id].xfer_cb(event.dev_addr, ep_addr, event.xfer_complete.result, event.xfer_complete.len);
        }
      }
      break;

      default: break;
    }
  }
}

//--------------------------------------------------------------------+
// INTERNAL HELPER
//--------------------------------------------------------------------+
static inline uint8_t get_new_address(void)
{
  for (uint8_t addr=1; addr <= CFG_TUSB_HOST_DEVICE_MAX; addr++)
  {
    if (_usbh_devices[addr].state == TUSB_DEVICE_STATE_UNPLUG) return addr;
  }
  return CFG_TUSB_HOST_DEVICE_MAX+1;
}

#endif
