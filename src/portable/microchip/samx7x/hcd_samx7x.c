#include "host/hcd.h"

bool hcd_init(uint8_t rhport)
{
    return false;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
}

void hcd_int_enable(uint8_t rhport)
{
}

void hcd_int_disable(uint8_t rhport)
{
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
    return TUSB_SPEED_INVALID;
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
    return false;
}

void hcd_int_handler(uint8_t rhport)
{

}