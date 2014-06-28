/*
 * This file is part of the i2c-cm3-usb project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <librfn/console.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xff,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0403,
	.idProduct = 0xc631,
	.bcdDevice = 0x0205,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor i2c_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor i2c_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = &i2c_endpoint,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &i2c_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* ?automatically calculated? */
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"redfelineninja.org.uk",
	"i2c-cm3-usb",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag fo I2C_IO
#define CMD_I2C_END    2  // flag fo I2C_IO

/* linux kernel flags */
#define I2C_M_TEN		0x10	/* we have a ten bit chip address */
#define I2C_M_RD		0x01
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */ 
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000 
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000 
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000 
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000 
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000 
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000 
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000 
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000 
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000 
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000 
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
                            I2C_FUNC_SMBUS_BYTE | \
                            I2C_FUNC_SMBUS_BYTE_DATA | \
                            I2C_FUNC_SMBUS_WORD_DATA | \
                            I2C_FUNC_SMBUS_PROC_CALL | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
                            I2C_FUNC_SMBUS_I2C_BLOCK

/* the currently support capability is quite limited */
const unsigned long func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;


#define STATUS_IDLE	   0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NAK 2

static uint8_t status = STATUS_IDLE;


static int usb_control_request(usbd_device *usbd_dev,
			       struct usb_setup_data *req, uint8_t **buf,
			       uint16_t *len,
			       void (**complete)(usbd_device *usbd_dev,
						 struct usb_setup_data *req))
{
	static uint8_t reply_buf[4];

	(void)usbd_dev;
	(void)complete;

	/* optimistically set the reply buffer (because replies are
	 * manipulated by *len and if this is unmodified then the reply
	 * buffer is ignored.
	 */
	*buf = reply_buf;

	printf("%s\n", __FUNCTION__);

	switch (req->bRequest) {
	case CMD_ECHO:
		memcpy(reply_buf, &req->wValue, sizeof(req->wValue));
		*len = sizeof(req->wValue);
		return USBD_REQ_HANDLED;
	
	case CMD_GET_FUNC:
		/* Report our capabilities */
		memcpy(reply_buf, &func, sizeof(func));
		*len = sizeof(func);
		return USBD_REQ_HANDLED;

	case CMD_SET_DELAY:
		/* This was used in i2c-tiny-usb to choose the clock
		 * frequency by specifying the shortest time between
		 * clock edges.
		 *
		 * This implementation silently ignores delay requests.
		 */
		printf("CMD_SET_DELAY\n");
		*len = 0;
		return USBD_REQ_HANDLED;

	case CMD_I2C_IO:
	case CMD_I2C_IO | CMD_I2C_BEGIN:
	case CMD_I2C_IO | CMD_I2C_END:
	case CMD_I2C_IO | CMD_I2C_BEGIN | CMD_I2C_END:
		break;

	case CMD_GET_STATUS:
		memcpy(reply_buf, &status, sizeof(status));
		*len = sizeof(status);
		return USBD_REQ_HANDLED;
		
	default:
		break;
		
	}

	return USBD_REQ_NEXT_CALLBACK;
}

static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usb_control_request);
}

static usbd_device *usbd_dev;

static int usb_fibre(fibre_t *fibre)
{
	PT_BEGIN_FIBRE(fibre);

	usbd_dev = usbd_init(&stm32f103_usb_driver, &dev, &config,
			usb_strings, 2,
			usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config);

	/* assert hotplug */
	gpio_set(GPIOB, GPIO8);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	while (true) {
		usbd_poll(usbd_dev);
		PT_YIELD();
	}

	PT_END();
}
static fibre_t usb_task = FIBRE_VAR_INIT(usb_fibre);
static console_t uart_console;

int main(void)
{
	int i;

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	console_init(&uart_console, stdout);
	time_init();

	printf("Booted OK\n");

	for (i = 0; i < 0x800000; i++)
		__asm__("nop");

	/* prepare the scheduler */
	fibre_run(&usb_task);

	fibre_scheduler_main_loop();
	return 0;
}
