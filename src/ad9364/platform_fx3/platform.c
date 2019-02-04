/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <string.h>
#include "stdint.h"
#include "../util.h"
#include "platform.h"
#include "parameters.h"

libusb_context *_ctx;
libusb_device_handle *_adsdr_handle;

void set_libusb_params(libusb_context *ctx, libusb_device_handle *adsdr_handle) {
	_ctx = ctx;
	_adsdr_handle = adsdr_handle;
}

/***************************************************************************//**
 * @brief spi_init
*******************************************************************************/
int32_t spi_init(uint32_t device_id,
				 uint8_t  clk_pha,
				 uint8_t  clk_pol)
{
	return 0;
}

void print_buf(const char *msg, unsigned char *buff, int len) {
	printf("%s: ", msg);
	for(int i = 0; i < len; i++) {
		printf("%X ", buff[i]);
	}
	printf("\n");
}
//
//char* buf_to_str(unsigned char *buff, int len) {
//    char* ret = malloc(sizeof(char) * 2048);
//    ret[0] = '\0';
//    for(int i = 0; i < len; i++) {
//        snprintf(ret + strlen(ret), 2048 - strlen(ret), "%02X ", buff[i]);
//    }
//    return ret;
//}

int txControlToDevice(uint8_t* src, uint32_t size8, uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
	if(_adsdr_handle != 0)
	{
//		printf("txControlToDevice\n");
		uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT;
		uint8_t bRequest = cmd;
		uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

		int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, src, size8, timeout_ms );
		if ( res != ( int ) size8 ) {
			fprintf( stderr, "FX3Dev::txControlToDevice() error %d %s\n", res, libusb_error_name(res) );
			return FX3_ERR_CTRL_TX_FAIL;
		}
		return FX3_ERR_OK;
	}

	return FX3_ERR_NO_DEVICE_FOUND;
}

int txControlFromDevice(uint8_t* dest, uint32_t size8 , uint8_t cmd, uint16_t wValue, uint16_t wIndex)
{
	if(_adsdr_handle != 0)
	{
//		printf("txControlFromDevice\n");
		uint8_t bmRequestType = LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN;
		uint8_t bRequest = cmd;
		uint32_t timeout_ms = DEV_UPLOAD_TIMEOUT_MS;

		int res = libusb_control_transfer(_adsdr_handle, bmRequestType, bRequest, wValue, wIndex, dest, size8, timeout_ms );
//		print_buf("dst", dest, size8);
		if ( res != ( int ) size8 ) {
			fprintf( stderr, "FX3Dev::transferDataFromDevice() error %d %s\n", res, libusb_error_name(res) );
			return FX3_ERR_CTRL_TX_FAIL;
		}
		return FX3_ERR_OK;
	}

	return FX3_ERR_NO_DEVICE_FOUND;
}

//int fx3_spi_write(uint16_t addr, uint8_t val) {
//    uint8_t buffer[3];
//    addr |= 0x8000;
//    buffer[0] = (uint8_t)(addr >> 8);
//    buffer[1] = (uint8_t)(addr & 0xFF);
//    buffer[2] = val;
//    return txControlToDevice(buffer, 3, REG_SPI_WRITE8, 0, 0);
//}
//
//int fx3_spi_read(uint16_t addr, uint8_t *val) {
//    uint8_t buffer[3];
//    buffer[0] = (uint8_t)(addr >> 8);
//    buffer[1] = (uint8_t)(addr & 0xFF);
//    buffer[2] = *val;
//    printf("r1 %x %x %x\n", buffer[0], buffer[1], buffer[2]);
//    int ret = txControlFromDevice(buffer, 3, REG_SPI_READ8, *val, addr);
//    *val = buffer[2];
//    printf("r2 %x %x %x\n", buffer[0], buffer[1], buffer[2]);
//    return ret;
//}

/***************************************************************************//**
 * @brief spi_write_then_read
*******************************************************************************/
//int spi_write_then_read(struct spi_device *spi,
//							const unsigned char *txbuf, unsigned n_tx,
//							unsigned char *rxbuf, unsigned n_rx)
//{
//	uint16_t cmd = (txbuf[0] << 8) | txbuf[1];
//    printf("%x\n", cmd);
//	bool write = (cmd >> 15 == 1);
//	uint16_t cnt = ((cmd >> 12) & 0x7) + 1;
//	uint16_t addr = cmd & 0x3FF;
//    int ret = 0;
//	for(int i = 0; i < cnt; i++) {
//        if (write) {
//            ret = fx3_spi_write(addr, txbuf[2 + i]);
//        } else {
//            ret = fx3_spi_read(addr, &rxbuf[i]);
//        }
//        if(ret != 0) {
//            break;
//        }
//        addr--;
//    }
//	return ret;
//}

int spi_write_then_read(struct spi_device *spi,
							const unsigned char *txbuf, unsigned n_tx,
							unsigned char *rxbuf, unsigned n_rx)
{
	uint8_t buff[32];
	memset(buff, 0, 32);
    memcpy(buff, txbuf, n_tx);
//    print_buf("send: ", buff, n_tx + n_rx);
	int ret = txControlToDevice(buff, 32, REG_SPI_WRITE_READ_MULTIPLE_STAGE1, n_tx + n_rx, 0);
	if(ret < 0) return ret;
	ret = txControlFromDevice(buff, 32, REG_SPI_WRITE_READ_MULTIPLE_STAGE2, n_tx + n_rx, 0);
//    print_buf("recv: ", buff, n_tx + n_rx);
	if(ret < 0) return ret;
	memcpy(rxbuf, buff + n_tx, n_rx);
	return 0;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
void gpio_init(uint32_t device_id)
{

}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
void gpio_direction(uint8_t pin, uint8_t direction)
{

}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
bool gpio_is_valid(int number)
{
	return (number == GPIO_RESET_PIN);
}

/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
void gpio_data(uint8_t pin, uint8_t data)
{

}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
void gpio_set_value(unsigned gpio, int value)
{
    if(gpio == GPIO_RESET_PIN && value == 0) {
        uint8_t tmp[3] = {0x00, 0x00, 0xFF};
        //TODO: reset
//        txControlToDevice(tmp, DEVICE_RESET, )
    }
}

/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
void udelay(unsigned long usecs)
{
	usleep(usecs);
}

/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
void mdelay(unsigned long msecs)
{
	usleep(msecs * 1000);
}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long msleep_interruptible(unsigned int msecs)
{

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void axiadc_init(struct ad9361_rf_phy *phy)
{

}

/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{

}

/***************************************************************************//**
* @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_idelay_set
*******************************************************************************/
void axiadc_idelay_set(struct axiadc_state *st,
				unsigned lane, unsigned val)
{

}
