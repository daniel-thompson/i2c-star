/*
 * Part of librfm3 (a utility library built on librfn and libopencm3)
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

#ifndef RF_I2C_CTX_H_
#define RF_I2C_CTX_H_

#include <stdbool.h>
#include <stdint.h>
#include <librfn/protothreads.h>

/*!
 * \defgroup librfm3_i2c_ctx I2C context manager
 *
 * \brief Protothreaded I2C driver for libopencm3.
 *
 * @{
 */

typedef struct i2c_ctx {
	pt_t pt;      //!< Protothread state for high-level functions
	pt_t leaf;    //!< Protothread state for low-level functions

	int err;      //!< Error reporting (becomes non-zero on error)
	bool verbose; //!< Automatically print error reports

	uint32_t i2c;		 //!< \private
	uint32_t timeout;	 //!< \private
	uint8_t bytes_remaining; //!< \private
	int i;			 //!< \private
} i2c_ctx_t;

typedef struct i2c_device_map {
	uint16_t devices[8]; //!< Bitmap recording detected devices
} i2c_device_map_t;

/*!
 * \brief Initialize the context structure ready for a single I2C transaction.
 *
 * The context must be reinitialized before each I2C transaction otherwise
 * the transaction may timeout immediately.
 */
void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c);

/*!
 * \brief Reset and reinitialize the I2C bus.
 */
void i2c_ctx_reset(i2c_ctx_t *c);

/*!
 * \brief Send a start condition.
 *
 * \note This is a low-level protothread; c->leaf must be zeroed by PT_SPAWN().
 */
pt_state_t i2c_ctx_start(i2c_ctx_t *c);

/*!
 * \brief Send the target bus address.
 *
 * \note This is a low-level protothread; c->leaf must be zeroed by PT_SPAWN().
 *
 * If bytes_to_read is zero then launch a write transaction, otherwise
 * launch a read and manage the ACK/NACK and STOP generation based on
 * the number of bytes to transfer.
 */
pt_state_t i2c_ctx_sendaddr(i2c_ctx_t *c, uint16_t addr, uint8_t bytes_to_read);

/*!
 * \brief Write a single byte.
 *
 * \note This is a low-level protothread; c->leaf must be zeroed by PT_SPAWN().
 */
pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t data);

/*!
 * \brief Read a single byte.
 *
 * \note This is a low-level protothread; c->leaf must be zeroed by PT_SPAWN().
 */
pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data);

/*!
 * \brief Send a stop condition.
 *
 * \note This is a low-level protothread; c->leaf must be zeroed by PT_SPAWN().
 *
 * Complete a low-level I2C write transaction. This is not required for
 * reads because the drivers have to request the stop prior to reading
 * the final bytes (so the stop is issued automatically).
 *
 */
pt_state_t i2c_ctx_stop(i2c_ctx_t *c);

/*!
 * \brief Detect attached I2C devices.
 *
 * \note This is a high-level protothread; c->pt must be zeroed by PT_SPAWN().
 */
pt_state_t i2c_ctx_detect(i2c_ctx_t *c, i2c_device_map_t *map);

/*!
 * \brief Write to an I2C register.
 *
 * \note This is a high-level protothread; c->pt must be zeroed by PT_SPAWN().
 *
 * \todo API leaves space for 16-bit addresses and registers but this
 *       is not implemented.
 */
pt_state_t i2c_ctx_setreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
			  uint8_t val);

/*!
 * \brief Read from an I2C register.
 *
 * \note This is a high-level protothread; c->pt must be zeroed by PT_SPAWN().
 *
 * \todo API leaves space for 16-bit addresses and registers but this
 *       is not implemented.
 */
pt_state_t i2c_ctx_getreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
			  uint8_t *val);

/*!
 * \brief Burst write to an I2C device.
 *
 * Primarily used for writes to memory devices or to initialize devices
 * whose registers can be written to continuously (without a stop/restart).
 *
 * \note This is a high-level protothread; c->pt must be zeroed by PT_SPAWN().
 */
pt_state_t i2c_ctx_write(i2c_ctx_t *c, uint16_t addr, uint8_t *data,
			 uint8_t len);

/*! @} */

#endif // RF_I2C_CTX_H_
