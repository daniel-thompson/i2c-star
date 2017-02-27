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

#include <librfm3/i2c_ctx.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <librfn/regdump.h>
#include <librfn/time.h>
#include <librfn/util.h>

#define D(x) { #x, x }
static const regdump_desc_t i2c_sr1_desc[] = { { "I2C_SR1", 0 },
					       D(I2C_SR1_SMBALERT),
					       D(I2C_SR1_TIMEOUT),
					       D(I2C_SR1_PECERR),
					       D(I2C_SR1_OVR),
					       D(I2C_SR1_AF),
					       D(I2C_SR1_ARLO),
					       D(I2C_SR1_BERR),
					       D(I2C_SR1_TxE),
					       D(I2C_SR1_RxNE),
					       D(I2C_SR1_STOPF),
					       D(I2C_SR1_ADD10),
					       D(I2C_SR1_BTF),
					       D(I2C_SR1_ADDR),
					       D(I2C_SR1_SB),
					       { NULL, 0 } };
#undef D

static bool i2c_ctx_is_timed_out(i2c_ctx_t *c)
{
	if (cyclecmp32(time_now(), c->timeout) > 0) {
		if (c->verbose) {
			printf("I2C TRANSACTION TIMED OUT\n");
			regdump(I2C_SR1(c->i2c), i2c_sr1_desc);
		}

		return true;
	}

	return false;
}

void i2c_ctx_init(i2c_ctx_t *c, uint32_t pi2c)
{
	memset(c, 0, sizeof(*c));

	c->i2c = pi2c;
	c->timeout = time_now() + 100000;
}

void i2c_ctx_reset(i2c_ctx_t *c)
{
	/* TODO: Latest opencm3 has this code built in */
	switch (c->i2c) {
	case I2C1:
		rcc_periph_reset_pulse(RST_I2C1);
		rcc_periph_reset_pulse(RST_I2C1);
		break;
	case I2C2:
		rcc_periph_reset_pulse(RST_I2C2);
		rcc_periph_reset_pulse(RST_I2C2);
		break;
#ifdef STM32F4
	case I2C3:
		rcc_periph_reset_pulse(RST_I2C3);
		rcc_periph_reset_pulse(RST_I2C3);
		break;
#endif
	}

	/* freq's numeric value ends up in MHz (i.e. in this case, 30) */
#ifdef STM32F4
	uint16_t freq = I2C_CR2_FREQ_30MHZ;
#else
	uint16_t freq = 36;
#endif

	/* CCR is the number of APB bus cycles in *half* an I2C bus
	 * cycle. For Sm (100Khz) this ends up as:
	 *   freq * 1MHz / 2 * 100KHz
	 *   freq * 1000000 / 200000
	 *   freq * 5
	 *
	 * Similar trise is the number of APB bus cycles in the rise
	 * time (plus 1). For Sm (1us) this ends up as:
	 *   freq * 1Mhz / (1/1us)  + 1
	 *   freq * 1MHz / 1MHz  + 1
	 *   freq + 1
	 */

	/* peripheral configuration */
	i2c_peripheral_disable(c->i2c);
	i2c_set_clock_frequency(c->i2c, freq);
	i2c_set_ccr(c->i2c, freq * 5);
	i2c_set_trise(c->i2c, freq + 1);
	i2c_set_own_7bit_slave_address(c->i2c, 0x32);
	i2c_peripheral_enable(c->i2c);
}

pt_state_t i2c_ctx_start(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	i2c_send_start(c->i2c);

	while (!i2c_ctx_is_timed_out(c) &&
	       !((I2C_SR1(c->i2c) & I2C_SR1_SB) &
		 (I2C_SR2(c->i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
		PT_YIELD();

	if (!I2C_SR1(c->i2c) & I2C_SR1_SB) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	PT_END();
}

pt_state_t i2c_ctx_sendaddr(i2c_ctx_t *c, uint16_t addr,
				   uint8_t bytes_to_read)
{
	PT_BEGIN(&c->leaf);

	c->bytes_remaining = bytes_to_read;

	i2c_send_7bit_address(c->i2c, addr, !!bytes_to_read);

	while (!i2c_ctx_is_timed_out(c) &&		// bad
	       !(I2C_SR1(c->i2c) & I2C_SR1_AF) &&	// bad
	       !(I2C_SR1(c->i2c) & I2C_SR1_ADDR))	// good
		PT_YIELD();

	if (!(I2C_SR1(c->i2c) & I2C_SR1_ADDR)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	/* If we are only reading one byte we must get ready to NACK the
	 * final byte.
	 */
	if (c->bytes_remaining == 1)
		I2C_CR1(c->i2c) &= ~I2C_CR1_ACK;
	else if (c->bytes_remaining >= 2)
		I2C_CR1(c->i2c) |= I2C_CR1_ACK;

	/* Read sequence has side effect or clearing I2C_SR1_ADDR */
	uint32_t reg32 __attribute__((unused));
	reg32 = I2C_SR2(c->i2c);

	if (c->bytes_remaining == 1)
		i2c_send_stop(c->i2c);

	PT_END();
}

pt_state_t i2c_ctx_senddata(i2c_ctx_t *c, uint8_t data)
{
	PT_BEGIN(&c->leaf);

	i2c_send_data(c->i2c, data);

	while (!i2c_ctx_is_timed_out(c) && !(I2C_SR1(c->i2c) & I2C_SR1_BTF))
		PT_YIELD();

	if (!(I2C_SR1(c->i2c) & I2C_SR1_BTF)) {
		i2c_ctx_reset(c);
		c->err = EIO;
	}

	PT_END();
}

pt_state_t i2c_ctx_getdata(i2c_ctx_t *c, uint8_t *data)
{
	PT_BEGIN(&c->leaf);

	while (!i2c_ctx_is_timed_out(c) && !(I2C_SR1(c->i2c) & I2C_SR1_RxNE))
		PT_YIELD();

	if (!(I2C_SR1(c->i2c) & I2C_SR1_RxNE)) {
		i2c_ctx_reset(c);
		c->err = EIO;
		PT_EXIT();
	}

	/* Need to NACK the final byte and STOP the transaction */
	if (--c->bytes_remaining == 1) {
		I2C_CR1(c->i2c) &= ~I2C_CR1_ACK;
		i2c_send_stop(c->i2c);
	}

	*data = i2c_get_data(c->i2c);

	PT_END();
}

pt_state_t i2c_ctx_stop(i2c_ctx_t *c)
{
	PT_BEGIN(&c->leaf);

	while (!i2c_ctx_is_timed_out(c) &&
	       !(I2C_SR1(c->i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)))
		PT_YIELD();

	if (!(I2C_SR1(c->i2c) & (I2C_SR1_BTF | I2C_SR1_TxE))) {
		i2c_ctx_reset(c);
		c->err = EIO;
		PT_EXIT();
	}

	i2c_send_stop(c->i2c);

	/* TODO: is it safe to just drop out of this */

	PT_END();
}

pt_state_t i2c_ctx_detect(i2c_ctx_t *c, i2c_device_map_t *map)
{
	PT_BEGIN(&c->pt);

	memset(map, 0, sizeof(*map));

	for (c->i = 0; c->i < 0x80; c->i++) {
		c->timeout = time_now() + 10000;
		c->err = 0;

		PT_SPAWN(&c->leaf, i2c_ctx_start(c));
		if (c->err)
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, c->i, I2C_WRITE));
		if (c->err)
			continue;

		PT_SPAWN(&c->leaf, i2c_ctx_stop(c));
		if (c->err)
			continue;

		map->devices[c->i / 16] |= 1 << (c->i % 16);
	}

	PT_END();
}

pt_state_t i2c_ctx_setreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, val));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}

pt_state_t i2c_ctx_getreg(i2c_ctx_t *c, uint16_t addr, uint16_t reg,
				 uint8_t *val)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, reg));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_READ));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_getdata(c, val));

	/* For reads STOP is generated automatically by sendaddr and/or
	 * getdata
	 */

	PT_END();
}

pt_state_t i2c_ctx_write(i2c_ctx_t *c, uint16_t addr, uint8_t *data,
				uint8_t len)
{
	PT_BEGIN(&c->pt);

	PT_SPAWN(&c->leaf, i2c_ctx_start(c));
	PT_EXIT_ON(c->err);

	PT_SPAWN(&c->leaf, i2c_ctx_sendaddr(c, addr, I2C_WRITE));
	PT_EXIT_ON(c->err);

	for (c->i = 0; c->i < len; c->i++) {
		PT_SPAWN(&c->leaf, i2c_ctx_senddata(c, data[c->i]));
		PT_EXIT_ON(c->err);
	}

	PT_SPAWN(&c->leaf, i2c_ctx_stop(c));

	PT_END();
}
