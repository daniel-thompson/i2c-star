/*
 * This file is part of the i2c-star project.
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

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <librfn/console.h>
#include <librfn/fibre.h>
#include <librfn/regdump.h>
#include <librfn/time.h>
#include <librfn/util.h>
#include <librfm3/i2c_ctx.h>

static console_t cdcacm_console;

static uint32_t i2c = I2C1;

static pt_state_t do_i2cstart(console_t *c)
{
	i2c_ctx_t *ctx = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	i2c_ctx_init(ctx, i2c);
	ctx->verbose = true;
	PT_SPAWN(&ctx->leaf, i2c_ctx_start(ctx));

	PT_END();
}

static pt_state_t do_i2csendaddr(console_t *c)
{
	struct {
		uint8_t addr;
		uint8_t bytes_to_read;
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	s->addr = strtol(c->argv[1], NULL, 0);
	s->bytes_to_read = strtol(c->argv[2], NULL, 0);

	i2c_ctx_init(&s->ctx, i2c);
	s->ctx.verbose = true;
	PT_SPAWN(&s->ctx.leaf,
		 i2c_ctx_sendaddr(&s->ctx, s->addr, s->bytes_to_read));

	PT_END();
}

static pt_state_t do_i2csenddata(console_t *c)
{
	struct {
		uint8_t data;
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	s->data = strtol(c->argv[1], NULL, 0);

	i2c_ctx_init(&s->ctx, i2c);
	s->ctx.verbose = true;
	PT_SPAWN(&s->ctx.leaf, i2c_ctx_senddata(&s->ctx, s->data));

	PT_END();
}

static pt_state_t do_i2cgetdata(console_t *c)
{
	struct {
		uint8_t data;
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	i2c_ctx_init(&s->ctx, i2c);
	s->ctx.verbose = true;
	PT_SPAWN(&s->ctx.leaf, i2c_ctx_getdata(&s->ctx, &s->data));

	if (!s->ctx.err)
		fprintf(c->out, "Read 0x%02x\n", s->data);

	PT_END();
}


static pt_state_t do_i2cstop(console_t *c)
{
	i2c_ctx_t *ctx = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	i2c_ctx_init(ctx, i2c);
	ctx->verbose = true;
	PT_SPAWN(&ctx->leaf, i2c_ctx_stop(ctx));

	PT_END();
}

static pt_state_t do_i2creset(console_t *c)
{
	i2c_ctx_t *ctx = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	i2c_ctx_init(ctx, i2c);
	ctx->verbose = true;
	i2c_ctx_reset(ctx);

	PT_END();
}

static pt_state_t do_i2cdetect(console_t *c)
{
	struct {
		i2c_ctx_t ctx;
		i2c_device_map_t map;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	i2c_ctx_init(&s->ctx, i2c);
	PT_SPAWN(&s->ctx.pt, i2c_ctx_detect(&s->ctx, &s->map));

	fprintf(c->out,
		"     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");

	for (s->ctx.i = 0; s->ctx.i < 0x80; s->ctx.i++) {
		if ((s->ctx.i & 0xf) == 0) {
			printf("\n");
			printf("%02x:", s->ctx.i);
		}

		if (s->map.devices[s->ctx.i / 16] & 1 << (s->ctx.i % 16))
			printf(" %02x", s->ctx.i);
		else
			printf(" --");
	}
	printf("\n");

	PT_END();
}

static pt_state_t do_i2cset(console_t *c)
{
	struct {
		uint16_t addr;
		uint16_t reg;
		uint16_t val;
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	s->addr = strtol(c->argv[1], NULL, 0);
	s->reg = strtol(c->argv[2], NULL, 0);
	s->val = strtol(c->argv[3], NULL, 0);

	i2c_ctx_init(&s->ctx, i2c);
	s->ctx.verbose = true;

	PT_SPAWN(&s->ctx.pt, i2c_ctx_setreg(&s->ctx, s->addr, s->reg, s->val));

	PT_END();
}

static pt_state_t do_i2cget(console_t *c)
{
	struct {
		uint16_t addr;
		uint16_t reg;
		uint8_t val;
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	s->addr = strtol(c->argv[1], NULL, 0);
	s->reg = strtol(c->argv[2], NULL, 0);

	i2c_ctx_init(&s->ctx, i2c);
	s->ctx.verbose = true;

	PT_SPAWN(&s->ctx.pt, i2c_ctx_getreg(&s->ctx, s->addr, s->reg, &s->val));

	if (!s->ctx.err)
		fprintf(c->out, "Read 0x%02x\n", s->val);

	PT_END();
}

static pt_state_t do_i2cwrite(console_t *c)
{
	struct {
		uint16_t addr;
		uint8_t num_bytes;
		uint8_t data[16];
		i2c_ctx_t ctx;
	} *s = (void *) &c->scratch.u8[0];

	PT_BEGIN(&c->pt);

	/* Argument parsing. The structure is carefully organised so
	 * that the arguments (which share the scratch space) are not
	 * clobbered before they are read.
	 */
	s->addr = strtol(c->argv[1], NULL, 0);
	s->num_bytes = strlen(c->argv[2]) / 2;
	for (int i=0; i<s->num_bytes; i++) {
		char byte[3] = { c->argv[2][i * 2], c->argv[2][i * 2 + 1],
				 '\0' };
		s->data[i] = strtol(byte, NULL, 16);
	}

	/* Issue the I2C transaction. */
	i2c_ctx_init(&s->ctx, i2c);
	PT_SPAWN(&s->ctx.pt,
		 i2c_ctx_write(&s->ctx, s->addr, s->data, s->num_bytes));

	if (s->ctx.err)
		fprintf(c->out, "Write failed\n");

	PT_END();
}

static pt_state_t do_i2cbus(console_t *c)
{
	int bus = strtol(c->argv[1], NULL, 0);

	switch (bus) {
	case 1:
		i2c = I2C1;
		break;
	case 2:
		i2c = I2C2;
		break;
	case 3:
		i2c = I2C3;
		break;
	default:
		fprintf(c->out, "ERROR: Bad bus %d\n", bus);
	}

	return PT_EXITED;
}

static pt_state_t do_uptime(console_t *c)
{
	unsigned int hours, minutes, seconds, microseconds;

	uint64_t t = time64_now();

	/* get to 32-bit values as directly as possible */
	minutes = t / (60 * 1000000);
	microseconds = t % (60 * 1000000);

	hours = minutes / 60;
	minutes %= 60;
	seconds = microseconds / 1000000;
	microseconds %= 1000000;

	fprintf(c->out, "%02u:%02u:%02u.%03u\n", hours, minutes, seconds,
		microseconds / 1000);

	return PT_EXITED;
}

static const console_cmd_t cmd_list[] = {
	CONSOLE_CMD_VAR_INIT("i2cstart", do_i2cstart),
	CONSOLE_CMD_VAR_INIT("i2csendaddr", do_i2csendaddr),
	CONSOLE_CMD_VAR_INIT("i2csenddata", do_i2csenddata),
	CONSOLE_CMD_VAR_INIT("i2cgetdata", do_i2cgetdata),
	CONSOLE_CMD_VAR_INIT("i2cstop", do_i2cstop),
	CONSOLE_CMD_VAR_INIT("i2creset", do_i2creset),
	CONSOLE_CMD_VAR_INIT("i2cdetect", do_i2cdetect),
	CONSOLE_CMD_VAR_INIT("i2cset", do_i2cset),
	CONSOLE_CMD_VAR_INIT("i2cget", do_i2cget),
	CONSOLE_CMD_VAR_INIT("i2cwrite", do_i2cwrite),
	CONSOLE_CMD_VAR_INIT("i2cbus", do_i2cbus),
	CONSOLE_CMD_VAR_INIT("uptime", do_uptime)
};

static const console_gpio_t gpio_list[] = {
	CONSOLE_GPIO_VAR_INIT("led", GPIOD, GPIO12, 0),
	CONSOLE_GPIO_VAR_INIT("dac", GPIOD, GPIO4, console_gpio_default_on)
};

static void i2c_init(void)
{
	/* clocks */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_I2C3);

	/* initialize the peripherals */
	i2c_ctx_t ctx;
	i2c_ctx_init(&ctx, I2C1);
	i2c_ctx_reset(&ctx);
	i2c_ctx_init(&ctx, I2C2);
	i2c_ctx_reset(&ctx);
	i2c_ctx_init(&ctx, I2C3);
	i2c_ctx_reset(&ctx);

	/* GPIO for I2C1 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO6 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO9);

	/* GPIO for I2C3 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);
}

int main(void)
{
        rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	i2c_init();
	time_init();

	console_init(&cdcacm_console, stdout);
	for (unsigned int i=0; i<lengthof(cmd_list); i++)
		console_register(&cmd_list[i]);
	for (unsigned int i=0; i<lengthof(gpio_list); i++)
		console_gpio_register(&gpio_list[i]);


	fibre_scheduler_main_loop();
}

