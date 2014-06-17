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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <librfn/console.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>

static console_t cdcacm_console;

uint32_t i2c = I2C1;

static pt_state_t do_i2cstart(console_t *c)
{
	PT_BEGIN(&c->pt);

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB) &
		 (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
		PT_YIELD();

	PT_END();
}

static pt_state_t do_i2csendaddr(console_t *c)
{
	uint32_t *timeout = &c->scratch.u32[0];

	PT_BEGIN(&c->pt);

	/* Send destination address. */
	uint8_t slaveaddr = strtol(c->argv[1], NULL, 0);
	i2c_send_7bit_address(i2c, slaveaddr, I2C_WRITE);

	/* Waiting for address is transferred. */
	*timeout = time_now() + 5000000;
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
		PT_YIELD();
		if (cyclecmp32(time_now(), *timeout) > 0) {
			fprintf(c->out, "TIMED OUT: I2C_SR1 0x%08lx\n",
				I2C_SR1(i2c));
			break;
		}
	}

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 __attribute__((unused));
	reg32 = I2C_SR2(i2c);

	PT_END();
}

static pt_state_t do_i2csendbyte(console_t *c)
{
	PT_BEGIN(&c->pt);

	/* Sending the data. */
	uint8_t data = strtol(c->argv[1], NULL, 0);
	i2c_send_data(i2c, data);
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF))
		PT_YIELD(); /* Await ByteTransferedFlag. */

	PT_END();
}

static pt_state_t do_i2cstop(console_t *c)
{
	PT_BEGIN(&c->pt);

	/* This looks wrong! */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)))
		PT_YIELD();

	/* Send STOP condition. */
	i2c_send_stop(i2c);

	PT_END();
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
    CONSOLE_CMD_VAR_INIT("i2csendbyte", do_i2csendbyte),
    CONSOLE_CMD_VAR_INIT("i2cstop", do_i2cstop),
    CONSOLE_CMD_VAR_INIT("uptime", do_uptime)
};

const console_gpio_t gpio_led = CONSOLE_GPIO_VAR_INIT("led", GPIOD, GPIO12, 0);

static void i2c_init(void)
{
	/* clocks */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);

	rcc_periph_reset_pulse(RST_I2C1);

	/* console commands */
	i2c = I2C1;

	/* gpio */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO6 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO9);

	/* peripheral configuration */
	i2c_peripheral_disable(i2c);
	i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_30MHZ);
	/*i2c_set_ccr(i2c, 0x1e);*/
	/*i2c_set_trise(i2c, 0x0b);*/

	/* Standard mode (Sm), "square" duty cycle, very slow */
	i2c_set_ccr(i2c, 0x0fff);
	i2c_set_trise(i2c, 0x3f);
	i2c_set_own_7bit_slave_address(i2c, 0x32);
	i2c_peripheral_enable(i2c);

}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);

	i2c_init();
	time_init();
	console_init(&cdcacm_console, stdout);
	for (unsigned int i=0; i<lengthof(cmd_list); i++)
		console_register(&cmd_list[i]);
	console_gpio_register(&gpio_led);

	fibre_scheduler_main_loop();
}

