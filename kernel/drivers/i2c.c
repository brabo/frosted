#include "frosted.h"
#include <stdint.h>
#include "i2c.h"

#include "libopencm3/cm3/nvic.h"

#ifdef LM3S
#   include "libopencm3/lm3s/i2c.h"
#   define CLOCK_ENABLE(C)
#endif
#ifdef STM32F4
#   include "libopencm3/stm32/i2c.h"
#   include "libopencm3/stm32/rcc.h"
#   define CLOCK_ENABLE(C)                 rcc_periph_clock_enable(C);
#endif
#ifdef LPC17XX
#   include "libopencm3/lpc17xx/i2c.h"
#   define CLOCK_ENABLE(C)
#endif
struct dev_i2c {
	uint32_t base;
	uint32_t irq;
	struct fnode *fno;
	mutex_t *mutex;
	uint16_t pid;
	uint8_t address;
};

#define MAX_I2CS 1

static struct dev_i2c DEV_I2C[MAX_I2CS];

static int i2c_subsys_initialized = 0;

static struct module mod_devi2c = {
};

static int devi2c_write(int fd, const void *buf, unsigned int len);

static struct dev_i2c *i2c_check_fd(int fd)
{
	struct fnode *fno;
	fno = task_filedesc_get(fd);

	if (!fno)
		return 0;
	if (fd < 0)
		return 0;

	if (fno->owner != &mod_devi2c)
		return 0;
	return fno->priv;
}


static uint8_t devi2c_start(uint32_t i2c, uint8_t address, uint8_t mode)
{
	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, address, mode);

	int timeout = 20000;
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {
		if (timeout > 0) {
			timeout--;
		} else {
			return 1;
		}
	}


	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	return 0;
}



static int devi2c_write(int fd, const void *buf, unsigned int len)
{
	int i;
	uint8_t *reg = (uint8_t *)buf++;
	uint8_t *data = (uint8_t *)buf;
	struct dev_i2c *i2c;

	i2c = i2c_check_fd(fd);
	if (!i2c)
		return -1;
	if (len <= 0)
		return len;
	if (fd < 0)
		return -1;

	frosted_mutex_lock(i2c->mutex);
	//usart_enable_tx_interrupt(uart->base);

	/* write to circular output buffer */

	devi2c_start(i2c->base, i2c->address, I2C_WRITE);

	i2c_send_data(i2c->base, reg[0]);

	while (!(I2C_SR1(i2c->base) & (I2C_SR1_BTF)));
	i2c_send_data(i2c->base, data[0]);

	while (!(I2C_SR1(i2c->base) & (I2C_SR1_BTF)));

	i2c_send_stop(i2c->base);

	frosted_mutex_unlock(i2c->mutex);
	return len;
}


static int devi2c_read(int fd, void *buf, unsigned int len)
{
	int out;
	uint8_t *reg = (uint8_t *)buf;
	volatile int len_available = 1;
	struct dev_i2c *i2c;

	if (len <= 0)
		return len;
	if (fd < 0)
		return -1;

	i2c = i2c_check_fd(fd);
	if (!i2c)
		return -1;

	frosted_mutex_lock(i2c->mutex);

	uint32_t timeout = 20000;
	while ((I2C_SR2(i2c->base) & I2C_SR2_BUSY)); // {
//		if (timeout > 0) {
//			timeout--;
//		} else {
//			return -1;
//		}
//	}

	if (devi2c_start(i2c->base, i2c->address, I2C_WRITE)) {
		return 0;
	}
	i2c_send_data(i2c->base, reg[0]);

	timeout = 20000;
	while (!(I2C_SR1(i2c->base) & (I2C_SR1_BTF))) {
		if (timeout > 0) {
			timeout--;
		} else {
			return -1;
		}
	}

	devi2c_start(i2c->base, reg[0], I2C_READ);

	i2c_send_stop(i2c->base);

	while (!(I2C_SR1(i2c->base) & I2C_SR1_RxNE));

	*((uint8_t*)buf) = (int)i2c_get_data(i2c->base);

	I2C_SR1(i2c->base) &= ~I2C_SR1_AF;
	//msleep(50);
	int i;
	for (i = 0; i < 1000000; i++) {
		__asm__("nop");
	}
	i2c_send_stop(i2c->base);

	frosted_mutex_unlock(i2c->mutex);
	return 1;
}


static int devi2c_open(const char *path, int flags)
{
	struct fnode *f = fno_search(path);
	if (!f)
		return -1;
	return task_filedesc_add(f);
}


static int i2c_fno_init(struct fnode *dev, uint32_t n, const struct i2c_addr * addr)
{
	struct dev_i2c *u = &DEV_I2C[n];
	static int num_i2cs = 0;

	char name[6] = "i2c";
	name[4] =  '0' + num_i2cs++;

	if (addr->base == 0)
		return -1;

	u->base = addr->base;

	u->fno = fno_create(&mod_devi2c, name, dev);
	u->pid = 0;
	u->mutex = frosted_mutex_init();
	u->address = addr->address;
	//u->inbuf = cirbuf_create(128);
	//u->outbuf = cirbuf_create(128);
	u->fno->priv = u;
	//usart_enable_rx_interrupt(u->base);
	//nvic_enable_irq(u->irq);
	return 0;

}



static struct module * devi2c_init(struct fnode *dev)
{
	mod_devi2c.family = FAMILY_FILE;
	strcpy(mod_devi2c.name,"i2c");
	mod_devi2c.ops.open = devi2c_open;
	mod_devi2c.ops.read = devi2c_read;
	//mod_devi2c.ops.poll = devi2c_poll;
	mod_devi2c.ops.write = devi2c_write;

	/* Module initialization */
	if (!i2c_subsys_initialized) {
		i2c_subsys_initialized++;
	}

	klog(LOG_INFO, "I2C Driver: KLOG enabled.\n");
	return &mod_devi2c;
}

void i2c_init(struct fnode * dev, const struct i2c_addr i2c_addrs[], int num_i2cs)
{
	int i;

	struct module * devi2c = devi2c_init(dev);

	for (i = 0; i < num_i2cs; i++)
	{
		CLOCK_ENABLE(i2c_addrs[i].rcc);
		i2c_fno_init(dev, i2c_addrs[i].devidx, &i2c_addrs[i]);

		i2c_peripheral_disable(i2c_addrs[i].base); /* disable i2c during setup */
		i2c_reset(i2c_addrs[i].base);

		i2c_set_fast_mode(i2c_addrs[i].base);
		i2c_set_clock_frequency(i2c_addrs[i].base, i2c_addrs[i].freq); //I2C_CR2_FREQ_42MHZ);
		i2c_set_ccr(i2c_addrs[i].base, 35);
		i2c_set_trise(i2c_addrs[i].base, 43);
		//i2c_set_speed(I2C3, 0);
		i2c_peripheral_enable(i2c_addrs[i].base); /* finally enable i2c */

		i2c_set_own_7bit_slave_address(i2c_addrs[i].base, 0x00);
	}
	register_module(devi2c);
}
