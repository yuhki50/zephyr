/*
 * Copyright (c) 2018, Yuuki Taguchi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <gpio.h>
#include <kernel.h>
#include <misc/printk.h>
#include <uart.h>

#define MAX_RX_BUFF 64 // RX buffer size

struct uart_gpio_config {
	u32_t baud_rate;
	u8_t data_bits;
	char *gpio_name;
	u8_t pin_tx;
	u8_t pin_rx;
	bool debug;
	u8_t pin_data_bit;
	u8_t pin_stop_bit;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(struct device *dev);
#endif
};

struct uart_gpio_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t irq_callback;
	void *irq_cb_data;
#endif

	struct device *dev;
	struct device *gpio;

	u32_t _rx_delay_centering;
	u32_t _rx_delay_intrabit;
	u32_t _rx_delay_stopbit;
	u32_t _tx_delay;

	bool _buffer_overflow;

	u8_t _receive_buffer[MAX_RX_BUFF];
	u8_t _receive_buffer_tail;
	u8_t _receive_buffer_head;
	struct gpio_callback gpio_cb;
};

static void uart_gpio_delay(unsigned int cycles_to_wait)
{
	u32_t start = k_cycle_get_32();

	/* Wait until the given number of cycles have passed */
	while (k_cycle_get_32() - start < cycles_to_wait);
}

/**
 * This function generates a brief pulse
 * for debugging or measuring on an oscilloscope.
 */
static inline void uart_gpio_debug(struct device *dev, u8_t pin)
{
	gpio_pin_write(dev, pin, 1);
	gpio_pin_write(dev, pin, 0);
}

static void uart_gpio_rx_gpio_callback(struct device *dev, struct gpio_callback *cb, u32_t pins)
{
	ARG_UNUSED(pins);

	struct uart_gpio_data *drv_data = CONTAINER_OF(cb, struct uart_gpio_data, gpio_cb);
	const struct uart_gpio_config *config = drv_data->dev->config->config_info;

	int ret;
	u8_t d = 0;

	// disable further interrupts during reception, this prevents
	// triggering another interrupt directly after we return, which can
	// cause problems at higher baudrates.
	ret = gpio_pin_disable_callback(dev, config->pin_rx);
	if (ret) {
		printk("Error disable gpio callback!\n");
	}

	// wait approximately 1/2 of a bit width to "center" the sample
	uart_gpio_delay(drv_data->_rx_delay_centering);
	if (config->debug) {
		uart_gpio_debug(dev, config->pin_data_bit);
	}

	// read each of the bits
	for (u8_t i = config->data_bits; i > 0; --i) {
		uart_gpio_delay(drv_data->_rx_delay_intrabit);
		d >>= 1;
		if (config->debug) {
			uart_gpio_debug(dev, config->pin_data_bit);
		}

		u32_t value = 0;
		int ret = gpio_pin_read(dev, config->pin_rx, &value);
		if (ret) {
			printk("Error configuring pin_rx: %d\n", config->pin_rx);
		}

		if (value) {
			d |= 0x80;
		}
	}

	// align to 8 bits
	d >>= 8 - config->data_bits;

	/* if buffer full, set the overflow flag and return */
	u8_t next = (drv_data->_receive_buffer_tail + 1) % MAX_RX_BUFF;

	if (next != drv_data->_receive_buffer_head) {
		/* save new data in buffer: tail points to where byte goes */
		drv_data->_receive_buffer[drv_data->_receive_buffer_tail] = d; // save new byte
		drv_data->_receive_buffer_tail = next;
	} else   {
		if (config->debug) {
			uart_gpio_debug(dev, config->pin_stop_bit);
		}
		drv_data->_buffer_overflow = true;
	}

	/* skip the stop bit */
	uart_gpio_delay(drv_data->_rx_delay_stopbit);
	if (config->debug) {
		uart_gpio_debug(dev, config->pin_stop_bit);
	}

	/* re-enable interrupts when we're sure to be inside the stop bit */
	ret = gpio_pin_enable_callback(dev, config->pin_rx);
	if (ret) {
		printk("Error enable gpio callback!\n");
	}
}

static int uart_gpio_poll_in(struct device *dev, unsigned char *c)
{
	struct uart_gpio_data *drv_data = dev->driver_data;

	/* empty buffer? */
	if (drv_data->_receive_buffer_head == drv_data->_receive_buffer_tail) {
		return -1;
	}

	/* read from "head" */
	*c = drv_data->_receive_buffer[drv_data->_receive_buffer_head];
	drv_data->_receive_buffer_head = (drv_data->_receive_buffer_head + 1) % MAX_RX_BUFF;

	return 0;
}

static unsigned char uart_gpio_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_gpio_config *config = dev->config->config_info;
	const struct uart_gpio_data *drv_data = dev->driver_data;
	unsigned int key = irq_lock();
	u8_t b = c;

	/* write start bit */
	gpio_pin_write(drv_data->gpio, config->pin_tx, 0);
	uart_gpio_delay(drv_data->_tx_delay);

	/* write data bit */
	for (u8_t i = config->data_bits; i > 0; --i) {
		gpio_pin_write(drv_data->gpio, config->pin_tx, b & 1);
		uart_gpio_delay(drv_data->_tx_delay);
		b >>= 1;
	}

	/* write stop bit */
	gpio_pin_write(drv_data->gpio, config->pin_tx, 1);
	uart_gpio_delay(drv_data->_tx_delay);

	irq_unlock(key);

	return c;
}

static int uart_gpio_err_check(struct device *dev)
{
	struct uart_gpio_data *drv_data = dev->driver_data;
	int err = 0;

	if (drv_data->_buffer_overflow) {
		err |= UART_ERROR_OVERRUN;
		drv_data->_buffer_overflow = false;
	}

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_gpio_fifo_fill(struct device *dev, const u8_t *tx_data, int len)
{
	u8_t num_tx = 0;

	while (len - num_tx > 0) {
		uart_gpio_poll_out(dev, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_gpio_fifo_read(struct device *dev, u8_t *rx_data, const int size)
{
	struct uart_gpio_data *drv_data = dev->driver_data;
	u8_t num_rx = 0;

	while ((size - num_rx > 0) && (drv_data->_receive_buffer_head != drv_data->_receive_buffer_tail)) {
		rx_data[num_rx++] = drv_data->_receive_buffer[drv_data->_receive_buffer_head];
		drv_data->_receive_buffer_head = (drv_data->_receive_buffer_head + 1) % MAX_RX_BUFF;
	}

	return num_rx;
}

static void uart_gpio_irq_tx_enable(struct device *dev)
{
    // none
}

static void uart_gpio_irq_tx_disable(struct device *dev)
{
    // none
}

static int uart_gpio_irq_tx_ready(struct device *dev)
{
	return 0;
}

static void uart_gpio_irq_rx_enable(struct device *dev)
{
	const struct uart_gpio_config *config = dev->config->config_info;
	struct uart_gpio_data *drv_data = dev->driver_data;

	int ret = gpio_pin_enable_callback(drv_data->gpio, config->pin_rx);

	if (ret) {
		printk("Error enabling callback!\n");
	}
}

static void uart_gpio_irq_rx_disable(struct device *dev)
{
	const struct uart_gpio_config *config = dev->config->config_info;
	struct uart_gpio_data *drv_data = dev->driver_data;

	int ret = gpio_pin_disable_callback(drv_data->gpio, config->pin_rx);

	if (ret) {
		printk("Error enabling callback!\n");
	}
}

static int uart_gpio_irq_tx_complete(struct device *dev)
{
	return 0;
}

static int uart_gpio_irq_rx_ready(struct device *dev)
{
	const struct uart_gpio_data *drv_data = dev->driver_data;

	return (drv_data->_receive_buffer_tail + MAX_RX_BUFF - drv_data->_receive_buffer_head) % MAX_RX_BUFF;
}

static void uart_gpio_irq_err_enable(struct device *dev)
{
	// none
}

static void uart_gpio_irq_err_disable(struct device *dev)
{
	// none
}

static int uart_gpio_irq_is_pending(struct device *dev)
{
	return uart_gpio_irq_tx_ready(dev) || uart_gpio_irq_rx_ready(dev);
}

static int uart_gpio_irq_update(struct device *dev)
{
	return 1;
}

static void uart_gpio_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_gpio_data *drv_data = dev->driver_data;

	drv_data->irq_callback = cb;
	drv_data->irq_cb_data = cb_data;
}

/*
   static void uart_gpio_isr(void *arg)
   {
        struct device *dev = arg;
        struct uart_gpio_data *drv_data = dev->driver_data;

        if (drv_data->irq_callback) {
                drv_data->irq_callback(drv_data->irq_cb_data);
        }
   }
 */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static void uart_gpio_init_pins(struct device *dev)
{
	const struct uart_gpio_config *config = dev->config->config_info;
	struct uart_gpio_data *drv_data = dev->driver_data;
	int ret;

	/* tx */
	ret = gpio_pin_configure(drv_data->gpio, config->pin_tx, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring pin_tx: %d\n", config->pin_tx);
	}

	ret = gpio_pin_write(drv_data->gpio, config->pin_tx, 1);
	if (ret) {
		printk("Error set pin_tx: %d\n", config->pin_tx);
	}

	/* rx */
	ret = gpio_pin_configure(drv_data->gpio, config->pin_rx, (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_PUD_NORMAL));
	if (ret) {
		printk("Error configuring pin_rx: %d\n", config->pin_rx);
	}

	gpio_init_callback(&drv_data->gpio_cb, uart_gpio_rx_gpio_callback, BIT(config->pin_rx));

	ret = gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb);
	if (ret) {
		printk("Cannot setup callback\n");
	}

	if (config->debug) {
		/* debug for data bit */
		ret = gpio_pin_configure(drv_data->gpio, config->pin_data_bit, GPIO_DIR_OUT);
		if (ret) {
			printk("Error configuring pin_data_bit: %d\n", config->pin_data_bit);
		}

		/* debug for stop bit */
		ret = gpio_pin_configure(drv_data->gpio, config->pin_stop_bit, GPIO_DIR_OUT);
		if (ret) {
			printk("Error configuring pin_stop_bit: %d\n", config->pin_stop_bit);
		}
	}
}

static int uart_gpio_init(struct device *dev)
{
	const struct uart_gpio_config *config = dev->config->config_info;
	struct uart_gpio_data *drv_data = dev->driver_data;

	/* Initializer */
	drv_data->_buffer_overflow = false;
	drv_data->_receive_buffer_head = 0;
	drv_data->_receive_buffer_tail = 0;
	drv_data->dev = dev;
	drv_data->gpio = device_get_binding(config->gpio_name);

	/* Initialize UART pins */
	uart_gpio_init_pins(dev);

	/* Precalculate the various delays */
	u32_t bit_delay = (u64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / config->baud_rate + 1;

	/* These are all close enough to just use 15 cycles, since the inter-bit
	 * timings are the most critical (deviations stack 8 times)
	 */
	drv_data->_tx_delay = bit_delay;

	/* When the start bit occurs, there are 3 or 4 cycles before the
	 * interrupt flag is set, 4 cycles before the PC is set to the right
	 * interrupt vector address and the old PC is pushed on the stack,
	 * and then 75 cycles of instructions (including the RJMP in the
	 * ISR vector table) until the first delay. After the delay, there
	 * are 17 more cycles until the pin value is read (excluding the
	 * delay in the loop).
	 * We want to have a total delay of 1.5 bit time. Inside the loop,
	 * we already wait for 1 bit time - 23 cycles, so here we wait for
	 * 0.5 bit time - (71 + 18 - 22) cycles.
	 */
	drv_data->_rx_delay_centering = bit_delay / 2;

	/* There are 23 cycles in each loop iteration (excluding the delay) */
	drv_data->_rx_delay_intrabit = bit_delay;

	/* There are 37 cycles from the last bit read to the start of
	 * stopbit delay and 11 cycles from the delay until the interrupt
	 * mask is enabled again (which _must_ happen during the stopbit).
	 * This delay aims at 3/4 of a bit time, meaning the end of the
	 * delay will be at 1/4th of the stopbit. This allows some extra
	 * time for ISR cleanup, which makes 115200 baud at 16Mhz work more
	 * reliably
	 */
	drv_data->_rx_delay_stopbit = bit_delay;

	// uart_gpio_delay(drv_data->_tx_delay); /* if we were low this establishes the end */

	int ret = gpio_pin_enable_callback(drv_data->gpio, config->pin_rx);
	if (ret) {
		printk("Error enabling callback!\n");
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

static const struct uart_driver_api uart_gpio_driver_api = {
	.poll_in = uart_gpio_poll_in,
	.poll_out = uart_gpio_poll_out,
	.err_check = uart_gpio_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_gpio_fifo_fill,
	.fifo_read = uart_gpio_fifo_read,
	.irq_tx_enable = uart_gpio_irq_tx_enable,
	.irq_tx_disable = uart_gpio_irq_tx_disable,
	.irq_tx_ready = uart_gpio_irq_tx_ready,
	.irq_rx_enable = uart_gpio_irq_rx_enable,
	.irq_rx_disable = uart_gpio_irq_rx_disable,
	.irq_tx_complete = uart_gpio_irq_tx_complete,
	.irq_rx_ready = uart_gpio_irq_rx_ready,
	.irq_err_enable = uart_gpio_irq_err_enable,
	.irq_err_disable = uart_gpio_irq_err_disable,
	.irq_is_pending = uart_gpio_irq_is_pending,
	.irq_update = uart_gpio_irq_update,
	.irq_callback_set = uart_gpio_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_GPIO_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gpio_config_func_0(struct device *dev);
#endif

static const struct uart_gpio_config uart_gpio_0_config = {
	.baud_rate = CONFIG_UART_GPIO_0_BAUD_RATE,
	.data_bits = CONFIG_UART_GPIO_0_DATA_BITS,
	.gpio_name = CONFIG_UART_GPIO_0_GPIO,
	.pin_tx = CONFIG_UART_GPIO_0_TX_PIN,
	.pin_rx = CONFIG_UART_GPIO_0_RX_PIN,
#ifdef CONFIG_UART_GPIO_0_DEBUG
	.debug = CONFIG_UART_GPIO_0_DEBUG,
	.pin_data_bit = CONFIG_UART_GPIO_0_DEBUG_DATA_BIT_PIN,
	.pin_stop_bit = CONFIG_UART_GPIO_0_DEBUG_STOP_BIT_PIN,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_gpio_config_func_0,
#endif
};

static struct uart_gpio_data uart_gpio_0_data;

DEVICE_AND_API_INIT(uart_0, CONFIG_UART_GPIO_0_NAME,
		    &uart_gpio_init,
		    &uart_gpio_0_data, &uart_gpio_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_gpio_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gpio_config_func_0(struct device *dev)
{
//	IRQ_CONNECT(UART0_RX_IRQn, CONFIG_UART_GPIO_0_IRQ_PRI, uart_gpio_isr, DEVICE_GET(uart_gpio_0), 0);
//	IRQ_CONNECT(UART0_TX_IRQn, CONFIG_UART_GPIO_0_IRQ_PRI, uart_gpio_isr, DEVICE_GET(uart_gpio_0), 0);
//
//	irq_enable(UART0_TX_IRQn);
//	irq_enable(UART0_RX_IRQn);
}
#endif

#endif /* CONFIG_UART_GPIO_0 */
