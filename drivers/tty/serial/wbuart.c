// SPDX-License-Identifier: GPL-2.0
/*
 * wbuart.c: Serial driver for Wishbone & AXI-lite Controlled UART
 *
 * Copyright (C) 2023 Johnson Liu
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>


/* ---------------------------------------------------------------------
 * Register definitions
 *
 * For register details see datasheet:
 * https://github.com/ZipCPU/wbuart32/blob/master/doc/spec.pdf
 */

#define WBUART_SETUP   0x00
#define WBUART_FIFO    0x04
#define WBUART_RX_DATA 0x08
#define WBUART_TX_DATA 0x0c

#define WBUART_MAX_PORTS 1
#define WBUART_BAUD_RATE 115200
#define WBUART_DATA_BIT  8
#define WBUART_FIFO_SIZE 1024

static struct uart_port *wbuart_ports[WBUART_MAX_PORTS];

#define to_wbuart_port(port) container_of(port, struct wbuart_port, port)

static inline void wbuart_write(struct uart_port *port, u32 val, unsigned int off)
{
	writel(val, port->membase + off);
}

static inline u32 wbuart_read(struct uart_port *port, unsigned int off)
{
	return readl(port->membase + off);
}

static void wbuart_putchar(struct uart_port *port, int ch)
{
	// wait while tx fifo is full
	while (((wbuart_read(port, WBUART_FIFO) >> 18) & 0x03FF) == 0)
		cpu_relax();

	wbuart_write(port, (u32 __force)cpu_to_le32(ch), WBUART_TX_DATA);
}

/* ---------------------------------------------------------------------
 * Platform bus binding
 */

static void wbuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* modem control register is not present in WBUART */
}

static unsigned int wbuart_get_mctrl(struct uart_port *port)
{
	/*CTS (Clear To Send), DSR (Data Set Ready), CAR (Data Carrier Detect)*/
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void wbuart_break_ctl(struct uart_port *port, int break_state)
{
	/* WBUART doesn't support sending break signal */
}

static void wbuart_stop_tx(struct uart_port *port)
{
}

static void wbuart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned char ch;

	if (unlikely(port->x_char)) 
	{
		wbuart_putchar(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
	} 
	else if (!uart_circ_empty(xmit)) 
	{
		while (xmit->head != xmit->tail) 
		{
			ch = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			wbuart_putchar(port, ch);
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static unsigned int wbuart_tx_empty(struct uart_port *port)
{
	/* not really tx empty, just checking if tx is not full */
	if (((wbuart_read(port, WBUART_FIFO) >> 18) & 0x03FF) != 0)
		return TIOCSER_TEMT;

	return 0;
}

static void wbuart_stop_rx(struct uart_port *port)
{
}

static int wbuart_startup(struct uart_port *port)
{
	return 0;
}

static void wbuart_shutdown(struct uart_port *port)
{
}

static void wbuart_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old)
{
	unsigned int baud;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* update baudrate */
	baud = uart_get_baud_rate(port, new, old, 0, 460800);
	uart_update_timeout(port, new->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *wbuart_type(struct uart_port *port)
{
	return "wbuart";
}

static void wbuart_release_port(struct uart_port *port)
{
}

static int wbuart_request_port(struct uart_port *port)
{
	return 0;
}

static void wbuart_config_port(struct uart_port *port, int flags)
{
	/*
	 * Driver core for serial ports forces a non-zero value for port type.
	 * Write an arbitrary value here to accommodate the serial core driver,
	 * as ID part of UAPI is redundant.
	 */
	port->type = 1;
}

static int wbuart_verify_port(struct uart_port *port,	struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != 1)
		return -EINVAL;

	return 0;
}

static const struct uart_ops wbuart_ops = {
	.set_mctrl	 = wbuart_set_mctrl,
	.get_mctrl	 = wbuart_get_mctrl,
	.break_ctl	 = wbuart_break_ctl,
	.stop_tx	 = wbuart_stop_tx,
	.start_tx	 = wbuart_start_tx,
	.tx_empty	 = wbuart_tx_empty,
	.stop_rx	 = wbuart_stop_rx,
	.startup	 = wbuart_startup,
	.shutdown	 = wbuart_shutdown,
	.set_termios = wbuart_set_termios,
	.type		 = wbuart_type,
	.release_port = wbuart_release_port,
	.request_port = wbuart_request_port,
	.config_port  = wbuart_config_port,
	.verify_port  = wbuart_verify_port,
};

/* ---------------------------------------------------------------------
 * Console driver operations
 */

#ifdef CONFIG_SERIAL_WBUART_CONSOLE
static struct console wbuart_console;
#endif

static struct uart_driver wbuart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "wbuart",
	.dev_name = "ttyWBUART",
	.major = 0,
	.minor = 0,
	.nr = WBUART_MAX_PORTS,
#ifdef CONFIG_SERIAL_WBUART_CONSOLE
	.cons = &wbuart_console,
#endif
};

#ifdef CONFIG_SERIAL_WBUART_CONSOLE

static int wbuart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = WBUART_BAUD_RATE;
	int bits = WBUART_DATA_BIT;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= WBUART_MAX_PORTS)
		return -EINVAL;

	port = wbuart_ports[co->index];
	if (!port || !port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void wbuart_console_write(struct console *co, const char *s, unsigned int count)
{
	unsigned long flags;
	struct uart_port *port = wbuart_ports[co->index];

	if (!port)
		return;

	spin_lock_irqsave(&port->lock, flags);
	uart_console_write(port, s, count, wbuart_putchar);
	spin_unlock_irqrestore(&port->lock, flags);
}

static struct console wbuart_console = {
	.name = "nitrouart",	// console names can't contain numbers !!!
	.write = wbuart_console_write,
	.device = uart_console_device,
	.setup = wbuart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &wbuart_driver,
};

static int __init wbuart_console_init(void)
{
	register_console(&wbuart_console);

	return 0;
}
console_initcall(wbuart_console_init);

#endif /* CONFIG_SERIAL_WBUART_CONSOLE */

/* ---------------------------------------------------------------------
 * Platform bus binding
 */

static int wbuart_probe(struct platform_device *pdev)
{
	struct uart_port *port;
	struct resource *res;
	int ret = 0;

	if (pdev->dev.of_node)
		pdev->id = of_alias_get_id(pdev->dev.of_node, "serial");

	if (pdev->id < 0) 
	{
		int id = 0;
		for (id = 0; id < WBUART_MAX_PORTS; id++) 
		{
			if (!wbuart_ports[id]) 
			{
				pdev->id = id;
				break;
			}
		}
	}

	if (pdev->id < 0 || pdev->id >= WBUART_MAX_PORTS)
		return -EINVAL;

	port = devm_kzalloc(&pdev->dev, sizeof(struct uart_port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (!port->membase)
		return -ENXIO;
	port->mapbase = res->start;

	port->line = pdev->id;
	port->dev = &pdev->dev;
	port->flags = UPF_BOOT_AUTOCONF;
	port->type = PORT_UNKNOWN;
	port->fifosize = WBUART_FIFO_SIZE;
	port->iotype = UPIO_MEM;
	port->ops = &wbuart_ops;
	port->iobase = 1;
	spin_lock_init(&port->lock);

	wbuart_ports[pdev->id] = port;
	platform_set_drvdata(pdev, port);

	ret = uart_add_one_port(&wbuart_driver, port);
	if (ret)
		wbuart_ports[pdev->id] = NULL;

	return ret;
}

static int wbuart_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	uart_remove_one_port(&wbuart_driver, port);
	wbuart_ports[pdev->id] = NULL;

	return 0;
}

/* work with hotplug and coldplug */

/* Search Device Tree */
static const struct of_device_id wbuart_of_match[] = {
	{ .compatible = "wbuart" },
	{}
};
MODULE_DEVICE_TABLE(of, wbuart_of_match);

static struct platform_driver wbuart_platform_driver = {
	.probe = wbuart_probe,
	.remove = wbuart_remove,
	.driver = {
		.name  = "wbuart",
		.of_match_table = wbuart_of_match,
	},
};

/* ---------------------------------------------------------------------
 * Module setup/teardown
 */

static int __init wbuart_init(void)
{
	int ret;

	ret = uart_register_driver(&wbuart_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&wbuart_platform_driver);
	if (ret)
		uart_unregister_driver(&wbuart_driver);

	return ret;
}

static void __exit wbuart_exit(void)
{
	platform_driver_unregister(&wbuart_platform_driver);
	uart_unregister_driver(&wbuart_driver);
}

module_init(wbuart_init);
module_exit(wbuart_exit);

MODULE_AUTHOR("Johnson Liu");
MODULE_DESCRIPTION("Wishbone & AXI-Lite Controlled UART serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wbuart");