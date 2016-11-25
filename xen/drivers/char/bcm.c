/*
 * (C) Copyright 2016 Stephen Warren <swarren@wwwdotorg.org>
 *
 * Derived from pl01x code:
 *
 * (C) Copyright 2000
 * Rob Taylor, Flying Pig Systems. robt@flyingpig.com.
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/* Simple U-Boot driver for the BCM283x mini UART */

#include <xen/config.h>
#include <xen/console.h>
#include <xen/serial.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/device_tree.h>
#include <xen/errno.h>
#include <asm/device.h>
#include <xen/mm.h>
#include <xen/vmap.h>
#include <asm/pl011-uart.h>
#include <asm/io.h>

struct bcm283x_mu_regs {
	u32 io;
	u32 iir;
	u32 ier;
	u32 lcr;
	u32 mcr;
	u32 lsr;
	u32 msr;
	u32 scratch;
	u32 cntl;
	u32 stat;
	u32 baud;
};

#define BCM283X_MU_LCR_DATA_SIZE_8	3

#define BCM283X_MU_LSR_TX_IDLE		BIT(6)
/* This actually means not full, but is named not empty in the docs */
#define BCM283X_MU_LSR_TX_EMPTY		BIT(5)
#define BCM283X_MU_LSR_RX_READY		BIT(0)

static struct bcm283x_mu_serial_platdata {
        unsigned long base;
        unsigned int clock;
        bool skip_init;
        bool disabled;
        struct bcm283x_mu_regs *regs;
} bcm283x_mu_serial_com = {0};

static int bcm283x_mu_serial_setbrg(struct bcm283x_mu_serial_platdata *plat, int baudrate)
{
	//struct bcm283x_mu_serial_platdata *plat = dev_get_platdata(dev);
	struct bcm283x_mu_regs *regs = plat->regs;
	u32 divider;

	if (plat->skip_init)
		return 0;

	divider = plat->clock / (baudrate * 8);

	writel(BCM283X_MU_LCR_DATA_SIZE_8, &regs->lcr);
	writel(divider - 1, &regs->baud);

	return 0;
}

static int bcm283x_mu_serial_probe(struct bcm283x_mu_serial_platdata *plat)
{
	//struct bcm283x_mu_serial_platdata *plat = dev_get_platdata(dev);

	if (plat->disabled)
		return -ENODEV;

	plat->regs = (struct bcm283x_mu_regs *)plat->base;

	return 0;
}

static int bcm283x_mu_serial_getc(struct bcm283x_mu_serial_platdata *plat)
{
	struct bcm283x_mu_regs *regs = plat->regs;
	u32 data;

	/* Wait until there is data in the FIFO */
	if (!(readl(&regs->lsr) & BCM283X_MU_LSR_RX_READY))
		return -EAGAIN;

	data = readl(&regs->io);

	return (int)data;
}

static int bcm283x_mu_serial_putc(struct bcm283x_mu_serial_platdata *plat, const char data)
{
	struct bcm283x_mu_regs *regs = plat->regs;

	/* Wait until there is space in the FIFO */
	if (!(readl(&regs->lsr) & BCM283X_MU_LSR_TX_EMPTY))
		return -EAGAIN;

	/* Send the character */
	writel(data, &regs->io);

	return 0;
}

static int bcm283x_mu_serial_pending(struct bcm283x_mu_serial_platdata *plat, bool input)
{
	struct bcm283x_mu_regs *regs = plat->regs;
	unsigned int lsr = readl(&regs->lsr);

	if (input) {
		/*WATCHDOG_RESET(); */
		return (lsr & BCM283X_MU_LSR_RX_READY) ? 1 : 0;
	} else {
		return (lsr & BCM283X_MU_LSR_TX_IDLE) ? 0 : 1;
	}
}

#if 0
static const struct dm_serial_ops bcm283x_mu_serial_ops = {
	.putc = bcm283x_mu_serial_putc,
	.pending = bcm283x_mu_serial_pending,
	.getc = bcm283x_mu_serial_getc,
	.setbrg = bcm283x_mu_serial_setbrg,
};
#endif

static int bcm_tx_ready(struct serial_port *port)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    return bcm283x_mu_serial_pending(uart, 0);
}

#if 0
static int bcm_rx_ready(struct serial_port *port)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    return bcm283x_mu_serial_pending(uart, 1);
}
#endif

static void bcm_putc(struct serial_port *port, char c)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    bcm283x_mu_serial_putc(uart, (char)c);
}

static int bcm_getc(struct serial_port *port, char *pc)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    *pc = bcm283x_mu_serial_getc(uart);
    return 1;
}


static struct uart_driver __read_mostly bcm_driver = {
    .tx_ready     = bcm_tx_ready,
    .putc         = bcm_putc,
    .getc         = bcm_getc,
    /*.start_tx     = pl011_tx_start, */
    /* .stop_tx      = pl011_tx_stop, */
    /* .vuart_info   = pl011_vuart, */
};

static int __init bcm_uart_init(int irq, u64 addr, u64 size, u32 clock, bool sbsa)
{
    struct bcm283x_mu_serial_platdata *uart;

    uart = &bcm283x_mu_serial_com;
#if 0
    uart->irq       = irq;
    uart->data_bits = 8;
    uart->parity    = PARITY_NONE;
    uart->stop_bits = 1;
    uart->sbsa      = sbsa;
#endif

    uart->regs = ioremap_nocache(addr, size);
    if ( !uart->regs )
    {
        printk("pl011: Unable to map the UART memory\n");
        return -ENOMEM;
    }

#if 0
    uart->vuart.base_addr = addr;
    uart->vuart.size = size;
    uart->vuart.data_off = DR;
    uart->vuart.status_off = FR;
    uart->vuart.status = 0;

#endif
    uart->clock = clock;
    bcm283x_mu_serial_probe(uart);
    bcm283x_mu_serial_setbrg(uart, 115200);
    /* Register with generic serial driver. */
    serial_register_uart(SERHND_DTUART, &bcm_driver, uart);

    return 0;
}

/* TODO: Parse UART config from the command line */
static int __init bcm_dt_uart_init(struct dt_device_node *dev,
                                     const void *data)
{
    const char *config = data;
    int res;
    u64 addr, size;
    u32 clock_frequency;

    if ( strcmp(config, "") )
    {
        printk("WARNING: UART configuration is not supported\n");
    }

    res = dt_device_get_address(dev, 0, &addr, &size);
    if ( res )
    {
        printk("bcm: Unable to retrieve the base"
               " address of the UART\n");
        return res;
    }

    res = platform_get_irq(dev, 0);
    if ( res < 0 )
    {
        printk("bcm: Unable to retrieve the IRQ\n");
        return -EINVAL;
    }

    res = dt_property_read_u32(dev, "clock",
                                            &clock_frequency);
    if ( res < 0 )
    {
        printk("bcm: Unable to retrieve clock freq\n");
        return -EINVAL;
    }


    res = bcm_uart_init(res, addr, size, clock_frequency, false);
    if ( res < 0 )
    {
        printk("bcm: Unable to initialize\n");
        return res;
    }

    dt_device_set_used_by(dev, DOMID_XEN);

    return 0;
}

static const struct dt_device_match bcm_dt_match[] __initconst =
{
    DT_MATCH_COMPATIBLE("arm,pl011"),
    DT_MATCH_COMPATIBLE("brcm,bcm283x-mu"),
    DT_MATCH_COMPATIBLE("brcm,bcm2835-aux-uart"),
    { /* sentinel */ },
};


DT_DEVICE_START(pl011, "PL011-custom UART", DEVICE_SERIAL)
        .dt_match = bcm_dt_match,
        .init = bcm_dt_uart_init,
DT_DEVICE_END
