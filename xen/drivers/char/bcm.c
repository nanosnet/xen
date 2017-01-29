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
        struct irqaction irqaction;

	unsigned int data_bits, parity, stop_bits;
    	unsigned int irq;

	struct vuart_info vuart;
        struct bcm283x_mu_regs __iomem *regs;
} bcm283x_mu_serial_com = {0};

#define PARITY_NONE  (0)

static int bcm283x_mu_serial_setbrg(struct bcm283x_mu_serial_platdata *plat, int baudrate)
{
	//struct bcm283x_mu_serial_platdata *plat = dev_get_platdata(dev);
	struct bcm283x_mu_regs *regs = plat->regs;
	u32 divider;

	if (plat->skip_init) {
    		printk("%s:%u BLAHHH\n", __func__, __LINE__);
		return 0;
	}

	divider = plat->clock / (baudrate * 8);

	writel(BCM283X_MU_LCR_DATA_SIZE_8, &regs->lcr);
	writel(divider - 1, &regs->baud);

    	printk("%s:%u BLAHHH\n", __func__, __LINE__);
	return 0;
}

int bcm283x_mu_serial_probe(struct bcm283x_mu_serial_platdata *plat)
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

static unsigned int bcm_intr_status(struct bcm283x_mu_serial_platdata *uart)
{
    /* UARTMIS is not documented in SBSA v2.x, so use UARTRIS/UARTIMSC. */
    return (readl(&uart->regs->iir));
}

static void bcm_interrupt(int irq, void *data, struct cpu_user_regs *regs)
{
    struct serial_port *port = data;
    struct bcm283x_mu_serial_platdata *uart = port->uart;
    unsigned int status = bcm_intr_status(uart);
    printk("%s:%u BLAHHH\n", __func__, __LINE__);

    if ( status )
	printk("BLAHHHH\n");
    else
	printk("BLAHHHH NOOOOOOT\n");
}

static void __init bcm_init_preirq(struct serial_port *port)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;
    unsigned char lcr;
    //int rc;

    printk("%s:%u enter \n", __func__, __LINE__);
    lcr = (uart->data_bits - 5) | ((uart->stop_bits - 1) << 2) | uart->parity;
    readl(&uart->regs->msr);
    writel(lcr, &uart->regs->lcr);

    writel(0x7, &uart->regs->mcr);
    writel(0x7, &uart->regs->cntl);
    /*writel(0x3, &uart->regs->ier); */

    printk("%s:%u BLAHHH\n", __func__, __LINE__);
#if 0
    if ( uart->irq > 0 )
    {
        uart->irqaction.handler = bcm_interrupt;
        uart->irqaction.name    = "bcm283xirq";
        uart->irqaction.dev_id  = port;
        printk("%s:%u in irq > 0\n", __func__, __LINE__);
        if ( (rc = setup_irq(uart->irq, 0, &uart->irqaction)) != 0 )
            printk("ERROR: Failed to allocate bcm IRQ %d\n", uart->irq);
    }

#endif
    /*writel(0x6, &uart->regs->ier); */
    writel(0x2, &uart->regs->ier);
    printk("%s:%u exit\n", __func__, __LINE__);

}

static void __init bcm_init_postirq(struct serial_port *port)
{
#if 1
    struct bcm283x_mu_serial_platdata *uart = port->uart;
    int rc;

    /*writel(0x3, &uart->regs->ier); */
    printk("%s:%u BLAHHH\n", __func__, __LINE__);
    if ( uart->irq > 0 )
    {
        uart->irqaction.handler = bcm_interrupt;
        uart->irqaction.name    = "bcm283xirq";
        uart->irqaction.dev_id  = port;
        printk("%s:%u in irq > 0\n", __func__, __LINE__);
        if ( (rc = setup_irq(uart->irq, 0, &uart->irqaction)) != 0 )
            printk("ERROR: Failed to allocate bcm IRQ %d\n", uart->irq);
    }

    writel(0x6, &uart->regs->ier);
    writel(0x3, &uart->regs->ier);
#if 0
    /* Clear pending error interrupts */
    pl011_write(uart, ICR, OEI|BEI|PEI|FEI);

    /* Unmask interrupts */
    pl011_write(uart, IMSC, RTI|OEI|BEI|PEI|FEI|TXI|RXI);
#endif
#endif
}

static int __init bcm_irq(struct serial_port *port)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    return ((uart->irq > 0) ? uart->irq : -1);
}

static void bcm_suspend(struct serial_port *port)
{
    BUG();
}

static void bcm_resume(struct serial_port *port)
{
    BUG();
}


static const struct vuart_info *bcm_vuart(struct serial_port *port)
{
    struct bcm283x_mu_serial_platdata *uart = port->uart;

    return &uart->vuart;
}

static struct uart_driver __read_mostly bcm_driver = {
    .init_preirq  = bcm_init_preirq,
    .init_postirq = bcm_init_postirq,
    .endboot      = NULL,
    .suspend      = bcm_suspend,
    .resume       = bcm_resume,
    .tx_ready     = bcm_tx_ready,
    .putc         = bcm_putc,
    .getc         = bcm_getc,
    .irq          = bcm_irq,
    .vuart_info   = bcm_vuart,

};

static int __init bcm_uart_init(int irq, u64 addr, u64 size, u32 clock, u32 baudrate)
{
    struct bcm283x_mu_serial_platdata *uart;
    //struct serial_port *port;
    //unsigned char lcr;

    uart = &bcm283x_mu_serial_com;

#if 1
    uart->irq       = irq;
    uart->data_bits = 8;
    uart->parity    = PARITY_NONE;
    uart->stop_bits = 1;
#endif

    printk("irq: %u addr:%#lx, size:%lu\n", irq, addr, size);
    uart->regs = ioremap_nocache(addr, size);
    if ( !uart->regs )
    {
        printk("bcm: Unable to map the UART memory\n");
        return -ENOMEM;
    }
    printk("regs:%p addr:%#lx, size:%lu\n", uart->regs, addr, size);

    printk("clock:%#x\n", clock);
    /*uart->skip_init = 1; */
    uart->clock = clock;
    printk("baud:%u\n", baudrate);
    bcm283x_mu_serial_setbrg(uart, baudrate);

    /*writel(0x0, &uart->regs->ier); 
    writel(0x7, &uart->regs->mcr);
    writel(0x7, &uart->regs->cntl); */
    /* Register with generic serial driver. */
    serial_register_uart(SERHND_DTUART, &bcm_driver, uart);

    return 0;
}

static int __init bcm_dt_uart_init(struct dt_device_node *dev,
                                     const void *data)
{
    const char *config = data;
    int res;
    u64 addr, size;
    u32 clock_frequency;
    u32 baudrate;

    /* FIXME */
    if ( strcmp(config, "") )
    {
        printk("WARNING: UART configuration is not yet supported\n");
    }

    res = dt_device_get_address(dev, 0, &addr, &size);
    if ( res )
    {
        printk("bcm: Unable to retrieve the base"
               " address of the UART\n");
        return res;
    }

    res = dt_property_read_u32(dev, "baudrate", &baudrate);
    if ( res < 0 )
    {
        baudrate = 115200;
        printk("bcm: Unable to retrieve baudrate, defaulting to %u\n", baudrate);
    }

    res = dt_property_read_u32(dev, "clock-frequency", &clock_frequency);
    if ( res < 0 )
    {
        printk("bcm: Unable to retrieve clock freq\n");
        return -EINVAL;
    }

    res = platform_get_irq(dev, 0);
    if ( res < 0 )
    {
        printk("bcm: Unable to retrieve the IRQ\n");
        return -ENODEV;
    }

    res = bcm_uart_init(res, addr, size, clock_frequency, baudrate);
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
    DT_MATCH_COMPATIBLE("brcm,bcm283x-mu"),
    DT_MATCH_COMPATIBLE("brcm,bcm2835-aux-uart"),
    { /* sentinel */ },
};


DT_DEVICE_START(bcm, "BCM-custom MU", DEVICE_SERIAL)
        .dt_match = bcm_dt_match,
        .init = bcm_dt_uart_init,
DT_DEVICE_END
