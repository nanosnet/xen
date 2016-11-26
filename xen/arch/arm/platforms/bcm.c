/*
 * xen/arch/arm/platforms/brcm.c
 *
 * Broadcom Platform startup.
 *
 * Jon Fraser  <jfraser@broadcom.com>
 * Copyright (c) 2013-2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/platform.h>
#include <xen/mm.h>
#include <xen/vmap.h>
#include <asm/io.h>
#include <xen/delay.h>

struct brcm_plat_regs {
    uint32_t    hif_mask;
    uint32_t    hif_cpu_reset_config;
    uint32_t    hif_boot_continuation;
    uint32_t    cpu0_pwr_zone_ctrl;
    uint32_t    scratch_reg;
};


/*static struct brcm_plat_regs regs; */

__init int brcm_get_dt_node(char *compat_str,
                                   const struct dt_device_node **dn,
                                   u32 *reg_base)
{
    const struct dt_device_node *node;
    u64 reg_base_64;
    int rc;

    node = dt_find_compatible_node(NULL, NULL, compat_str);
    if ( !node )
    {
        dprintk(XENLOG_ERR, "%s: missing \"%s\" node\n", __func__, compat_str);
        return -ENOENT;
    }

    rc = dt_device_get_address(node, 0, &reg_base_64, NULL);
    if ( rc )
    {
        dprintk(XENLOG_ERR, "%s: missing \"reg\" prop\n", __func__);
        return rc;
    }

    if ( dn )
        *dn = node;

    if ( reg_base )
        *reg_base = reg_base_64;

    return 0;
}

static __init int brcm_populate_plat_regs(void)
{
    int rc;
    const struct dt_device_node *node;
    u64 reg_base;
    u64 *reg_base_va;
    u64 size;
    u32 prescaler, cntfreq;

    node = dt_find_compatible_node(NULL, NULL, "brcm,bcm2836-l1-intc");
    if (!node) {
        printk("%s: enodev \n", __func__);
        return -ENODEV;
    }
    
    rc = dt_device_get_address(node, 0, &reg_base, &size);
    if ( rc )
    {
        printk("%s: missing \"reg\" prop\n", __func__);
        return rc;
    }
    printk("%s: regbase:%#lx, %lu size\n", __func__, reg_base, size);
    
    reg_base_va = ioremap_nocache(reg_base, size);
    if ( !reg_base_va)
    {
        printk("Unable to map reg_base\n");
        return -ENOMEM;
    }

#define mrs(spr)                ({ u64 rval; asm volatile(\
                                "mrs %0," #spr :"=r"(rval)); rval; })

    cntfreq = mrs(cntfrq_el0);
    switch (cntfreq) {
        case 19200000:
                prescaler = 0x80000000;
        case 1000000:
                prescaler = 0x06AAAAAB;
        default:
                prescaler = (u32)((u64)0x80000000 * (u64)cntfreq/
                                        (u64)19200000);
                break;
    };

    if (!prescaler) {
	printk("prescaler error\n");
        return rc;
    }
    writel(prescaler, reg_base_va + 0x008); /* LOCAL_TIMER_PRESCALER */



#if 0
    rc = brcm_get_dt_node("brcm,brcmstb-cpu-biu-ctrl", &node, &reg_base);
    if ( rc )
        return rc;

    if ( !dt_property_read_u32(node, "cpu-reset-config-reg", &val) )
    {
        dprintk(XENLOG_ERR, "Missing property \"cpu-reset-config-reg\"\n");
        return -ENOENT;
    }
    regs.hif_cpu_reset_config = reg_base + val;

    if ( !dt_property_read_u32(node, "cpu0-pwr-zone-ctrl-reg", &val) )
    {
        dprintk(XENLOG_ERR, "Missing property \"cpu0-pwr-zone-ctrl-reg\"\n");
        return -ENOENT;
    }
    regs.cpu0_pwr_zone_ctrl = reg_base + val;

    if ( !dt_property_read_u32(node, "scratch-reg", &val) )
    {
        dprintk(XENLOG_ERR, "Missing property \"scratch-reg\"\n");
        return -ENOENT;
    }
    regs.scratch_reg = reg_base + val;

    rc = brcm_get_dt_node("brcm,brcmstb-hif-continuation", NULL, &reg_base);
    if ( rc )
        return rc;

    regs.hif_boot_continuation = reg_base;

    dprintk(XENLOG_INFO, "hif_cpu_reset_config  : %08xh\n",
                    regs.hif_cpu_reset_config);
    dprintk(XENLOG_INFO, "cpu0_pwr_zone_ctrl    : %08xh\n",
                    regs.cpu0_pwr_zone_ctrl);
    dprintk(XENLOG_INFO, "hif_boot_continuation : %08xh\n",
                    regs.hif_boot_continuation);
    dprintk(XENLOG_INFO, "scratch_reg : %08xh\n",
                    regs.scratch_reg);
#endif

    return 0;
}

static __init int brcm_init(void)
{
    return brcm_populate_plat_regs();
}

static const char const *brcm_dt_compat[] __initconst =
{
    "brcm,bcm2837",
    NULL
};

PLATFORM_START(brcm, "Broadcom BCM2837")
    .compatible     = brcm_dt_compat,
    .init           = brcm_init,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
