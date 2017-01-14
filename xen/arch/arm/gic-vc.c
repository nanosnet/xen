/*
 * xen/arch/arm/gic-vc.c
 *
 * ARM VideoCore IV Interrupt Controller support
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

#include <xen/config.h>
#include <xen/lib.h>
#include <xen/init.h>
#include <xen/mm.h>
#include <xen/vmap.h>
#include <xen/irq.h>
#include <xen/iocap.h>
#include <xen/sched.h>
#include <xen/errno.h>
#include <xen/softirq.h>
#include <xen/list.h>
#include <xen/device_tree.h>
#include <xen/libfdt/libfdt.h>
#include <xen/sizes.h>
#include <xen/acpi.h>
#include <acpi/actables.h>
#include <asm/p2m.h>
#include <asm/domain.h>
#include <asm/platform.h>
#include <asm/device.h>

#include <asm/io.h>
#include <asm/gic.h>
#include <asm/gic-vc.h>
#include <asm/acpi.h>

static struct gic_info gicvc_info;

static paddr_t __initdata hbase, dbase, cbase, vbase;//, csize;

static void __init gicvc_dt_init(void)
{
    int res;
    //paddr_t vsize;
    const struct dt_device_node *node = gicvc_info.node;

    res = dt_device_get_address(node, 0, &dbase, NULL);
    printk("%s: dbase:%#lx\n", __func__, dbase);
    if ( res )
        panic("GICv2: Cannot find a valid address for the distributor");

#if 0
    res = dt_device_get_address(node, 1, &cbase, &csize);
    if ( res )
        panic("GICv2: Cannot find a valid address for the CPU");

    res = dt_device_get_address(node, 2, &hbase, NULL);
    if ( res )
        panic("GICv2: Cannot find a valid address for the hypervisor");

    res = dt_device_get_address(node, 3, &vbase, &vsize);
    if ( res )
        panic("GICv2: Cannot find a valid address for the virtual CPU");

    res = platform_get_irq(node, 0);
    if ( res < 0 )
        panic("GICv2: Cannot find the maintenance IRQ");
    gicv2_info.maintenance_irq = res;
#endif

    /* TODO: Add check on distributor */
}
static int __init gicvc_init(void)
{
    //uint32_t aliased_offset = 0;

    gicvc_dt_init();

    printk("GICvC initialization:\n"
              "        gic_dist_addr=%"PRIpaddr"\n"
              "        gic_cpu_addr=%"PRIpaddr"\n"
              "        gic_hyp_addr=%"PRIpaddr"\n"
              "        gic_vcpu_addr=%"PRIpaddr"\n"
              "        gic_maintenance_irq=%u\n",
              dbase, cbase, hbase, vbase,
              gicvc_info.maintenance_irq);

    if ( (dbase & ~PAGE_MASK) || (cbase & ~PAGE_MASK) ||
         (hbase & ~PAGE_MASK) || (vbase & ~PAGE_MASK) )
        panic("GICvC interfaces not page aligned");

    return 0;
}

const static struct gic_hw_operations gicvc_ops = {
    .info                = &gicvc_info,
    .init                = gicvc_init,
};


/* Set up the GIC */
static int __init gicvc_dt_preinit(struct dt_device_node *node,
                                   const void *data)
{
    gicvc_info.hw_version = GIC_VC;
    gicvc_info.node = node;
    register_gic_ops(&gicvc_ops);
    dt_irq_xlate = gic_irq_xlate;

    return 0;
}

static const struct dt_device_match gicvc_dt_match[] __initconst =
{
    /* DT_MATCH_GIC_V2, */
    DT_MATCH_COMPATIBLE("brcm,bcm2836-l1-intc"),
    { /* sentinel */ },
};

DT_DEVICE_START(gicvc, "GICv2", DEVICE_GIC)
        .dt_match = gicvc_dt_match,
        .init = gicvc_dt_preinit,
DT_DEVICE_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
