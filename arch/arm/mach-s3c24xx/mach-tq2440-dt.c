#include <linux/clocksource.h>
#include <linux/irqchip.h>

#include <asm/mach/arch.h>
#include <mach/map.h>

#include <plat/cpu.h>
#include <plat/pm.h>

#include "common.h"

static void __init tq2440_dt_map_io(void)
{
	s3c24xx_init_io(NULL, 0);
}

static void __init tq2440_dt_machine_init(void)
{
	s3c_pm_init();
}

static const char *const tq2440_dt_compat[] __initconst = {
	"samsung,s3c2440",
	"samsung,tq2440",
	NULL
};

DT_MACHINE_START(TQ2440_DT, "Samsung S3C2440 (Flattened Device Tree)")
	.dt_compat	= tq2440_dt_compat,
	.map_io		= tq2440_dt_map_io,
	.init_irq	= irqchip_init,
	.init_machine	= tq2440_dt_machine_init,
MACHINE_END
