/*
 * Jailhouse paravirt_ops implementation
 *
 * Copyright (c) Siemens AG, 2015-2017
 *
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <linux/acpi_pmtmr.h>
#include <linux/kernel.h>
#include <linux/serial_8250.h>
#include <asm/apic.h>
#include <asm/cpu.h>
#include <asm/hypervisor.h>
#include <asm/i8259.h>
#include <asm/pci_x86.h>
#include <asm/setup.h>

/*
 * The boot loader is passing platform information via a specific setup data
 * structure.
 */
struct jailhouse_setup_data {
	struct setup_data header;
	u16 pm_timer_address;
	u16 num_cpus;
	u8 cpu_ids[255];
};

static uint32_t jailhouse_cpuid_base(void)
{
	if (boot_cpu_data.cpuid_level < 0 ||
	    !boot_cpu_has(X86_FEATURE_HYPERVISOR))
		return 0;

	return hypervisor_cpuid_base("Jailhouse\0\0\0", 0);
}

static uint32_t __init jailhouse_detect(void)
{
	return jailhouse_cpuid_base();
}

static void jailhouse_get_wallclock(struct timespec *now)
{
	memset(now, 0, sizeof(*now));
}

#define MAX_RETRIES	5
#define SMI_TRESHOLD	50000

static unsigned long apic_timer_access(u64 *pmt, bool setup)
{
	unsigned long ret = 0;
	unsigned int n;
	u64 t1, t2;

	for (n = 0; n < MAX_RETRIES; n++) {
		t1 = get_cycles();
		*pmt = acpi_pm_read_early();
		if (setup)
			apic_write(APIC_TMICT, 0xffffffff);
		else
			ret = apic_read(APIC_TMCCT);
		t2 = get_cycles();

		if ((t2 - t1) < SMI_TRESHOLD * 2)
			return ret;
	}

	panic("SMI disturbed APIC timer calibration");
}

static void jailhouse_timer_init(void)
{
	u64 divided_apic_freq;
	unsigned long tmr;
	u64 start, end;

	if (boot_cpu_has(X86_FEATURE_TSC_DEADLINE_TIMER))
		return;

	apic_write(APIC_LVTT, APIC_LVT_MASKED);
	apic_write(APIC_TDCR, APIC_TDR_DIV_16);

	apic_timer_access(&start, true);
	while ((acpi_pm_read_early() - start) < 100000)
		cpu_relax();
	tmr = apic_timer_access(&end, false);

	divided_apic_freq =
		tsc_calc_pmtimer_ref((0xffffffffU - tmr) * 1000000, start, end);

	lapic_timer_frequency = divided_apic_freq * 16;
	apic_write(APIC_TMICT, 0);
}

static unsigned long jailhouse_calibrate_cpu(void)
{
	u64 tsc1, tsc2, pm1, pm2;
	unsigned long flags;

	local_irq_save(flags);

	tsc1 = tsc_read_refs(&pm1, 0);
	while ((get_cycles() - tsc1) < 50000000)
		cpu_relax();
	tsc2 = tsc_read_refs(&pm2, 0);

	local_irq_restore(flags);

	/* Check, whether the sampling was disturbed by an SMI */
	if (tsc1 == ULLONG_MAX || tsc2 == ULLONG_MAX)
		return 0;

	return tsc_calc_pmtimer_ref((tsc2 - tsc1) * 1000000, pm1, pm2);
}

static unsigned long jailhouse_calibrate_tsc(void)
{
	return 0;
}

static unsigned int x2apic_get_apic_id(unsigned long id)
{
        return id;
}

static void __init jailhouse_init_platform(void)
{
	/*
	 * The IO-APIC setup of Linux is not compatible with Jailhouse
	 * restrictions. Therefore, we cannot use a UART in IRQ mode. Leave
	 * irq 0 so that the driver enters a timer-driven polling.
	 */
	struct uart_port polled_uart0 = {
		.iobase = 0x3f8,
		.iotype = UPIO_PORT,
		.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF,
		.uartclk = 1843200,
	};
	u64 pa_data = boot_params.hdr.setup_data;
	struct jailhouse_setup_data *data;
	unsigned int cpu;

	x86_init.timers.timer_init	= jailhouse_timer_init;
	x86_init.irqs.pre_vector_init	= x86_init_noop;
	legacy_pic			= &null_legacy_pic;

	x86_platform.get_wallclock = jailhouse_get_wallclock;
	x86_platform.calibrate_cpu = jailhouse_calibrate_cpu;
	x86_platform.calibrate_tsc = jailhouse_calibrate_tsc;

	data = early_memremap(pa_data, sizeof(*data));
	pmtmr_ioport = data->pm_timer_address;
	printk(KERN_INFO "Jailhouse: PM-Timer IO Port: %#x\n", pmtmr_ioport);

	/*
	 * Register x2APIC handlers early. We need them when running
	 * register_lapic_address.
	 */
	if (x2apic_enabled()) {
#ifdef CONFIG_X86_X2APIC
		apic->read = native_apic_msr_read;
		apic->write = native_apic_msr_write;
#endif
		apic->get_apic_id = x2apic_get_apic_id;
	}
	register_lapic_address(0xfee00000);
	for (cpu = 0; cpu < data->num_cpus; cpu++)
		generic_processor_info(data->cpu_ids[cpu],
				       boot_cpu_apic_version);
	smp_found_config = 1;

	early_memunmap(data, sizeof(*data));

	pci_probe = 0;
	pci_direct_init(1);

	/*
	 * There are no bridges on the virtual PCI root bus under Jailhouse,
	 * thus no other way to discover all devices than a full scan.
	 */
	pcibios_last_bus = 0xff;

	/*
	 * Avoid that the kernel complains about missing ACPI tables - there
	 * are none in a non-root cell.
	 */
	disable_acpi();

	if (early_serial_setup(&polled_uart0) < 0)
		pr_warn("jailhouse: failed to register UART\n");
}

bool jailhouse_paravirt(void)
{
	return jailhouse_cpuid_base() != 0;
}

static bool jailhouse_x2apic_available(void)
{
	/*
	 * The x2APIC is only available if the root cell enabled it. Jailhouse
	 * does not support switching between xAPIC and x2APIC.
	 */
	return x2apic_enabled();
}

const struct hypervisor_x86 x86_hyper_jailhouse __refconst = {
	.name			= "Jailhouse",
	.detect			= jailhouse_detect,
	.init_platform		= jailhouse_init_platform,
	.x2apic_available	= jailhouse_x2apic_available,
};
