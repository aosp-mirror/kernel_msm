/*
 * arch/arm64/kernel/topology.c
 *
 * Copyright (C) 2011,2013,2014 Linaro Limited.
 *
 * Based on the arm32 version written by Vincent Guittot in turn based on
 * arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/node.h>
#include <linux/nodemask.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/cputype.h>
#include <asm/topology.h>

static DEFINE_PER_CPU(unsigned long, cpu_scale) = SCHED_CAPACITY_SCALE;

unsigned long scale_cpu_capacity(struct sched_domain *sd, int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

static void set_capacity_scale(unsigned int cpu, unsigned long capacity)
{
	per_cpu(cpu_scale, cpu) = capacity;
}

static int __init get_cpu_for_node(struct device_node *node)
{
	struct device_node *cpu_node;
	int cpu;

	cpu_node = of_parse_phandle(node, "cpu", 0);
	if (!cpu_node)
		return -1;

	for_each_possible_cpu(cpu) {
		if (of_get_cpu_node(cpu, NULL) == cpu_node) {
			of_node_put(cpu_node);
			return cpu;
		}
	}

	pr_crit("Unable to find CPU node for %s\n", cpu_node->full_name);

	of_node_put(cpu_node);
	return -1;
}

static int __init parse_core(struct device_node *core, int cluster_id,
			     int core_id)
{
	char name[10];
	bool leaf = true;
	int i = 0;
	int cpu;
	struct device_node *t;

	do {
		snprintf(name, sizeof(name), "thread%d", i);
		t = of_get_child_by_name(core, name);
		if (t) {
			leaf = false;
			cpu = get_cpu_for_node(t);
			if (cpu >= 0) {
				cpu_topology[cpu].cluster_id = cluster_id;
				cpu_topology[cpu].core_id = core_id;
				cpu_topology[cpu].thread_id = i;
			} else {
				pr_err("%s: Can't get CPU for thread\n",
				       t->full_name);
				of_node_put(t);
				return -EINVAL;
			}
			of_node_put(t);
		}
		i++;
	} while (t);

	cpu = get_cpu_for_node(core);
	if (cpu >= 0) {
		if (!leaf) {
			pr_err("%s: Core has both threads and CPU\n",
			       core->full_name);
			return -EINVAL;
		}

		cpu_topology[cpu].cluster_id = cluster_id;
		cpu_topology[cpu].core_id = core_id;
	} else if (leaf) {
		pr_err("%s: Can't get CPU for leaf core\n", core->full_name);
		return -EINVAL;
	}

	return 0;
}

static int __init parse_cluster(struct device_node *cluster, int depth)
{
	char name[10];
	bool leaf = true;
	bool has_cores = false;
	struct device_node *c;
	static int cluster_id __initdata;
	int core_id = 0;
	int i, ret;

	/*
	 * First check for child clusters; we currently ignore any
	 * information about the nesting of clusters and present the
	 * scheduler with a flat list of them.
	 */
	i = 0;
	do {
		snprintf(name, sizeof(name), "cluster%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			leaf = false;
			ret = parse_cluster(c, depth + 1);
			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	/* Now check for cores */
	i = 0;
	do {
		snprintf(name, sizeof(name), "core%d", i);
		c = of_get_child_by_name(cluster, name);
		if (c) {
			has_cores = true;

			if (depth == 0) {
				pr_err("%s: cpu-map children should be clusters\n",
				       c->full_name);
				of_node_put(c);
				return -EINVAL;
			}

			if (leaf) {
				ret = parse_core(c, cluster_id, core_id++);
			} else {
				pr_err("%s: Non-leaf cluster with core %s\n",
				       cluster->full_name, name);
				ret = -EINVAL;
			}

			of_node_put(c);
			if (ret != 0)
				return ret;
		}
		i++;
	} while (c);

	if (leaf && !has_cores)
		pr_warn("%s: empty cluster\n", cluster->full_name);

	if (leaf)
		cluster_id++;

	return 0;
}

static int __init parse_dt_topology(void)
{
	struct device_node *cn, *map;
	int ret = 0;
	int cpu;

	cn = of_find_node_by_path("/cpus");
	if (!cn) {
		pr_err("No CPU information found in DT\n");
		return 0;
	}

	/*
	 * When topology is provided cpu-map is essentially a root
	 * cluster with restricted subnodes.
	 */
	map = of_get_child_by_name(cn, "cpu-map");
	if (!map)
		goto out;

	ret = parse_cluster(map, 0);
	if (ret != 0)
		goto out_map;

	/*
	 * Check that all cores are in the topology; the SMP code will
	 * only mark cores described in the DT as possible.
	 */
	for_each_possible_cpu(cpu)
		if (cpu_topology[cpu].cluster_id == -1)
			ret = -EINVAL;

out_map:
	of_node_put(map);
out:
	of_node_put(cn);
	return ret;
}

/*
 * cpu topology table
 */
struct cpu_topology cpu_topology[NR_CPUS];
EXPORT_SYMBOL_GPL(cpu_topology);

/*
 * ARM JUNO specific energy cost model data. There are no unit requirements for
 * the data. Data can be normalized to any reference point, but the
 * normalization must be consistent. That is, one bogo-joule/watt must be the
 * same quantity for all data, but we don't care what it is.
 */

static struct idle_state idle_states_cluster_a53[] = {
	{ .power = 56 }, /* arch_cpu_idle() (active idle) = WFI */
	{ .power = 56 }, /* WFI */
	{ .power = 56 }, /* cpu-sleep-0 */
	{ .power = 17 }, /* cluster-sleep-0 */
};

static struct idle_state idle_states_cluster_a57[] = {
	{ .power = 65 }, /* arch_cpu_idle() (active idle) = WFI */
	{ .power = 65 }, /* WFI */
	{ .power = 65 }, /* cpu-sleep-0 */
	{ .power = 24 }, /* cluster-sleep-0 */
};

static struct capacity_state cap_states_cluster_a53[] = {
        /* Power per cluster */
	{ .cap =  235, .power = 26, }, /*  450 MHz */
	{ .cap =  303, .power = 30, }, /*  575 MHz */
	{ .cap =  368, .power = 39, }, /*  700 MHz */
	{ .cap =  406, .power = 47, }, /*  775 MHz */
	{ .cap =  447, .power = 57, }, /*  850 Mhz */
};

static struct capacity_state cap_states_cluster_a57[] = {
        /* Power per cluster */
	{ .cap =  417, .power = 24, }, /*  450 MHz */
	{ .cap =  579, .power = 32, }, /*  625 MHz */
	{ .cap =  744, .power = 43, }, /*  800 MHz */
	{ .cap =  883, .power = 49, }, /*  950 MHz */
	{ .cap = 1024, .power = 64, }, /* 1100 MHz */
};

static struct sched_group_energy energy_cluster_a53 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_cluster_a53),
	.idle_states    = idle_states_cluster_a53,
	.nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a53),
	.cap_states     = cap_states_cluster_a53,
};

static struct sched_group_energy energy_cluster_a57 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_cluster_a57),
	.idle_states    = idle_states_cluster_a57,
	.nr_cap_states  = ARRAY_SIZE(cap_states_cluster_a57),
	.cap_states     = cap_states_cluster_a57,
};

static struct idle_state idle_states_core_a53[] = {
	{ .power = 6 }, /* arch_cpu_idle() (active idle) = WFI */
	{ .power = 6 }, /* WFI */
	{ .power = 0 }, /* cpu-sleep-0 */
	{ .power = 0 }, /* cluster-sleep-0 */
};

static struct idle_state idle_states_core_a57[] = {
	{ .power = 15 }, /* arch_cpu_idle() (active idle) = WFI */
	{ .power = 15 }, /* WFI */
	{ .power = 0  }, /* cpu-sleep-0 */
	{ .power = 0  }, /* cluster-sleep-0 */
};

static struct capacity_state cap_states_core_a53[] = {
        /* Power per cpu */
	{ .cap =  235, .power =  33, }, /*  450 MHz */
	{ .cap =  302, .power =  46, }, /*  575 MHz */
	{ .cap =  368, .power =  61, }, /*  700 MHz */
	{ .cap =  406, .power =  76, }, /*  775 MHz */
	{ .cap =  447, .power =  93, }, /*  850 Mhz */
};

static struct capacity_state cap_states_core_a57[] = {
        /* Power per cpu */
	{ .cap =  417, .power = 168, }, /*  450 MHz */
	{ .cap =  579, .power = 251, }, /*  625 MHz */
	{ .cap =  744, .power = 359, }, /*  800 MHz */
	{ .cap =  883, .power = 479, }, /*  950 MHz */
	{ .cap = 1024, .power = 616, }, /* 1100 MHz */
};

static struct sched_group_energy energy_core_a53 = {
	.nr_idle_states = ARRAY_SIZE(idle_states_core_a53),
	.idle_states    = idle_states_core_a53,
	.nr_cap_states  = ARRAY_SIZE(cap_states_core_a53),
	.cap_states     = cap_states_core_a53,
};

static struct sched_group_energy energy_core_a57 = {
	  .nr_idle_states = ARRAY_SIZE(idle_states_core_a57),
	  .idle_states    = idle_states_core_a57,
	  .nr_cap_states  = ARRAY_SIZE(cap_states_core_a57),
	  .cap_states     = cap_states_core_a57,
};

/* sd energy functions */
static inline
const struct sched_group_energy * const cpu_cluster_energy(int cpu)
{
	return cpu_topology[cpu].cluster_id ? &energy_cluster_a53 :
			&energy_cluster_a57;
}

static inline
const struct sched_group_energy * const cpu_core_energy(int cpu)
{
	return cpu_topology[cpu].cluster_id ? &energy_core_a53 :
			&energy_core_a57;
}

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	return &cpu_topology[cpu].core_sibling;
}

static inline int cpu_corepower_flags(void)
{
	return SD_SHARE_PKG_RESOURCES  | SD_SHARE_POWERDOMAIN | \
	       SD_SHARE_CAP_STATES;
}

static struct sched_domain_topology_level arm64_topology[] = {
#ifdef CONFIG_SCHED_MC
	{ cpu_coregroup_mask, cpu_corepower_flags, cpu_core_energy, SD_INIT_NAME(MC) },
#endif
	{ cpu_cpu_mask, NULL, cpu_cluster_energy, SD_INIT_NAME(DIE) },
	{ NULL, },
};

static void update_cpu_capacity(unsigned int cpu)
{
	unsigned long capacity = SCHED_CAPACITY_SCALE;

	if (cpu_core_energy(cpu)) {
		int max_cap_idx = cpu_core_energy(cpu)->nr_cap_states - 1;
		capacity = cpu_core_energy(cpu)->cap_states[max_cap_idx].cap;
	}

	set_capacity_scale(cpu, capacity);

	pr_info("CPU%d: update cpu_capacity %lu\n",
		cpu, arch_scale_cpu_capacity(NULL, cpu));
}

static void update_siblings_masks(unsigned int cpuid)
{
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->cluster_id != cpu_topo->cluster_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->core_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->core_sibling);

		if (cpuid_topo->core_id != cpu_topo->core_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->thread_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->thread_sibling);
	}
}

void store_cpu_topology(unsigned int cpuid)
{
	struct cpu_topology *cpuid_topo = &cpu_topology[cpuid];
	u64 mpidr;

	if (cpuid_topo->cluster_id != -1)
		goto topology_populated;

	mpidr = read_cpuid_mpidr();

	/* Uniprocessor systems can rely on default topology values */
	if (mpidr & MPIDR_UP_BITMASK)
		return;

	/* Create cpu topology mapping based on MPIDR. */
	if (mpidr & MPIDR_MT_BITMASK) {
		/* Multiprocessor system : Multi-threads per core */
		cpuid_topo->thread_id  = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		cpuid_topo->core_id    = MPIDR_AFFINITY_LEVEL(mpidr, 1);
		cpuid_topo->cluster_id = MPIDR_AFFINITY_LEVEL(mpidr, 2);
	} else {
		/* Multiprocessor system : Single-thread per core */
		cpuid_topo->thread_id  = -1;
		cpuid_topo->core_id    = MPIDR_AFFINITY_LEVEL(mpidr, 0);
		cpuid_topo->cluster_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	}

	pr_debug("CPU%u: cluster %d core %d thread %d mpidr %#016llx\n",
		 cpuid, cpuid_topo->cluster_id, cpuid_topo->core_id,
		 cpuid_topo->thread_id, mpidr);

topology_populated:
	update_siblings_masks(cpuid);
	update_cpu_capacity(cpuid);
}

static void __init reset_cpu_topology(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpu_topology *cpu_topo = &cpu_topology[cpu];

		cpu_topo->thread_id = -1;
		cpu_topo->core_id = 0;
		cpu_topo->cluster_id = -1;

		cpumask_clear(&cpu_topo->core_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->core_sibling);
		cpumask_clear(&cpu_topo->thread_sibling);
		cpumask_set_cpu(cpu, &cpu_topo->thread_sibling);
	}
}

void __init init_cpu_topology(void)
{
	reset_cpu_topology();

	/*
	 * Discard anything that was parsed if we hit an error so we
	 * don't use partial information.
	 */
	if (parse_dt_topology())
		reset_cpu_topology();
	else
		set_sched_topology(arm64_topology);
}
