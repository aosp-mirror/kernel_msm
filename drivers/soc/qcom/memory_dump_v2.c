/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/qcom/memory_dump.h>
#include <soc/qcom/scm.h>

#define MSM_DUMP_TABLE_VERSION		MSM_DUMP_MAKE_VERSION(2, 0)

#define SCM_CMD_DEBUG_LAR_UNLOCK	0x4

struct msm_dump_table {
	uint32_t version;
	uint32_t num_entries;
	struct msm_dump_entry entries[MAX_NUM_ENTRIES];
};

struct msm_memory_dump {
	uint64_t table_phys;
	struct msm_dump_table *table;
};

static struct msm_memory_dump memdump;

#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
static uint64_t mem_dump_table_phys;
static struct msm_dump_table *dump_table_apps_addr;
#endif

uint32_t msm_dump_table_version(void)
{
	return MSM_DUMP_TABLE_VERSION;
}
EXPORT_SYMBOL(msm_dump_table_version);

static int msm_dump_table_register(struct msm_dump_entry *entry)
{
	struct msm_dump_entry *e;
	struct msm_dump_table *table = memdump.table;

	if (!table || table->num_entries >= MAX_NUM_ENTRIES)
		return -EINVAL;

	e = &table->entries[table->num_entries];
	e->id = entry->id;
	e->type = MSM_DUMP_TYPE_TABLE;
	e->addr = entry->addr;
	table->num_entries++;

	dmac_flush_range(table, (void *)table + sizeof(struct msm_dump_table));
	return 0;
}

static struct msm_dump_table *msm_dump_get_table(enum msm_dump_table_ids id)
{
	struct msm_dump_table *table = memdump.table;
	int i;

	if (!table) {
		pr_err("mem dump base table does not exist\n");
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < MAX_NUM_ENTRIES; i++) {
		if (table->entries[i].id == id)
			break;
	}
	if (i == MAX_NUM_ENTRIES || !table->entries[i].addr) {
		pr_err("mem dump base table entry %d invalid\n", id);
		return ERR_PTR(-EINVAL);
	}

	/* Get the apps table pointer */
#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	if(id == MSM_DUMP_TABLE_APPS)
		table = dump_table_apps_addr;
	else
		table = phys_to_virt(table->entries[i].addr);
#else
	table = phys_to_virt(table->entries[i].addr);
#endif

	return table;
}

int msm_dump_data_register(enum msm_dump_table_ids id,
			   struct msm_dump_entry *entry)
{
	struct msm_dump_entry *e;
	struct msm_dump_table *table;

	table = msm_dump_get_table(id);
	if (IS_ERR(table))
		return PTR_ERR(table);

	if (!table || table->num_entries >= MAX_NUM_ENTRIES)
		return -EINVAL;

	e = &table->entries[table->num_entries];
	e->id = entry->id;
	e->type = MSM_DUMP_TYPE_DATA;
	e->addr = entry->addr;
	table->num_entries++;

	dmac_flush_range(table, (void *)table + sizeof(struct msm_dump_table));
	return 0;
}
EXPORT_SYMBOL(msm_dump_data_register);

static int __init init_memory_dump(void)
{
	struct msm_dump_table *table;
	struct msm_dump_entry entry;
	struct device_node *np;
	void __iomem *imem_base;
	int ret;

	np = of_find_compatible_node(NULL, NULL,
				     "qcom,msm-imem-mem_dump_table");
	if (!np) {
		pr_err("mem dump base table DT node does not exist\n");
		return -ENODEV;
	}

#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	ret = of_property_read_u32(np,"htc,mem-dump-table-phys", (u32 *)&mem_dump_table_phys);
	if(ret) {
		pr_err("reading 'htc,mem-dump-table-phys' failed\n");
		return -ENXIO;
	}
#endif

	imem_base = of_iomap(np, 0);
	if (!imem_base) {
		pr_err("mem dump base table imem offset mapping failed\n");
		return -ENOMEM;
	}

#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	if (mem_dump_table_phys) {
		memdump.table = ioremap(mem_dump_table_phys, sizeof(struct msm_dump_table));
		if (!memdump.table) {
			pr_err("mem dump base table ioreamp failed\n");
			ret = -ENOMEM;
			goto err0;
		}
	}
#else
	memdump.table = kzalloc(sizeof(struct msm_dump_table), GFP_KERNEL);
	if (!memdump.table) {
		pr_err("mem dump base table allocation failed\n");
		ret = -ENOMEM;
		goto err0;
	}
#endif
	memdump.table->version = MSM_DUMP_TABLE_VERSION;
#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	memdump.table_phys = mem_dump_table_phys;
#else
	memdump.table_phys = virt_to_phys(memdump.table);
#endif
	memcpy_toio(imem_base, &memdump.table_phys, sizeof(memdump.table_phys));
	/* Ensure write to imem_base is complete before unmapping */
	mb();
	pr_info("MSM Memory Dump base table set up\n");

	iounmap(imem_base);

#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	table = ioremap(mem_dump_table_phys + sizeof(struct msm_dump_table), sizeof(struct msm_dump_table));
	if (!table) {
		pr_err("mem dump apps data table ioremap failed\n");
		ret = -ENOMEM;
		goto err1;
	}
	dump_table_apps_addr = table;
	/* memory dump table and MSM_DUMP_TAABLE_APPS are already initialized in LK, so just return */
	pr_info("MSM Memory Dump apps data table already set up in LK\n");
	return 0;
#else
	table = kzalloc(sizeof(struct msm_dump_table), GFP_KERNEL);
	if (!table) {
		pr_err("mem dump apps data table allocation failed\n");
		ret = -ENOMEM;
		goto err1;
	}
#endif
	table->version = MSM_DUMP_TABLE_VERSION;

	entry.id = MSM_DUMP_TABLE_APPS;
	entry.addr = virt_to_phys(table);
	ret = msm_dump_table_register(&entry);
	if (ret) {
		pr_info("mem dump apps data table register failed\n");
		goto err2;
	}
	pr_info("MSM Memory Dump apps data table set up\n");

	return 0;
err2:
	kfree(table);
err1:
#if defined(CONFIG_HTC_DEBUG_MEM_DUMP_TABLE)
	iounmap(memdump.table);
#else
	kfree(memdump.table);
#endif
	return ret;
err0:
	iounmap(imem_base);
	return ret;
}
early_initcall(init_memory_dump);

#ifdef CONFIG_MSM_DEBUG_LAR_UNLOCK
static int __init init_debug_lar_unlock(void)
{
	int ret;
	uint32_t argument = 0;
	struct scm_desc desc = {0};

	if (!is_scm_armv8())
		ret = scm_call(SCM_SVC_TZ, SCM_CMD_DEBUG_LAR_UNLOCK, &argument,
			       sizeof(argument), NULL, 0);
	else
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_TZ,
				SCM_CMD_DEBUG_LAR_UNLOCK), &desc);
	if (ret)
		pr_err("Core Debug Lock unlock failed, ret: %d\n", ret);
	else
		pr_info("Core Debug Lock unlocked\n");

	return ret;
}
early_initcall(init_debug_lar_unlock);
#endif
