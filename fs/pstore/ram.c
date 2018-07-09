/*
 * RAM Oops/Panic logger
 *
 * Copyright (C) 2010 Marco Stornelli <marco.stornelli@gmail.com>
 * Copyright (C) 2011 Kees Cook <keescook@chromium.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/pstore.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/compiler.h>
#include <linux/pstore_ram.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/bldr_debug_tools.h>
#include <linux/scatterlist.h>
#include <crypto/aead.h>

#define RAMOOPS_KERNMSG_HDR "===="
#define MIN_MEM_SIZE 4096UL

static ulong record_size = MIN_MEM_SIZE;
module_param(record_size, ulong, 0400);
MODULE_PARM_DESC(record_size,
		"size of each dump done on oops/panic");

static ulong ramoops_console_size = MIN_MEM_SIZE;
module_param_named(console_size, ramoops_console_size, ulong, 0400);
MODULE_PARM_DESC(console_size, "size of kernel console log");

static ulong ramoops_ftrace_size = MIN_MEM_SIZE;
module_param_named(ftrace_size, ramoops_ftrace_size, ulong, 0400);
MODULE_PARM_DESC(ftrace_size, "size of ftrace log");

static ulong ramoops_pmsg_size = MIN_MEM_SIZE;
module_param_named(pmsg_size, ramoops_pmsg_size, ulong, 0400);
MODULE_PARM_DESC(pmsg_size, "size of user space message log");

static unsigned long long mem_address;
module_param(mem_address, ullong, 0400);
MODULE_PARM_DESC(mem_address,
		"start of reserved RAM used to store oops/panic logs");

static ulong mem_size;
module_param(mem_size, ulong, 0400);
MODULE_PARM_DESC(mem_size,
		"size of reserved RAM used to store oops/panic logs");

static unsigned int mem_type;
module_param(mem_type, uint, 0600);
MODULE_PARM_DESC(mem_type,
		"set to 1 to try to use unbuffered memory (default 0)");

static int dump_oops = 1;
module_param(dump_oops, int, 0600);
MODULE_PARM_DESC(dump_oops,
		"set to 1 to dump oopses, 0 to only dump panics (default 1)");

static int ramoops_ecc;
module_param_named(ecc, ramoops_ecc, int, 0600);
MODULE_PARM_DESC(ramoops_ecc,
		"if non-zero, the option enables ECC support and specifies "
		"ECC buffer size in bytes (1 is a special value, means 16 "
		"bytes ECC)");

#define AES_KEY_MAX_LEN (256 / 8)
#define AES_KEY_IV_LEN 12
#define AES_KEY_TAG_LEN 16

struct ramoops_context {
	struct persistent_ram_zone **przs;
	struct persistent_ram_zone *cprz;
	struct persistent_ram_zone *fprz;
	struct persistent_ram_zone *mprz;
	phys_addr_t alt_phys_addr;
	phys_addr_t phys_addr;
	unsigned long size;
	unsigned int memtype;
	size_t record_size;
	size_t console_size;
	size_t ftrace_size;
	size_t pmsg_size;
	int dump_oops;
	struct persistent_ram_ecc_info ecc_info;
	unsigned int max_dump_cnt;
	unsigned int dump_write_cnt;
	/* _read_cnt need clear on ramoops_pstore_open */
	unsigned int dump_read_cnt;
	unsigned int console_read_cnt;
	unsigned int ftrace_read_cnt;
	unsigned int pmsg_read_cnt;
	struct pstore_info pstore;
	bool use_alt;
	bool need_update;
	int decrypt_state;
	int aes_key_len;
	unsigned char aes_key[AES_KEY_MAX_LEN];
	unsigned char aes_key_iv[AES_KEY_IV_LEN];
	unsigned char aes_key_tag[AES_KEY_TAG_LEN];
	struct class *ramoops_class;
	struct device *ramoops_device;
};

static struct platform_device *dummy;
static struct ramoops_platform_data *dummy_data;

static int ramoops_pstore_open(struct pstore_info *psi)
{
	struct ramoops_context *cxt = psi->data;

	cxt->dump_read_cnt = 0;
	cxt->console_read_cnt = 0;
	cxt->ftrace_read_cnt = 0;
	cxt->pmsg_read_cnt = 0;
	return 0;
}

static int ramoops_pstore_close(struct pstore_info *psi)
{
	struct ramoops_context *cxt = psi->data;

	cxt->need_update = 0;
	return 0;
}

static struct persistent_ram_zone *
ramoops_get_next_prz(struct persistent_ram_zone *przs[], uint *c, uint max,
		     u64 *id,
		     enum pstore_type_id *typep, enum pstore_type_id type,
		     bool update, bool use_alt)
{
	struct persistent_ram_zone *prz;
	int i = (*c)++;

	if (i >= max)
		return NULL;

	prz = przs[i];
	if (!prz)
		return NULL;

	/* Update old/shadowed buffer. */
	if (update)
		persistent_ram_save_old(prz, use_alt);

	if (!persistent_ram_old_size(prz))
		return NULL;

	*typep = type;
	*id = i;

	return prz;
}

static int ramoops_read_kmsg_hdr(char *buffer, struct timespec *time,
				  bool *compressed)
{
	char data_type;
	int header_length = 0;

	if (sscanf(buffer, RAMOOPS_KERNMSG_HDR "%lu.%lu-%c\n%n", &time->tv_sec,
			&time->tv_nsec, &data_type, &header_length) == 3) {
		if (data_type == 'C')
			*compressed = true;
		else
			*compressed = false;
	} else if (sscanf(buffer, RAMOOPS_KERNMSG_HDR "%lu.%lu\n%n",
			&time->tv_sec, &time->tv_nsec, &header_length) == 2) {
			*compressed = false;
	} else {
		time->tv_sec = 0;
		time->tv_nsec = 0;
		*compressed = false;
	}
	return header_length;
}

static bool prz_ok(struct persistent_ram_zone *prz)
{
	return !!prz && !!(persistent_ram_old_size(prz) +
			   persistent_ram_ecc_string(prz, NULL, 0));
}

static int ramoops_decrypt_alt(struct ramoops_context *cxt)
{
	struct crypto_aead *tfm;
	struct sg_table sgt;
	struct page **pages;
	unsigned int nr_pages;
	unsigned int i;
	int ret = 0;
	struct aead_request *aead_req;
	void *va;
	phys_addr_t start;
	size_t size;
	size_t buffer_size;
	void *buf;

	if (!cxt->alt_phys_addr)
		return -EINVAL;

	start = cxt->alt_phys_addr;
	size = cxt->size;
	if (!request_mem_region(start, size, "persistent_ram")) {
		pr_err("request mem region (0x%llx@0x%llx) failed\n",
			(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	if (cxt->memtype)
		va = ioremap(start, size);
	else
		va = ioremap_wc(start, size);
	if (va == NULL) {
		pr_err("ioremap mem region (0x%llx@0x%llx) failed\n",
			(unsigned long long)size, (unsigned long long)start);
		release_mem_region(start, size);
		ret = -ENOMEM;
		goto out_iomap;
	}

	tfm = crypto_alloc_aead("gcm(aes)", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		ret = PTR_ERR(tfm);
		pr_err("Failed to alloc aes, %d\n", ret);
		goto out_crypto_alloc;
	}

	ret = crypto_aead_setkey(tfm, cxt->aes_key, cxt->aes_key_len);
	if (ret) {
		pr_err("Failed to set key, %d\n", ret);
		goto out_set;
	}

	ret = crypto_aead_setauthsize(tfm, 16);
	if (ret) {
		pr_err("Failed to set auth size, %d\n", ret);
		goto out_set;
	}

	aead_req = aead_request_alloc(tfm, GFP_KERNEL);
	if (!aead_req) {
		ret = -ENOMEM;
		pr_err("Failed to allocate aead request\n");
		goto out_set;
	}

	buffer_size = size + AES_KEY_TAG_LEN;
	buf = vmalloc(buffer_size);
	if (!buf) {
		ret = -ENOMEM;
		pr_err("Failed to allocate bounce\n");
		goto out_no_buf;
	}

	memcpy_fromio(buf, va, size);
	memcpy(buf + size, cxt->aes_key_tag, AES_KEY_TAG_LEN);

	nr_pages = DIV_ROUND_UP(buffer_size, PAGE_SIZE);
	pages = kmalloc_array(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		pr_err("Failed to allocate page list\n");
		goto out_no_page_list;
	}

	for (i = 0; i < nr_pages; ++i)
		pages[i] = vmalloc_to_page(buf + (i * PAGE_SIZE));

	ret = sg_alloc_table_from_pages(&sgt, pages, nr_pages, 0,
					buffer_size, GFP_KERNEL);

	if (ret) {
		pr_err("Failed to allocate sg_table\n");
		goto out_no_sg_table;
	}

	aead_request_set_crypt(aead_req, sgt.sgl, sgt.sgl, buffer_size,
			       cxt->aes_key_iv);
	aead_request_set_ad(aead_req, 0);
	ret = crypto_aead_decrypt(aead_req);
	if (ret) {
		pr_err("Failed to decrypt %d\n", ret);
		goto out_decrypt;
	}
	memcpy_toio(va, buf, size);

out_decrypt:
	sg_free_table(&sgt);
out_no_sg_table:
	kfree(pages);
out_no_page_list:
	vfree(buf);
out_no_buf:
	aead_request_free(aead_req);
out_set:
	crypto_free_aead(tfm);
out_crypto_alloc:
	iounmap(va);
out_iomap:
	release_mem_region(start, size);
	return ret;
}

static ssize_t ramoops_pstore_read(u64 *id, enum pstore_type_id *type,
				   int *count, struct timespec *time,
				   char **buf, bool *compressed,
				   ssize_t *ecc_notice_size,
				   struct pstore_info *psi)
{
	ssize_t size;
	struct ramoops_context *cxt = psi->data;
	struct persistent_ram_zone *prz = NULL;
	int header_length = 0;
	bool use_alt = cxt->use_alt;
	bool update = cxt->need_update;

	/* Ramoops headers provide time stamps for PSTORE_TYPE_DMESG, but
	 * PSTORE_TYPE_CONSOLE and PSTORE_TYPE_FTRACE don't currently have
	 * valid time stamps, so it is initialized to zero.
	 */
	time->tv_sec = 0;
	time->tv_nsec = 0;
	*compressed = false;

	/* Find the next valid persistent_ram_zone for DMESG */
	while (cxt->dump_read_cnt < cxt->max_dump_cnt && !prz) {
		prz = ramoops_get_next_prz(cxt->przs, &cxt->dump_read_cnt,
					   cxt->max_dump_cnt, id, type,
					   PSTORE_TYPE_DMESG, 1, use_alt);
		if (!prz_ok(prz))
			continue;
		header_length = ramoops_read_kmsg_hdr(persistent_ram_old(prz),
						      time, compressed);
		/* Clear and skip this DMESG record if it has no valid header */
		if (!header_length) {
			persistent_ram_free_old(prz);
			persistent_ram_zap(prz, use_alt);
			prz = NULL;
		}
	}

	if (!prz_ok(prz))
		prz = ramoops_get_next_prz(&cxt->cprz, &cxt->console_read_cnt,
					   1, id, type, PSTORE_TYPE_CONSOLE,
					   update, use_alt);
	if (!prz_ok(prz))
		prz = ramoops_get_next_prz(&cxt->fprz, &cxt->ftrace_read_cnt,
					   1, id, type, PSTORE_TYPE_FTRACE,
					   update, use_alt);
	if (!prz_ok(prz))
		prz = ramoops_get_next_prz(&cxt->mprz, &cxt->pmsg_read_cnt,
					   1, id, type, PSTORE_TYPE_PMSG,
					   update, use_alt);
	if (!prz_ok(prz))
		return 0;

	size = persistent_ram_old_size(prz) - header_length;

	/* ECC correction notice */
	*ecc_notice_size = persistent_ram_ecc_string(prz, NULL, 0);

	*buf = vmalloc(size + *ecc_notice_size + 1);
	if (*buf == NULL)
		return -ENOMEM;

	memcpy(*buf, (char *)persistent_ram_old(prz) + header_length, size);
	persistent_ram_ecc_string(prz, *buf + size, *ecc_notice_size + 1);

	return size;
}

static size_t ramoops_write_kmsg_hdr(struct persistent_ram_zone *prz,
				     bool compressed)
{
	char *hdr;
	struct timespec timestamp;
	size_t len;

	/* Report zeroed timestamp if called before timekeeping has resumed. */
	if (__getnstimeofday(&timestamp)) {
		timestamp.tv_sec = 0;
		timestamp.tv_nsec = 0;
	}
	hdr = kasprintf(GFP_ATOMIC, RAMOOPS_KERNMSG_HDR "%lu.%lu-%c\n",
		(long)timestamp.tv_sec, (long)(timestamp.tv_nsec / 1000),
		compressed ? 'C' : 'D');
	WARN_ON_ONCE(!hdr);
	len = hdr ? strlen(hdr) : 0;
	persistent_ram_write(prz, hdr, len);
	kfree(hdr);

	return len;
}

static int notrace ramoops_pstore_write_buf(enum pstore_type_id type,
					    enum kmsg_dump_reason reason,
					    u64 *id, unsigned int part,
					    const char *buf,
					    bool compressed, size_t size,
					    struct pstore_info *psi)
{
	struct ramoops_context *cxt = psi->data;
	struct persistent_ram_zone *prz;
	size_t hlen;

	if (type == PSTORE_TYPE_CONSOLE) {
		if (!cxt->cprz)
			return -ENOMEM;
		persistent_ram_write(cxt->cprz, buf, size);
		return 0;
	} else if (type == PSTORE_TYPE_FTRACE) {
		if (!cxt->fprz)
			return -ENOMEM;
		persistent_ram_write(cxt->fprz, buf, size);
		return 0;
	} else if (type == PSTORE_TYPE_PMSG) {
		if (!cxt->mprz)
			return -ENOMEM;
		persistent_ram_write(cxt->mprz, buf, size);
		return 0;
	}

	if (type != PSTORE_TYPE_DMESG)
		return -EINVAL;

	/* Out of the various dmesg dump types, ramoops is currently designed
	 * to only store crash logs, rather than storing general kernel logs.
	 */
	if (reason != KMSG_DUMP_OOPS &&
	    reason != KMSG_DUMP_PANIC)
		return -EINVAL;

	/* Skip Oopes when configured to do so. */
	if (reason == KMSG_DUMP_OOPS && !cxt->dump_oops)
		return -EINVAL;

	/* Explicitly only take the first part of any new crash.
	 * If our buffer is larger than kmsg_bytes, this can never happen,
	 * and if our buffer is smaller than kmsg_bytes, we don't want the
	 * report split across multiple records.
	 */
	if (part != 1)
		return -ENOSPC;

	if (!cxt->przs)
		return -ENOSPC;

	prz = cxt->przs[cxt->dump_write_cnt];

	hlen = ramoops_write_kmsg_hdr(prz, compressed);
	if (size + hlen > prz->buffer_size)
		size = prz->buffer_size - hlen;
	persistent_ram_write(prz, buf, size);

	cxt->dump_write_cnt = (cxt->dump_write_cnt + 1) % cxt->max_dump_cnt;

	return 0;
}

static int notrace ramoops_pstore_write_buf_user(enum pstore_type_id type,
						 enum kmsg_dump_reason reason,
						 u64 *id, unsigned int part,
						 const char __user *buf,
						 bool compressed, size_t size,
						 struct pstore_info *psi)
{
	if (type == PSTORE_TYPE_PMSG) {
		struct ramoops_context *cxt = psi->data;

		if (!cxt->mprz)
			return -ENOMEM;
		return persistent_ram_write_user(cxt->mprz, buf, size);
	}

	return -EINVAL;
}

static int ramoops_pstore_erase(enum pstore_type_id type, u64 id, int count,
				struct timespec time, struct pstore_info *psi)
{
	struct ramoops_context *cxt = psi->data;
	struct persistent_ram_zone *prz;

	switch (type) {
	case PSTORE_TYPE_DMESG:
		if (id >= cxt->max_dump_cnt)
			return -EINVAL;
		prz = cxt->przs[id];
		break;
	case PSTORE_TYPE_CONSOLE:
		prz = cxt->cprz;
		break;
	case PSTORE_TYPE_FTRACE:
		prz = cxt->fprz;
		break;
	case PSTORE_TYPE_PMSG:
		prz = cxt->mprz;
		break;
	default:
		return -EINVAL;
	}

	persistent_ram_free_old(prz);
	persistent_ram_zap(prz, 0);

	return 0;
}

static struct ramoops_context oops_cxt = {
	.pstore = {
		.owner	= THIS_MODULE,
		.name	= "ramoops",
		.open	= ramoops_pstore_open,
		.read	= ramoops_pstore_read,
		.close  = ramoops_pstore_close,
		.write_buf	= ramoops_pstore_write_buf,
		.write_buf_user	= ramoops_pstore_write_buf_user,
		.erase	= ramoops_pstore_erase,
	},
};


static ssize_t use_alt_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct ramoops_context *cxt = &oops_cxt;
	return snprintf(buf, PAGE_SIZE, "%d\n", cxt->use_alt);
}

static ssize_t use_alt_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct ramoops_context *cxt = &oops_cxt;
	int use_alt = 0;

	if (kstrtoint(buf, 10, &use_alt) < 0)
		return -EINVAL;
	if (!cxt->use_alt && use_alt && !cxt->decrypt_state) {
		int ret = ramoops_decrypt_alt(cxt);
		if (ret < 0) {
			cxt->decrypt_state = ret;
			return ret;
		}
		cxt->decrypt_state = 1;
	}
	cxt->use_alt = use_alt;
	cxt->need_update = 1;
	return count;
}
static DEVICE_ATTR_RW(use_alt);

static ssize_t decrypt_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ramoops_context *cxt = &oops_cxt;
	return snprintf(buf, PAGE_SIZE, "%d\n", cxt->decrypt_state);
}
static DEVICE_ATTR_RO(decrypt_state);

static ssize_t aes_key_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct ramoops_context *cxt = &oops_cxt;

	if (count > AES_KEY_MAX_LEN)
		count = AES_KEY_MAX_LEN;
	if ((count != 16) && (count != 32))
		return -EINVAL;
	cxt->aes_key_len = count;
	memcpy(cxt->aes_key, buf, count);
	return count;
}
static DEVICE_ATTR_WO(aes_key);

static ssize_t aes_key_iv_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ramoops_context *cxt = &oops_cxt;

	if (count > AES_KEY_IV_LEN)
		count = AES_KEY_IV_LEN;
	memcpy(cxt->aes_key_iv, buf, count);
	return count;
}
static DEVICE_ATTR_WO(aes_key_iv);

static ssize_t aes_key_tag_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ramoops_context *cxt = &oops_cxt;

	if (count > AES_KEY_TAG_LEN)
		count = AES_KEY_TAG_LEN;
	memcpy(cxt->aes_key_tag, buf, count);
	return count;
}
static DEVICE_ATTR_WO(aes_key_tag);

static struct attribute *ramoops_attrs[] = {
	&dev_attr_use_alt.attr,
	&dev_attr_decrypt_state.attr,
	&dev_attr_aes_key.attr,
	&dev_attr_aes_key_iv.attr,
	&dev_attr_aes_key_tag.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ramoops);

static void ramoops_free_przs(struct ramoops_context *cxt)
{
	int i;

	if (!cxt->przs)
		return;

	for (i = 0; i < cxt->max_dump_cnt; i++)
		persistent_ram_free(cxt->przs[i]);

	kfree(cxt->przs);
	cxt->max_dump_cnt = 0;
}

static int ramoops_init_przs(struct device *dev, struct ramoops_context *cxt,
			     phys_addr_t *paddr, phys_addr_t *alt_paddr,
			     size_t dump_mem_sz)
{
	int err = -ENOMEM;
	int i;

	if (!cxt->record_size)
		return 0;

	if (*paddr + dump_mem_sz - cxt->phys_addr > cxt->size) {
		dev_err(dev, "no room for dumps\n");
		return -ENOMEM;
	}

	cxt->max_dump_cnt = dump_mem_sz / cxt->record_size;
	if (!cxt->max_dump_cnt)
		return -ENOMEM;

	cxt->przs = kzalloc(sizeof(*cxt->przs) * cxt->max_dump_cnt,
			     GFP_KERNEL);
	if (!cxt->przs) {
		dev_err(dev, "failed to initialize a prz array for dumps\n");
		goto fail_mem;
	}

	for (i = 0; i < cxt->max_dump_cnt; i++) {
		cxt->przs[i] = persistent_ram_new(*paddr, *alt_paddr,
						  cxt->record_size, 0,
						  &cxt->ecc_info,
						  cxt->memtype, 0);
		if (IS_ERR(cxt->przs[i])) {
			err = PTR_ERR(cxt->przs[i]);
			dev_err(dev, "failed to request mem region (0x%zx@0x%llx): %d\n",
				cxt->record_size, (unsigned long long)*paddr, err);

			while (i > 0) {
				i--;
				persistent_ram_free(cxt->przs[i]);
			}
			goto fail_prz;
		}
		*paddr += cxt->record_size;
		if (*alt_paddr)
			*alt_paddr += cxt->record_size;
	}

	return 0;
fail_prz:
	kfree(cxt->przs);
fail_mem:
	cxt->max_dump_cnt = 0;
	return err;
}

static int ramoops_init_prz(struct device *dev, struct ramoops_context *cxt,
			    struct persistent_ram_zone **prz,
			    phys_addr_t *paddr, phys_addr_t *alt_paddr,
			    size_t sz, u32 sig)
{
	if (!sz)
		return 0;

	if (*paddr + sz - cxt->phys_addr > cxt->size) {
		dev_err(dev, "no room for mem region (0x%zx@0x%llx) in (0x%lx@0x%llx)\n",
			sz, (unsigned long long)*paddr,
			cxt->size, (unsigned long long)cxt->phys_addr);
		return -ENOMEM;
	}

	*prz = persistent_ram_new(*paddr, *alt_paddr, sz, sig, &cxt->ecc_info,
				  cxt->memtype, 0);
	if (IS_ERR(*prz)) {
		int err = PTR_ERR(*prz);

		dev_err(dev, "failed to request mem region (0x%zx@0x%llx): %d\n",
			sz, (unsigned long long)*paddr, err);
		return err;
	}

	persistent_ram_zap(*prz, 0);

	*paddr += sz;
	if (*alt_paddr)
		*alt_paddr += sz;

	return 0;
}

static int ramoops_parse_dt_size(struct platform_device *pdev,
				 const char *propname, u32 *value)
{
	u32 val32 = 0;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, propname, &val32);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&pdev->dev, "failed to parse property %s: %d\n",
			propname, ret);
		return ret;
	}

	if (val32 > INT_MAX) {
		dev_err(&pdev->dev, "%s %u > INT_MAX\n", propname, val32);
		return -EOVERFLOW;
	}

	*value = val32;
	return 0;
}

static int ramoops_parse_dt(struct platform_device *pdev,
			    struct ramoops_platform_data *pdata)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *mem_region;
	struct resource res;
	size_t alt_mem_size;
	unsigned int alt_mem_type;
	u32 value;
	int ret;

	dev_dbg(&pdev->dev, "using Device Tree\n");

	mem_region = of_parse_phandle(of_node, "memory-region", 0);
	if (!mem_region) {
		dev_err(&pdev->dev, "no memory-region phandle\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(mem_region, 0, &res);
	of_node_put(mem_region);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to translate memory-region to resource: %d\n",
			ret);
		return ret;
	}

	pdata->mem_size = resource_size(&res);
	pdata->mem_address = res.start;
	pdata->mem_type = of_property_read_bool(of_node, "unbuffered");

	mem_region = of_parse_phandle(of_node, "alt-memory-region", 0);
	if (mem_region) {
		ret = of_address_to_resource(mem_region, 0, &res);
		of_node_put(mem_region);
		if (ret) {
			dev_err(&pdev->dev,
				"failed alt-memory-region to resource: %d\n",
				ret);
			return ret;
		}

		pdata->alt_mem_address = res.start;
		alt_mem_size = resource_size(&res);
		alt_mem_type = of_property_read_bool(of_node, "unbuffered");
		/* Alt memory region should be same size/type as orig. region */
		if (alt_mem_size != pdata->mem_size ||
		    alt_mem_type != pdata->mem_type) {
			dev_err(&pdev->dev,
				"alt region not the same size/type as main.\n");
			return -EINVAL;
		}
	}

	pdata->dump_oops = !of_property_read_bool(of_node, "no-dump-oops");

#define parse_size(name, field) {					\
		ret = ramoops_parse_dt_size(pdev, name, &value);	\
		if (ret < 0)						\
			return ret;					\
		field = value;						\
	}

	parse_size("record-size", pdata->record_size);
	parse_size("console-size", pdata->console_size);
	parse_size("ftrace-size", pdata->ftrace_size);
	parse_size("pmsg-size", pdata->pmsg_size);
	parse_size("ecc-size", pdata->ecc_info.ecc_size);

#undef parse_size

	return 0;
}

void notrace ramoops_console_write_buf(const char *buf, size_t size)
{
	struct ramoops_context *cxt = &oops_cxt;
	persistent_ram_write(cxt->cprz, buf, size);
}

static int ramoops_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ramoops_platform_data *pdata = dev->platform_data;
	struct ramoops_context *cxt = &oops_cxt;
	size_t dump_mem_sz;
	phys_addr_t paddr;
	phys_addr_t alt_paddr;
	int err = -EINVAL;

	if (dev_of_node(dev) && !pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto fail_out;
		}

		err = ramoops_parse_dt(pdev, pdata);
		if (err < 0)
			goto fail_out;
	}

	/* Only a single ramoops area allowed at a time, so fail extra
	 * probes.
	 */
	if (cxt->max_dump_cnt)
		goto fail_out;

	if (!pdata->mem_size || (!pdata->record_size && !pdata->console_size &&
			!pdata->ftrace_size && !pdata->pmsg_size)) {
		pr_err("The memory size and the record/console size must be "
			"non-zero\n");
		goto fail_out;
	}

	if (pdata->record_size && !is_power_of_2(pdata->record_size))
		pdata->record_size = rounddown_pow_of_two(pdata->record_size);
	if (pdata->console_size && !is_power_of_2(pdata->console_size))
		pdata->console_size = rounddown_pow_of_two(pdata->console_size);
	if (pdata->ftrace_size && !is_power_of_2(pdata->ftrace_size))
		pdata->ftrace_size = rounddown_pow_of_two(pdata->ftrace_size);
	if (pdata->pmsg_size && !is_power_of_2(pdata->pmsg_size))
		pdata->pmsg_size = rounddown_pow_of_two(pdata->pmsg_size);

	cxt->size = pdata->mem_size;
	cxt->phys_addr = pdata->mem_address;
	cxt->memtype = pdata->mem_type;
	cxt->alt_phys_addr = pdata->alt_mem_address;
	cxt->record_size = pdata->record_size;
	cxt->console_size = pdata->console_size;
	cxt->ftrace_size = pdata->ftrace_size;
	cxt->pmsg_size = pdata->pmsg_size;
	cxt->dump_oops = pdata->dump_oops;
	cxt->ecc_info = pdata->ecc_info;

	paddr = cxt->phys_addr;
	alt_paddr = cxt->alt_phys_addr;

	dump_mem_sz = cxt->size - cxt->console_size - cxt->ftrace_size
			- cxt->pmsg_size;
	err = ramoops_init_przs(dev, cxt, &paddr, &alt_paddr, dump_mem_sz);
	if (err)
		goto fail_out;

	err = ramoops_init_prz(dev, cxt, &cxt->cprz, &paddr, &alt_paddr,
			       cxt->console_size, 0);
	if (err)
		goto fail_init_cprz;

	err = ramoops_init_prz(dev, cxt, &cxt->fprz, &paddr, &alt_paddr,
			       cxt->ftrace_size, LINUX_VERSION_CODE);
	if (err)
		goto fail_init_fprz;

	err = ramoops_init_prz(dev, cxt, &cxt->mprz, &paddr, &alt_paddr,
			       cxt->pmsg_size, 0);
	if (err)
		goto fail_init_mprz;

	cxt->pstore.data = cxt;
	/*
	 * Console can handle any buffer size, so prefer LOG_LINE_MAX. If we
	 * have to handle dumps, we must have at least record_size buffer. And
	 * for ftrace, bufsize is irrelevant (if bufsize is 0, buf will be
	 * ZERO_SIZE_PTR).
	 */
	if (cxt->console_size)
		cxt->pstore.bufsize = 1024; /* LOG_LINE_MAX */
	cxt->pstore.bufsize = max(cxt->record_size, cxt->pstore.bufsize);
	cxt->pstore.buf = kmalloc(cxt->pstore.bufsize, GFP_KERNEL);
	if (!cxt->pstore.buf) {
		pr_err("cannot allocate pstore buffer\n");
		err = -ENOMEM;
		goto fail_clear;
	}
	spin_lock_init(&cxt->pstore.buf_lock);

	cxt->pstore.flags = PSTORE_FLAGS_DMESG;
	if (cxt->console_size)
		cxt->pstore.flags |= PSTORE_FLAGS_CONSOLE;
	if (cxt->ftrace_size)
		cxt->pstore.flags |= PSTORE_FLAGS_FTRACE;
	if (cxt->pmsg_size)
		cxt->pstore.flags |= PSTORE_FLAGS_PMSG;

	err = pstore_register(&cxt->pstore);
	if (err) {
		pr_err("registering with pstore failed\n");
		goto fail_buf;
	}

	/*
	 * Update the module parameter variables as well so they are visible
	 * through /sys/module/ramoops/parameters/
	 */
	mem_size = pdata->mem_size;
	mem_address = pdata->mem_address;
	record_size = pdata->record_size;
	dump_oops = pdata->dump_oops;
	ramoops_console_size = pdata->console_size;
	ramoops_pmsg_size = pdata->pmsg_size;
	ramoops_ftrace_size = pdata->ftrace_size;

	cxt->ramoops_class = class_create(THIS_MODULE, "ramoops");
	if (IS_ERR(cxt->ramoops_class)) {
		pr_err("device class file already in use\n");
		err = PTR_ERR(cxt->ramoops_class);
		goto fail_buf;
	}
	cxt->ramoops_class->dev_groups = ramoops_groups;

	cxt->ramoops_device = device_create(cxt->ramoops_class, NULL, 0,
					    NULL, "pstore");
	if (IS_ERR(cxt->ramoops_device)) {
		pr_err("failed to create device\n");
		err = PTR_ERR(cxt->ramoops_device);
		goto fail_device;
	}

	pr_info("attached 0x%lx@0x%llx, ecc: %d/%d\n",
		cxt->size, (unsigned long long)cxt->phys_addr,
		cxt->ecc_info.ecc_size, cxt->ecc_info.block_size);

	if (cxt->console_size)
		bldr_log_init();

	return 0;

fail_device:
	class_destroy(cxt->ramoops_class);
fail_buf:
	kfree(cxt->pstore.buf);
fail_clear:
	cxt->pstore.bufsize = 0;
	persistent_ram_free(cxt->mprz);
fail_init_mprz:
	persistent_ram_free(cxt->fprz);
fail_init_fprz:
	persistent_ram_free(cxt->cprz);
fail_init_cprz:
	ramoops_free_przs(cxt);
fail_out:
	return err;
}

static int ramoops_remove(struct platform_device *pdev)
{
	struct ramoops_context *cxt = &oops_cxt;

	device_unregister(cxt->ramoops_device);
	class_destroy(cxt->ramoops_class);
	pstore_unregister(&cxt->pstore);

	kfree(cxt->pstore.buf);
	cxt->pstore.bufsize = 0;

	persistent_ram_free(cxt->mprz);
	persistent_ram_free(cxt->fprz);
	persistent_ram_free(cxt->cprz);
	ramoops_free_przs(cxt);

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "ramoops" },
	{}
};

static struct platform_driver ramoops_driver = {
	.probe		= ramoops_probe,
	.remove		= ramoops_remove,
	.driver		= {
		.name		= "ramoops",
		.of_match_table	= dt_match,
	},
};

static void ramoops_register_dummy(void)
{
	if (!mem_size)
		return;

	pr_info("using module parameters\n");

	dummy_data = kzalloc(sizeof(*dummy_data), GFP_KERNEL);
	if (!dummy_data) {
		pr_info("could not allocate pdata\n");
		return;
	}

	dummy_data->mem_size = mem_size;
	dummy_data->mem_address = mem_address;
	dummy_data->mem_type = mem_type;
	dummy_data->record_size = record_size;
	dummy_data->console_size = ramoops_console_size;
	dummy_data->ftrace_size = ramoops_ftrace_size;
	dummy_data->pmsg_size = ramoops_pmsg_size;
	dummy_data->dump_oops = dump_oops;
	/*
	 * For backwards compatibility ramoops.ecc=1 means 16 bytes ECC
	 * (using 1 byte for ECC isn't much of use anyway).
	 */
	dummy_data->ecc_info.ecc_size = ramoops_ecc == 1 ? 16 : ramoops_ecc;

	dummy = platform_device_register_data(NULL, "ramoops", -1,
			dummy_data, sizeof(struct ramoops_platform_data));
	if (IS_ERR(dummy)) {
		pr_info("could not create platform device: %ld\n",
			PTR_ERR(dummy));
	}
}

static int __init ramoops_init(void)
{
	ramoops_register_dummy();
	return platform_driver_register(&ramoops_driver);
}
postcore_initcall(ramoops_init);

static void __exit ramoops_exit(void)
{
	platform_driver_unregister(&ramoops_driver);
	platform_device_unregister(dummy);
	kfree(dummy_data);
}
module_exit(ramoops_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marco Stornelli <marco.stornelli@gmail.com>");
MODULE_DESCRIPTION("RAM Oops/Panic logger/driver");
