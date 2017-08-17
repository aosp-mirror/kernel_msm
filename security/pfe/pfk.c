/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

/*
 * Per-File-Key (PFK).
 *
 * PFK exposes API's for loading and removing keys from encryption hw
 * and also API to determine whether 2 adjacent blocks can be agregated by
 * Block Layer in one request to encryption hw.
 *
 * Please note, the only API that uses EXPORT_SYMBOL() is pfk_remove_key,
 * this is intentionally, as it is the only API that is intended to be used
 * by any kernel module, including dynamically loaded ones.
 */


/* Uncomment the line below to enable debug messages */
/* #define DEBUG 1 */
#define pr_fmt(fmt)	"pfk [%s]: " fmt, __func__

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/bio.h>
#include <linux/security.h>
#include <crypto/ice.h>

#include <linux/pfk.h>

#include "pfk_kc.h"
#include "objsec.h"
#include "pfk_ice.h"

static DEFINE_MUTEX(pfk_lock);
static bool pfk_ready;


/* might be replaced by a table when more than one cipher is supported */
#define PFK_SUPPORTED_CIPHER "aes_xts"
#define PFK_SUPPORTED_KEY_SIZE 32
#define PFK_SUPPORTED_SALT_SIZE 32


static int pfk_inode_alloc_security(struct inode *inode)
{
	struct inode_security_struct *i_sec = NULL;

	if (inode == NULL)
		return -EINVAL;

	i_sec = kzalloc(sizeof(*i_sec), GFP_KERNEL);

	if (i_sec == NULL)
		return -ENOMEM;

	inode->i_security = i_sec;

	return 0;
}

static void pfk_inode_free_security(struct inode *inode)
{
	if (inode == NULL)
		return;

	kzfree(inode->i_security);
}

static struct security_operations pfk_security_ops = {
	.name			= "pfk",

	.inode_alloc_security	= pfk_inode_alloc_security,
	.inode_free_security	= pfk_inode_free_security,

	.allow_merge_bio	= pfk_allow_merge_bio,
};

static int __init pfk_lsm_init(void)
{
	int ret;

	/* Check if PFK is the chosen lsm via security_module_enable() */
	if (security_module_enable(&pfk_security_ops)) {
		/* replace null callbacks with empty callbacks */
		security_fixup_ops(&pfk_security_ops);
		ret = register_security(&pfk_security_ops);
		if (ret != 0) {
			pr_err("pfk lsm registeration failed, ret=%d.\n", ret);
			return ret;
		}
		pr_debug("pfk is the chosen lsm, registered successfully !\n");
	} else {
		pr_debug("pfk is not the chosen lsm.\n");
		if (!selinux_is_enabled()) {
			pr_err("se linux is not enabled.\n");
			return -ENODEV;
		}

	}

	return 0;
}

/**
 * pfk_is_ready() - driver is initialized and ready.
 *
 * Return: true if the driver is ready.
 */
inline bool pfk_is_ready(void)
{
	return pfk_ready;
}
EXPORT_SYMBOL(pfk_is_ready);

static int get_key_from_bio(const struct bio *bio, bool *is_pfe,
			    const unsigned char **key_ret, size_t *key_size_ret,
			    const unsigned char **salt_ret, size_t *salt_size_ret,
			    enum ice_cryto_algo_mode *algo_mode_ret,
			    enum ice_crpto_key_size *key_size_type_ret)
{
	const struct bio_crypt_ctx *ctx = &bio->bi_crypt_ctx;

	if (!(ctx->bc_flags & BC_ENCRYPT_FL)) {
		*is_pfe = false;
		return -ENOTSUPP;
	}

	if (!(ctx->bc_flags & BC_AES_256_XTS_FL)) {
		pr_err("pfk: unsupported encryption algorithm (bc_flags=0x%x)\n",
		       ctx->bc_flags);
		return -ENOTSUPP;
	}

	if (ctx->bc_key_size !=
	    PFK_SUPPORTED_KEY_SIZE + PFK_SUPPORTED_SALT_SIZE) {
		pr_err("pfk: unsupported key size: %u\n", ctx->bc_key_size);
		return -ENOTSUPP;
	}

	*key_ret = &ctx->bc_key[0];
	*key_size_ret = PFK_SUPPORTED_KEY_SIZE;
	*salt_ret = &ctx->bc_key[PFK_SUPPORTED_KEY_SIZE];
	*salt_size_ret = PFK_SUPPORTED_SALT_SIZE;

	*algo_mode_ret = ICE_CRYPTO_ALGO_MODE_AES_XTS;

	BUILD_BUG_ON(PFK_SUPPORTED_KEY_SIZE != 256 / 8);
	*key_size_type_ret = ICE_CRYPTO_KEY_SIZE_256;
	return 0;
}

/**
 * pfk_load_key_start() - loads PFE encryption key to the ICE
 *						  Can also be invoked from non
 *						  PFE context, than it is not
 *						  relevant and is_pfe flag is
 *						  set to true
 * @bio: Pointer to the BIO structure
 * @ice_setting: Pointer to ice setting structure that will be filled with
 * ice configuration values, including the index to which the key was loaded
 * @is_pfe: Pointer to is_pfe flag, which will be true if function was invoked
 *			from PFE context
 *
 * Returns the index where the key is stored in encryption hw and additional
 * information that will be used later for configuration of the encryption hw.
 *
 * Must be followed by pfk_load_key_end when key is no longer used by ice
 *
 */
int pfk_load_key_start(const struct bio *bio,
		struct ice_crypto_setting *ice_setting, bool *is_pfe,
		bool async)
{
	int ret = 0;
	const unsigned char *key = NULL;
	const unsigned char *salt = NULL;
	size_t key_size = 0;
	size_t salt_size = 0;
	enum ice_cryto_algo_mode algo_mode = 0;
	enum ice_crpto_key_size key_size_type = 0;
	u32 key_index = 0;

	if (!is_pfe) {
		pr_err("is_pfe is NULL\n");
		return -EINVAL;
	}

	/* only a few errors below can indicate that
	 * this function was not invoked within PFE context,
	 * otherwise we will consider it PFE
	 */
	*is_pfe = true;

	if (!pfk_is_ready())
		return -ENODEV;

	if (!ice_setting) {
		pr_err("ice setting is NULL\n");
		return -EINVAL;
	}

	ret = get_key_from_bio(bio, is_pfe, &key, &key_size, &salt, &salt_size,
			       &algo_mode, &key_size_type);
	if (ret)
		return ret;

#if 0  /* debug */
	printk(KERN_WARNING "%s: Loading key w/ key[0:1] == [%.2x%.2x] and "
	       "salt[0:1] == [%.2x%.2x] into key_index [%d]\n", __func__,
	       key[0], key[1], salt[0], salt[1], key_index);
#endif
	ret = pfk_kc_load_key_start(key, key_size, salt, salt_size, &key_index,
			async);
	if (ret) {
		if (ret != -EBUSY && ret != -EAGAIN) {
			pr_err("start: could not load key into pfk key cache, "
			       "error %d\n", ret);
		}
		return ret;
	}

	ice_setting->key_size = key_size_type;
	ice_setting->algo_mode = algo_mode;
	/* hardcoded for now */
	ice_setting->key_mode = ICE_CRYPTO_USE_LUT_SW_KEY;
	ice_setting->key_index = key_index;

	return 0;
}

/**
 * pfk_load_key_end() - marks the PFE key as no longer used by ICE
 *						Can also be invoked from non
 *						PFE context, than it is not
 *						relevant and is_pfe flag is
 *						set to true
 * @bio: Pointer to the BIO structure
 * @is_pfe: Pointer to is_pfe flag, which will be true if function was invoked
 *			from PFE context
 */
int pfk_load_key_end(const struct bio *bio, bool *is_pfe)
{
	int ret = 0;
	const unsigned char *key = NULL;
	const unsigned char *salt = NULL;
	size_t key_size = 0;
	size_t salt_size = 0;
	enum ice_cryto_algo_mode algo_mode = 0;
	enum ice_crpto_key_size key_size_type = 0;

	if (!is_pfe) {
		pr_err("is_pfe is NULL\n");
		return -EINVAL;
	}

	/* only a few errors below can indicate that
	 * this function was not invoked within PFE context,
	 * otherwise we will consider it PFE
	 */
	*is_pfe = true;

	if (!pfk_is_ready())
		return -ENODEV;

	ret = get_key_from_bio(bio, is_pfe, &key, &key_size, &salt, &salt_size,
			       &algo_mode, &key_size_type);
	if (ret)
		return ret;

	pfk_kc_load_key_end(key, key_size, salt, salt_size);

	return 0;
}

/**
 * pfk_remove_key() - removes key from hw
 * @key: pointer to the key
 * @key_size: key size
 *
 * Will be used by external clients to remove a particular key for security
 * reasons.
 * The only API that can be used by dynamically loaded modules,
 * see explanations above at the beginning of this file.
 * The key is removed securely (by memsetting the previous value)
 */
int pfk_remove_key(const unsigned char *key, size_t key_size)
{
	int ret = 0;

	if (!pfk_is_ready())
		return -ENODEV;

	if (!key)
		return -EINVAL;

	ret = pfk_kc_remove_key(key, key_size);

	return ret;
}
EXPORT_SYMBOL(pfk_remove_key);

/**
 * pfk_allow_merge_bio() - Check if 2 BIOs can be merged.
 * @bio1:	Pointer to first BIO structure.
 * @bio2:	Pointer to second BIO structure.
 *
 * Prevent merging of BIOs from encrypted and non-encrypted
 * files, or files encrypted with different key.
 * This API is called by the file system block layer.
 *
 * Return: true if the BIOs allowed to be merged, false
 * otherwise.
 */
bool pfk_allow_merge_bio(struct bio *bio1, struct bio *bio2)
{
	/* if there is no pfk, don't disallow merging blocks */
	if (!pfk_is_ready())
		return true;

	/* cannot merge uninitialized BIO */
	if (!bio1 || !bio2)
		return false;

	/* Allow other components to make the decision. */
	return true;
}

static void __exit pfk_exit(void)
{
	pfk_ready = false;
	pfk_kc_deinit();
}

static int __init pfk_init(void)
{
	int ret = 0;

	ret = pfk_kc_init();
	if (ret != 0) {
		pr_err("could init pfk key cache, error %d\n", ret);
		goto fail;
	}

	ret = pfk_lsm_init();
	if (ret != 0) {
		pr_debug("neither pfk nor se-linux sec modules are enabled\n");
		pr_debug("not an error, just don't enable pfk\n");
		pfk_kc_deinit();
		return 0;
	}

	pfk_ready = true;
	pr_info("Driver initialized successfully\n");

	return 0;

fail:
	pr_err("Failed to init driver\n");
	return -ENODEV;
}

module_init(pfk_init);
module_exit(pfk_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Per-File-Key driver");
