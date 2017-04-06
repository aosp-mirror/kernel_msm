/*
 * MNH Firmware for firmware authentication
 *
 * Copyright 2017 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/types.h>
#include <crypto/pkcs7.h>
#include <crypto/public_key.h>
#include <keys/asymmetric-type.h>
#include <keys/system_keyring.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/key.h>
#include <linux/slab.h>
#include "mnh-crypto.h"
#include "mnh-sm.h"

#define PKEY_ID_PKCS7 2
#define ESL_SIG_STRING "~Module signature appended~\n"

/**
 * Verify signature of a firmware blob in the filesystem
 * @dev: Device node
 * @path: path of the firmware file to be verified
 * RETURNS:
 * 0             success
 * -EBADMSG      PKCS#7 signature with non-detached data
 * -ECANCELED    invalid PKCS#7 message or signature block is missing
 * -EKEYREJECTED key had a usage restriction, or signature failed to match
 * -ENOPKG       crypto module could not be found
 * -ENOKEY       PKCS#7 signature not signed with a trusted key
 */
int mnh_crypto_verify_fw(struct device *dev, const char *path)
{
	struct cert_info *info;
	const struct firmware *img;
	int err = 0;

	/* load firmware from filesystem */
	dev_dbg(dev, "%s loading firmware: %s\n", __func__, path);
	err = request_firmware(&img, path, dev);
	if (err) {
		dev_err(dev, "%s request firmware failed - %d\n",
			__func__, err);
		return err;
	}
	dev_dbg(dev, "%s firmware loaded\n", __func__);

	info = kzalloc(sizeof(struct cert_info),
			     GFP_KERNEL);
	if (info == NULL) {
		release_firmware(img);
		return -ENOMEM;
	}

	info->img = img->data;
	info->img_len = img->size;
	err = mnh_crypto_verify(dev, info);

	kfree(info);
	release_firmware(img);

	return err;
}
EXPORT_SYMBOL_GPL(mnh_crypto_verify_fw);

/**
 * Check proper framing of image, signature, header and end-marker
 * and perform signature check.
 * @dev: Device node
 * @info: data structure including buffer address and size
 * RETURNS:
 * 0             success
 * -EBADMSG      PKCS#7 signature with non-detached data
 * -ECANCELED    invalid PKCS#7 message or signature block is missing
 * -EKEYREJECTED key had a usage restriction, or signature failed to match
 * -ENOPKG       crypto module could not be found
 * -ENOKEY       PKCS#7 signature not signed with a trusted key
 */
int mnh_crypto_verify(struct device *dev, struct cert_info *info)
{
	int err = -ENOKEY;
	const unsigned long marker_len = sizeof(ESL_SIG_STRING) - 1;
	const void *img = info->img;

	dev_dbg(dev, "%s starting signature check\n", __func__);
	/* check if signature string is present at the end of the image */
	dev_dbg(dev, "%s - buffer addr: %pP | buffer size: %lu\n", __func__,
		info->img, info->img_len);
	if (memcmp(img + info->img_len - marker_len,
		   ESL_SIG_STRING, marker_len) == 0) {
		dev_dbg(dev, "%s ESL_SIG_STRING found, proceeding\n",
			__func__);
		info->img_len -= marker_len;
		/* verify the signature against the available keys */
		err = mnh_crypto_verify_sig(dev, info);
		info->cert_size += marker_len;
		if (!err) {
			info->cert = FW_IMAGE_CERT_OK;
			dev_dbg(dev, "%s Image signature verified OK\n",
				__func__);
		} else {
			dev_err(dev, "%s Image signature error: %d\n",
				__func__, err);
			info->cert = FW_IMAGE_TAINTED;
		}
	} else {
		info->cert = FW_IMAGE_NO_CERT;
		dev_err(dev, "%s ESL_SIG_STRING not found, aborting\n",
			__func__);
		err = -ECANCELED;
	}
	return err;
}
EXPORT_SYMBOL_GPL(mnh_crypto_verify);

/*
 * mnh_crypto_verify_sig checks the validity of the image's signature type
 * and validates the signature of the image against the available public
 * keys in the system keychain. Function populates info->cert & ->cert_size.
 * RETURNS:
 * 0             success
 * -EBADMSG      PKCS#7 signature with non-detached data
 * -ECANCELED    invalid PKCS#7 message
 * -EKEYREJECTED key had a usage restriction, or signature failed to match
 * -ENOPKG       crypto module could not be found
 * -ENOKEY       PKCS#7 signature not signed with a trusted key
 */
int mnh_crypto_verify_sig(struct device *dev, struct cert_info *info)
{
	struct esl_img_signature esl_sig;
	size_t sig_len;
	size_t imglen = info->img_len;
	const void *esl_img = info->img;

	dev_dbg(dev, "mnh %s image size %zu\n", __func__, imglen);
	if (imglen <= sizeof(esl_sig))
		return -ECANCELED;
	/* this structure is appended at the end of the file */
	memcpy(&esl_sig, esl_img + (imglen - sizeof(esl_sig)),
	       sizeof(esl_sig));
	/* strip out the esl_img_signature structure at the end */
	imglen -= sizeof(esl_sig);
	/* length of the actual signature block */
	sig_len = be32_to_cpu(esl_sig.sig_len);
	/* check signature type */
	if (esl_sig.id_type != PKEY_ID_PKCS7) {
		dev_err(dev, "mnh %s Image not signed with PKCS#7 message\n",
			__func__);
		return -ECANCELED;
	}
	/* strip out the signature, all that is left is the payload */
	if (imglen <= sig_len)
		return -ECANCELED;
	imglen -= sig_len;
	dev_dbg(dev, "mnh %s sizes: payload %zu | signature %zu | ident %zu\n",
		__func__, imglen, sig_len, sizeof(esl_sig));

	/* total size of signature and header information */
	info->cert_size = info->img_len - imglen;

	return verify_pkcs7_signature(esl_img, imglen,
				      esl_img + imglen, sig_len,
				      NULL, VERIFYING_MODULE_SIGNATURE,
				      NULL, NULL);
}
