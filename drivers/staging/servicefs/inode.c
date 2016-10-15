/*
 *  inode.c - part of servicefs, a small filesystem for service namespaces.
 *
 *  Copyright (C) 2015 Corey Tabaka <eieio@google.com>
 *  Copyright (C) 2015 Google, Inc.
 *
 *  Based on debugfs:
 *  Copyright (C) 2004 Greg Kroah-Hartman <greg@kroah.com>
 *  Copyright (C) 2004 IBM Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/namei.h>
#include <linux/fsnotify.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/parser.h>
#include <linux/magic.h>
#include <linux/slab.h>

#include "servicefs_private.h"

#define SERVICEFS_DEFAULT_MODE	0700

/* declared in file.c */
extern const struct file_operations initial_file_operations;

static int servicefs_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode,
			 void *data, const struct file_operations *fops);

static int servicefs_create(struct inode *dir, struct dentry *dentry, umode_t mode,
			  void *data, const struct file_operations *fops);

static int inode_ops_create(struct inode *dir, struct dentry *dentry, umode_t mode,
		bool excl)
{
	struct service *svc;

	svc = service_new();
	if (!svc)
		return -ENOMEM;

	return servicefs_create(dir, dentry, mode, svc, &initial_file_operations);
}

static int inode_ops_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode)
{
	return servicefs_mkdir(dir, dentry, mode, NULL, NULL);
}

static const struct inode_operations servicefs_inode_operations = {
	.lookup         = simple_lookup,
	.create         = inode_ops_create,
	.mkdir          = inode_ops_mkdir,
	.rmdir          = simple_rmdir,
	.rename         = simple_rename,
	.unlink         = simple_unlink,
};

static struct inode *servicefs_new_inode(struct super_block *sb, umode_t mode, dev_t dev,
				       void *data, const struct file_operations *fops)

{
	struct inode *inode = new_inode(sb);

	if (inode) {
		inode->i_ino = get_next_ino();
		inode->i_mode = mode;
		inode->i_uid = current_fsuid();
		inode->i_gid = current_fsgid();
		inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
		switch (mode & S_IFMT) {
		default:
			init_special_inode(inode, mode, dev);
			break;
		case S_IFREG:
			inode->i_fop = fops;
			inode->i_private = data;
			break;
		case S_IFDIR:
			inode->i_op = &servicefs_inode_operations;
			inode->i_fop = fops ? fops : &simple_dir_operations;
			inode->i_private = data;

			/* directory inodes start off with i_nlink == 2
			 * (for "." entry) */
			inc_nlink(inode);
			break;
		}
	}
	return inode;
}

/* SMP-safe */
static int servicefs_mknod(struct inode *dir, struct dentry *dentry,
			 umode_t mode, dev_t dev, void *data,
			 const struct file_operations *fops)
{
	struct inode *inode;
	int error = -EPERM;

	if (dentry->d_inode)
		return -EEXIST;

	inode = servicefs_new_inode(dir->i_sb, mode, dev, data, fops);
	if (inode) {
		d_instantiate(dentry, inode);
		dget(dentry);
		error = 0;
	}
	return error;
}

static int servicefs_mkdir(struct inode *dir, struct dentry *dentry, umode_t mode,
			 void *data, const struct file_operations *fops)
{
	int res;

	mode = (mode & (S_IRWXUGO | S_ISVTX)) | S_IFDIR;
	res = servicefs_mknod(dir, dentry, mode, 0, data, fops);
	if (!res) {
		inc_nlink(dir);
		fsnotify_mkdir(dir, dentry);
	}
	return res;
}

static int servicefs_create(struct inode *dir, struct dentry *dentry, umode_t mode,
			  void *data, const struct file_operations *fops)
{
	int res;

	mode = (mode & S_IALLUGO) | S_IFREG;
	res = servicefs_mknod(dir, dentry, mode, 0, data, fops);
	if (!res)
		fsnotify_create(dir, dentry);
	return res;
}

static inline int servicefs_positive(struct dentry *dentry)
{
	return dentry->d_inode && !d_unhashed(dentry);
}

static void servicefs_destroy_inode(struct inode *inode)
{
	if (inode->i_mode & S_IFREG) {
		struct service *svc = inode->i_private;
		BUG_ON(!svc);

		pr_debug("svc=%p\n", svc);
		service_free(svc);
	}

	free_inode_nonrcu(inode);
}

struct servicefs_mount_opts {
	kuid_t uid;
	kgid_t gid;
	umode_t mode;
};

enum {
	Opt_uid,
	Opt_gid,
	Opt_mode,
	Opt_err
};

static const match_table_t tokens = {
	{Opt_uid, "uid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_mode, "mode=%o"},
	{Opt_err, NULL}
};

struct servicefs_fs_info {
	struct servicefs_mount_opts mount_opts;
};

static int servicefs_parse_options(char *data, struct servicefs_mount_opts *opts)
{
	substring_t args[MAX_OPT_ARGS];
	int option;
	int token;
	kuid_t uid;
	kgid_t gid;
	char *p;

	opts->mode = SERVICEFS_DEFAULT_MODE;

	while ((p = strsep(&data, ",")) != NULL) {
		if (!*p)
			continue;

		token = match_token(p, tokens, args);
		switch (token) {
		case Opt_uid:
			if (match_int(&args[0], &option))
				return -EINVAL;
			uid = make_kuid(current_user_ns(), option);
			if (!uid_valid(uid))
				return -EINVAL;
			opts->uid = uid;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return -EINVAL;
			gid = make_kgid(current_user_ns(), option);
			if (!gid_valid(gid))
				return -EINVAL;
			opts->gid = gid;
			break;
		case Opt_mode:
			if (match_octal(&args[0], &option))
				return -EINVAL;
			opts->mode = option & S_IALLUGO;
			break;
		/*
		 * We might like to report bad mount options here;
		 * but traditionally servicefs has ignored all mount options
		 */
		}
	}

	return 0;
}

static int servicefs_apply_options(struct super_block *sb)
{
	struct servicefs_fs_info *fsi = sb->s_fs_info;
	struct inode *inode = sb->s_root->d_inode;
	struct servicefs_mount_opts *opts = &fsi->mount_opts;

	inode->i_mode &= ~S_IALLUGO;
	inode->i_mode |= opts->mode;

	inode->i_uid = opts->uid;
	inode->i_gid = opts->gid;

	return 0;
}

static int servicefs_remount(struct super_block *sb, int *flags, char *data)
{
	int err;
	struct servicefs_fs_info *fsi = sb->s_fs_info;

	sync_filesystem(sb);
	err = servicefs_parse_options(data, &fsi->mount_opts);
	if (err)
		goto fail;

	servicefs_apply_options(sb);

fail:
	return err;
}

static int servicefs_show_options(struct seq_file *m, struct dentry *root)
{
	struct servicefs_fs_info *fsi = root->d_sb->s_fs_info;
	struct servicefs_mount_opts *opts = &fsi->mount_opts;

	if (!uid_eq(opts->uid, GLOBAL_ROOT_UID))
		seq_printf(m, ",uid=%u",
			   from_kuid_munged(&init_user_ns, opts->uid));
	if (!gid_eq(opts->gid, GLOBAL_ROOT_GID))
		seq_printf(m, ",gid=%u",
			   from_kgid_munged(&init_user_ns, opts->gid));
	if (opts->mode != SERVICEFS_DEFAULT_MODE)
		seq_printf(m, ",mode=%o", opts->mode);

	return 0;
}

static const struct super_operations servicefs_super_operations = {
	.destroy_inode = servicefs_destroy_inode,
	.statfs        = simple_statfs,
	.remount_fs    = servicefs_remount,
	.show_options  = servicefs_show_options,
};

static int servicefs_fill_super(struct super_block *s, void *data, int silent)
{
	struct inode *inode;
	struct dentry *root;
	struct servicefs_fs_info *fsi;
	int err;

	save_mount_options(s, data);

	fsi = kzalloc(sizeof(struct servicefs_fs_info), GFP_KERNEL);
	s->s_fs_info = fsi;
	if (!fsi)
		return -ENOMEM;

	err = servicefs_parse_options(data, &fsi->mount_opts);
	if (err)
		goto error;

	s->s_blocksize = PAGE_CACHE_SIZE;
	s->s_blocksize_bits = PAGE_CACHE_SHIFT;
	s->s_magic = SERVICEFS_MAGIC;
	s->s_op = &servicefs_super_operations;
	s->s_time_gran = 1;

	inode = new_inode(s);
	if (!inode) {
		err = -ENOMEM;
		goto error;
	}

	inode->i_ino = 1;
	inode->i_mode = S_IFDIR | 0755;
	inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
	inode->i_op = &servicefs_inode_operations;
	inode->i_fop = &simple_dir_operations;

	/* directory inodes start of with i_nlink == 2 for "." entry */
	set_nlink(inode, 2);

	root = d_make_root(inode);
	s->s_root = root;
	if (!root) {
		err = -ENOMEM;
		goto error;
	}

	servicefs_apply_options(s);

	return 0;

error:
	kfree(fsi);
	s->s_fs_info = NULL;
	return err;
}

static struct dentry *servicefs_mount(struct file_system_type *fs_type,
			int flags, const char *dev_name, void *data)
{
	return mount_single(fs_type, flags, data, servicefs_fill_super);
}

static struct file_system_type service_fs_type = {
	.owner =	THIS_MODULE,
	.name =		"servicefs",
	.mount =	servicefs_mount,
	.kill_sb =	kill_litter_super,
};

static int __servicefs_remove(struct dentry *dentry, struct dentry *parent)
{
	int ret = 0;

	if (servicefs_positive(dentry)) {
		if (dentry->d_inode) {
			dget(dentry);
			switch (dentry->d_inode->i_mode & S_IFMT) {
			case S_IFDIR:
				ret = simple_rmdir(parent->d_inode, dentry);
				break;
			case S_IFLNK:
				kfree(dentry->d_inode->i_private);
				/* fall through */
			default:
				simple_unlink(parent->d_inode, dentry);
				break;
			}
			if (!ret)
				d_delete(dentry);
			dput(dentry);
		}
	}
	return ret;
}

/**
 * servicefs_remove_dentry - removes a dentry from the servicefs filesystem
 * @dentry: a pointer to a the dentry to be removed.
 */
void servicefs_remove_dentry(struct dentry *dentry)
{
	struct dentry *parent;
	int ret;

	if (!dentry)
		return;

	parent = dentry->d_parent;
	if (!parent || !parent->d_inode)
		return;

	mutex_lock(&parent->d_inode->i_mutex);
	ret = __servicefs_remove(dentry, parent);
	mutex_unlock(&parent->d_inode->i_mutex);
}

static struct kobject *servicefs_kobj;

static int __init servicefs_init(void)
{
	int retval;

	retval = servicefs_cache_init();
	if (retval)
		return retval;

	servicefs_kobj = kobject_create_and_add("servicefs", kernel_kobj);
	if (!servicefs_kobj)
		return -EINVAL;

	retval = register_filesystem(&service_fs_type);
	if (retval)
		kobject_put(servicefs_kobj);

	return retval;
}
core_initcall(servicefs_init);

