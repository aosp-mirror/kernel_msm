/*
 * idme.c
 *
 * Copyright 2013 Amazon Technologies, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License Version 2. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define PRODUCT_FEATURES_DIR "product_features"
#define PRODUCT_FEATURE_NAME_GPS "gps"
#define PRODUCT_FEATURE_NAME_WAN "wan"

#define PRODUCT_FEATURE_STRING_GPS " "
#define PRODUCT_FEATURE_STRING_WAN " "


static int idme_proc_show(struct seq_file *seq, void *v)
{
	struct property *pp = (struct property *)seq->private;

	BUG_ON(!pp);

	seq_write(seq, pp->value, pp->length);

	return 0;
}

static int idme_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, idme_proc_show, PDE_DATA(inode));
}

static const struct file_operations idme_fops = {
	.owner		= THIS_MODULE,
	.open		= idme_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#if 0
static int proc_pfeature_wan_name_func(char *buffer, char **start, off_t off,
                                int count, int *eof, void *data)
{
        char str[] = PRODUCT_FEATURE_STRING_WAN;
        if (strlen(str) < PAGE_SIZE){
                memcpy(buffer, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}
#endif
bool board_has_wan(void)
{
       struct device_node *ap;
       int len;

       ap = of_find_node_by_path("/idme/board_id");
       if (ap) {
               const char *boardid = of_get_property(ap, "value", &len);
               if (len >= 2) {
                       if (boardid[0] == '0' && boardid[5] == '1')
                               return true;
               }
       }

       return false;
}

static int __init idme_init(void)
{
	struct proc_dir_entry *proc_idme = NULL;
	struct device_node *root = NULL, *child = NULL;
	struct property *pp_value = NULL;
	int perm = 0;
	/*static struct proc_dir_entry *proc_product_features_dir;*/

	root = of_find_node_by_path("/idme");
	if (!root)
		return -EINVAL;

	/* Create the root IDME procfs node */
	proc_idme = proc_mkdir("idme", NULL);
	if (!proc_idme) {
		of_node_put(root);
		return -ENOMEM;
	}

	/* Populate each IDME field */
	for (child = NULL; (child = of_get_next_child(root, child)); ) {
		pp_value = of_find_property(child, "value", NULL);

		if (!pp_value)
			continue;

		if (of_property_read_u32(child, "permission", &perm))
			continue;

		/* These values aren't writable anyways */
		perm &= ~(S_IWUGO);

		proc_create_data(child->name, perm, proc_idme,
			&idme_fops, pp_value);
	}

	of_node_put(child);
	of_node_put(root);

        /*proc_product_features_dir = proc_mkdir("product_features", NULL);
        if (proc_product_features_dir) {
                if (board_has_wan()) {
                        struct proc_dir_entry *proc_wan_name =
                                create_proc_entry(PRODUCT_FEATURE_NAME_WAN,
                                                S_IRUGO,
                                                proc_product_features_dir);
                        if (proc_wan_name != NULL) {
                                proc_wan_name->data = NULL;
                                proc_wan_name->read_proc =
                                        proc_pfeature_wan_name_func;
                                proc_wan_name->write_proc = NULL;
                        }
                }
        }*/

	return 0;
}
fs_initcall(idme_init);
