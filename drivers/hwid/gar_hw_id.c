#include <linux/module.h>
#include <linux/gar_hw_id.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include <soc/qcom/smsm.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/seq_file.h>

static gar_hwid_data_type *gar_hwid_data;

static const char gar_proj_id_str[][GAR_HWID_TYPE_STRLEN] = {
	"PFW1",
	"PFW2",
	"PFW3",
	"PFW4",
	"PFW5",
	"PFW6",
	"Undefined"
};

static const char gar_hw_id_str[][GAR_HWID_TYPE_STRLEN] = {
	"HVT3",
	"EVT",
	"DVT",
	"PVT",
	"SA",
	"DVT1_2",
	"Invalid"
};

static void gar_hwid_read_smem(void)
{
	unsigned hwid_smem_size;

	gar_hwid_data = (gar_hwid_data_type *)(smem_get_entry(SMEM_ID_VENDOR0,
		&hwid_smem_size, 0, SMEM_ANY_HOST_FLAG));

	printk(KERN_ERR "gar_proj_id = %ld, gar_hw_id = %ld\n",
		gar_hwid_data->gar_proj_id, gar_hwid_data->gar_hw_id);

	return;
}

long get_gar_proj_id(void)
{
	return gar_hwid_data->gar_proj_id;
}
EXPORT_SYMBOL(get_gar_proj_id);

long get_gar_hw_id(void)
{
	return gar_hwid_data->gar_hw_id;
}
EXPORT_SYMBOL(get_gar_hw_id);

int get_gar_qfuse(void)
{
        return gar_hwid_data->gar_qfuse;
}
EXPORT_SYMBOL(get_gar_qfuse);

static int gar_proj_id_read(struct seq_file *m, void *v)
{
	int proj_id;

	proj_id = get_gar_proj_id();
	if ((proj_id >= PROJ_ID_SIZE) || (proj_id < 0)) {
		seq_printf(m, "%s\n", gar_proj_id_str[PROJ_ID_SIZE - 1]);
	} else {
		seq_printf(m, "%s\n", gar_proj_id_str[proj_id]);
	}

	return 0;
}

static int gar_hw_id_read(struct seq_file *m, void *v)
{
	int hw_id;

	hw_id = get_gar_hw_id();
	if ((hw_id >= HW_ID_SIZE) || (hw_id < 0)) {
		seq_printf(m, "%s\n", gar_hw_id_str[HW_ID_SIZE - 1]);
	} else {
		seq_printf(m, "%s\n", gar_hw_id_str[hw_id]);
	}

	return 0;
}

static int gar_qfuse_read(struct seq_file *m, void *v)
{
	int qfuse;

	qfuse = get_gar_qfuse();
	if ((qfuse != true) && (qfuse != false)) {
		seq_printf(m, "unknown\n");
	} else {
		seq_printf(m, "%s\n", qfuse ? "true" : "false");
	}

	return 0;
}

static int gar_proj_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gar_proj_id_read, NULL);
}

static int gar_hw_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gar_hw_id_read, NULL);
}

static int gar_qfuse_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, gar_qfuse_read, NULL);
}

static const struct file_operations gar_proj_id_fops = {
	.open       = gar_proj_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations gar_hw_id_fops = {
	.open       = gar_hw_id_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations gar_qfuse_fops = {
	.open       = gar_qfuse_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

void gar_hwid_info_read(void)
{
	gar_hwid_read_smem();

	proc_create("gar_proj_id_type", 0, NULL, &gar_proj_id_fops);
	proc_create("gar_hw_id_type", 0, NULL, &gar_hw_id_fops);
	proc_create("gar_qfuse_type", 0, NULL, &gar_qfuse_fops);
}
EXPORT_SYMBOL(gar_hwid_info_read);

static int __init gar_hw_id_init(void)
{
	int err = 0;

	gar_hwid_info_read();
	return err;
}

static void __exit gar_hw_id_exit(void)
{
	printk(KERN_INFO "gar_hwid_exit enter\n");
}

arch_initcall(gar_hw_id_init);
module_exit(gar_hw_id_exit);

MODULE_DESCRIPTION("gar hardware ID driver");
MODULE_LICENSE("GPL");
