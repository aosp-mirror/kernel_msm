#include <asm/cacheflush.h>
#include <asm/compiler.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <misc/faceauth_hypx.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>

#define HYPX_SMC_ID(func) (0x43DEAD00 | func)
#define HYPX_SMC_FUNC_CHECK_PIL_COMPLETION HYPX_SMC_ID(0x1)
#define HYPX_SMC_FUNC_INIT HYPX_SMC_ID(0x2)
#define HYPX_SMC_FUNC_PROCESS HYPX_SMC_ID(0x3)
#define HYPX_SMC_FUNC_CHECK_PROCESS_RESULT HYPX_SMC_ID(0x4)
#define HYPX_SMC_FUNC_CLEANUP HYPX_SMC_ID(0x5)
#define HYPX_SMC_FUNC_GET_DEBUG_RESULT HYPX_SMC_ID(0x6)

#define MIN(x, y) ((x) < (y) ? (x) : (y))

#define PIL_DMA_TIMEOUT 3000

struct hypx_mem_segment {
	/* address of the segment begin */
	uint32_t addr;
	/* number of pages in the segment */
	uint32_t pages;
} __packed;

#define HYPX_MEMSEGS_NUM (PAGE_SIZE / sizeof(struct hypx_mem_segment))

struct hypx_blob {
	struct hypx_mem_segment segments[HYPX_MEMSEGS_NUM];
} __packed;

/* Keep this struct in sync with HypX firmware code at
 * https://team.git.corp.google.com/soc-gerrit-admin/faceauth-hypx/
 * Put fields in order to avoid unaligned access that EL2 does not like
 */
struct hypx_fa_init {
	uint64_t verbosity_level;
	/* phy address for 4KiB temporary buffer used by EL2 DMA engine */
	uint64_t bounce_buff;
	uint64_t features;
} __packed;

struct hypx_fa_process {
	uint64_t image_dot_left; /* PHY addr */
	uint64_t image_dot_right; /* PHY addr */
	uint64_t image_flood; /* PHY addr */
	uint64_t calibration; /* PHY addr */

	uint32_t operation;
	uint32_t profile_id;

	uint32_t image_dot_left_size;
	uint32_t image_dot_right_size;
	uint32_t image_flood_size;
	uint32_t calibration_size;
} __packed;

struct hypx_fa_process_results {
	uint32_t result;
	uint32_t bin_result;
	uint32_t fw_version;
	int32_t error_code;
	uint64_t debug_buffer; /* PHY addr*/
	uint32_t debug_buffer_size;
} __packed;

static void hypx_free_blob(phys_addr_t blob_phy)
{
	int source_vm[] = { VMID_EXT_DSP, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE };
	struct hypx_blob *blob = phys_to_virt(blob_phy);
	int i;

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		uint64_t phy_addr;
		void *virt_addr;
		int ret;

		if (!blob->segments[i].addr)
			break;
		phy_addr = (uint64_t)blob->segments[i].addr * PAGE_SIZE;
		virt_addr = phys_to_virt(phy_addr);

		ret = hyp_assign_phys(phy_addr,
				      blob->segments[i].pages * PAGE_SIZE,
				      source_vm, ARRAY_SIZE(source_vm), dest_vm,
				      dest_perm, ARRAY_SIZE(dest_vm));
		if (ret)
			pr_err("hyp_assign_phys returned an error %d\n", ret);

		kfree(virt_addr);
	}

	free_page((unsigned long)blob);
}

static int hypx_copy_from_blob(void __user *buffer, phys_addr_t blob_phy,
				size_t size)
{
	int source_vm[] = { VMID_EXT_DSP, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE };
	struct hypx_blob *blob = phys_to_virt(blob_phy);
	void __user *buffer_iter = buffer;
	uint64_t buffer_iter_remaining = size;
	uint64_t tocopy;
	int i;
	int lret, ret = 0;

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		uint64_t phy_addr;
		void *virt_addr;

		if (!blob->segments[i].addr)
			break;
		tocopy = MIN(buffer_iter_remaining,
			     blob->segments[i].pages * PAGE_SIZE);
		phy_addr = (uint64_t)blob->segments[i].addr * PAGE_SIZE;
		virt_addr = phys_to_virt(phy_addr);

		lret = hyp_assign_phys(phy_addr,
				      blob->segments[i].pages * PAGE_SIZE,
				      source_vm, ARRAY_SIZE(source_vm), dest_vm,
				      dest_perm, ARRAY_SIZE(dest_vm));
		if (lret) {
			ret = lret;
			pr_err("hyp_assign_phys returned an error %d\n", ret);
		}

		if (copy_to_user(buffer_iter, virt_addr, tocopy)) {
			return -EFAULT;
		}
		buffer_iter += tocopy;
		buffer_iter_remaining -= tocopy;
	}
	return ret;
}

/* Returns PHY address for the allocated buffer */
static phys_addr_t hypx_create_blob(void __user *buffer, size_t size)
{
	uint64_t buffer_iter_remaining;
	const void __user *buffer_iter;
	struct hypx_blob *blob;
	struct hypx_mem_segment *segments_iter;
	int i;
	uint64_t page_order;

	/* note that allocated page is not reclaimable */
	blob = (struct hypx_blob *)get_zeroed_page(0);
	if (!blob) {
		pr_err("Cannot allocate memory for hypx blob\n");
		goto exit;
	}
	WARN_ON(virt_to_phys(blob) % PAGE_SIZE);
	page_order = MIN(MAX_ORDER - 1, roundup_pow_of_two(size));
	buffer_iter = buffer;
	buffer_iter_remaining = size;
	segments_iter = blob->segments;

	for (i = 0; i < HYPX_MEMSEGS_NUM; i++) {
		uint64_t size = (1ULL << page_order) * PAGE_SIZE;
		void *out_buffer = kmalloc(size, 0);
		uint64_t tocopy = MIN(buffer_iter_remaining, size);

		int ret;
		int source_vm[] = { VMID_HLOS };
		int dest_vm[] = { VMID_EXT_DSP, VMID_HLOS_FREE };
		int dest_perm[] = { PERM_READ | PERM_WRITE,
				    PERM_READ | PERM_WRITE };

		if (tocopy == 0) {
			kfree(out_buffer);
			break;
		}

		while (!out_buffer) {
			if (page_order == 0) {
				pr_err("Cannot allocate memory for copying data for hypx");
				goto exit;
			}
			page_order--;
			size = (1ULL << page_order) * PAGE_SIZE;
			out_buffer = kmalloc(size, 0);
			tocopy = MIN(buffer_iter_remaining, size);
		}

		copy_from_user(out_buffer, buffer_iter, tocopy);

		/*
		 * In the future this hyp_assign call will be invoked for
		 * Camera buffers by camera stack itself.
		 * We will need to assign only calibration/debug buffers.
		 */
		ret = hyp_assign_phys(virt_to_phys(out_buffer), size, source_vm,
				      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
				      ARRAY_SIZE(dest_vm));
		if (ret)
			pr_err("hyp_assign_phys returned an error %d\n", ret);

		segments_iter->addr = virt_to_phys(out_buffer) / PAGE_SIZE;
		segments_iter->pages = 1ULL << page_order;
		segments_iter++;

		buffer_iter_remaining -= tocopy;
		buffer_iter += tocopy;
	}

	if (buffer_iter_remaining) {
		pr_err("Memory allocator is fragmented so we were not able to "
			     "fit %d into segments header\n",
		       size);
		goto exit;
	}

	return virt_to_phys(blob);

exit:
	hypx_free_blob(virt_to_phys(blob));

	return 0;
}

int el2_faceauth_wait_pil_dma_over(void)
{
	struct scm_desc check_dma_desc = { 0 };
	int ret;
	unsigned long stop;

	stop = jiffies + msecs_to_jiffies(PIL_DMA_TIMEOUT);

	do {
		check_dma_desc.arginfo = SCM_ARGS(0);
		ret = scm_call2(HYPX_SMC_FUNC_CHECK_PIL_COMPLETION,
				&check_dma_desc);
		if (ret)
			return ret;

		ret = check_dma_desc.ret[0];
		if (ret == 1) {
			/* DMA completed successfully */
			return 0;
		} else if (ret == 2) {
			/* DMA is still running */
			msleep(1);
		} else {
			/* EL2 call failed */
			return -EFAULT;
		}
		if (time_before(stop, jiffies)) {
			pr_err("PIL DMA timeout!\n");
			return -ETIME;
		}

	} while (true);
}

static void *bounce_buff;
static dma_addr_t bounce_buff_bus_addr;

int el2_faceauth_init(struct device *dev, struct faceauth_init_data *data,
		      uint64_t verbosity_level)
{
	int ret;
	struct scm_desc desc = { 0 };
	struct hypx_fa_init *hypx_data;
	int source_vm[] = { VMID_HLOS };
	int dest_vm[] = { VMID_EXT_DSP, VMID_HLOS_FREE };
	int dest_perm[] = { PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE };

	hypx_data = (void *)get_zeroed_page(0);
	hypx_data->verbosity_level = verbosity_level;
	hypx_data->features = data->features;

	bounce_buff = dma_alloc_coherent(dev, PAGE_SIZE, &bounce_buff_bus_addr,
					 GFP_KERNEL);
	hypx_data->bounce_buff = bounce_buff_bus_addr;

	ret = hyp_assign_phys(bounce_buff_bus_addr, PAGE_SIZE, source_vm,
			      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
			      ARRAY_SIZE(dest_vm));
	if (ret) {
		pr_err("hyp_assign_phys returned an error %d\n", ret);
		goto exit;
	}

	desc.args[0] = virt_to_phys(hypx_data);
	desc.arginfo = SCM_ARGS(1);

	flush_cache_all();
	ret = scm_call2(HYPX_SMC_FUNC_INIT, &desc);
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit;
	}

exit:
	free_page((unsigned long)hypx_data);
	return ret;
}

int el2_faceauth_cleanup(struct device *dev)
{
	int ret;
	struct scm_desc desc = { 0 };
	int source_vm[] = { VMID_EXT_DSP, VMID_HLOS_FREE };
	int dest_vm[] = { VMID_HLOS };
	int dest_perm[] = { PERM_READ | PERM_WRITE };

	desc.arginfo = SCM_ARGS(0);
	ret = scm_call2(HYPX_SMC_FUNC_CLEANUP, &desc);
	if (ret)
		pr_err("Failed scm_call %d\n", ret);

	ret = hyp_assign_phys(bounce_buff_bus_addr, PAGE_SIZE, source_vm,
			      ARRAY_SIZE(source_vm), dest_vm, dest_perm,
			      ARRAY_SIZE(dest_vm));
	if (ret)
		pr_err("hyp_assign_phys returned an error %d\n", ret);

	dma_free_coherent(dev, PAGE_SIZE, bounce_buff, bounce_buff_bus_addr);
	bounce_buff = NULL;
	bounce_buff_bus_addr = 0;

	return ret;
}

int el2_faceauth_process(struct faceauth_start_data *data)
{
	int ret;
	struct scm_desc desc = { 0 };
	bool pass_images_to_el2;
	struct hypx_fa_process *hypx_data;

	pass_images_to_el2 = data->operation == FACEAUTH_OP_ENROLL ||
			     data->operation == FACEAUTH_OP_VALIDATE;

	hypx_data = (void *)get_zeroed_page(0);

	hypx_data->operation = data->operation;
	hypx_data->profile_id = data->profile_id;
	if (pass_images_to_el2) {
		hypx_data->image_dot_left = hypx_create_blob(
			data->image_dot_left, data->image_dot_left_size);
		hypx_data->image_dot_left_size = data->image_dot_left_size;
		hypx_data->image_dot_right = hypx_create_blob(
			data->image_dot_right, data->image_dot_right_size);
		hypx_data->image_dot_right_size = data->image_dot_right_size;
		hypx_data->image_flood = hypx_create_blob(
			data->image_flood, data->image_flood_size);
		hypx_data->image_flood_size = data->image_flood_size;
		if (data->calibration) {
			hypx_data->calibration = hypx_create_blob(
				data->calibration, data->calibration_size);
			hypx_data->calibration_size = data->calibration_size;
		}
	}

	desc.args[0] = virt_to_phys(hypx_data);
	desc.arginfo = SCM_ARGS(1);

	flush_cache_all();
	ret = scm_call2(HYPX_SMC_FUNC_PROCESS, &desc);
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit;
	}

exit:
	if (pass_images_to_el2) {
		hypx_free_blob(hypx_data->image_dot_left);
		hypx_free_blob(hypx_data->image_dot_right);
		hypx_free_blob(hypx_data->image_flood);
	}

	if (data->calibration)
		hypx_free_blob(hypx_data->calibration);

	free_page((unsigned long)hypx_data);

	return ret;
}

int el2_faceauth_get_process_result(struct faceauth_start_data *data)
{
	int ret;
	struct scm_desc desc = { 0 };
	struct hypx_fa_process_results *hypx_data;

	hypx_data = (void *)get_zeroed_page(0);

	desc.arginfo = SCM_ARGS(1);
	desc.args[0] = virt_to_phys(hypx_data);

	flush_cache_all();
	ret = scm_call2(HYPX_SMC_FUNC_CHECK_PROCESS_RESULT, &desc);
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit;
	}
	data->result = hypx_data->result;
	data->bin_bitmap = hypx_data->bin_result;
	data->fw_version = hypx_data->fw_version;
	data->error_code = hypx_data->error_code;

exit:
	free_page((unsigned long)hypx_data);
	return ret;
}

int el2_faceauth_gather_debug_log(struct faceauth_debug_data *data)
{
	int ret;
	struct scm_desc desc = { 0 };
	struct hypx_fa_process_results *hypx_data;

	hypx_data = (struct hypx_fa_process_results *)get_zeroed_page(0);

	hypx_data->debug_buffer =
		hypx_create_blob(data->debug_buffer, data->debug_buffer_size);
	hypx_data->debug_buffer_size = data->debug_buffer_size;

	if (!hypx_data->debug_buffer) {
		pr_err("Fail to alloc mem for debug_buffer");
		goto exit2;
	}

	desc.arginfo = SCM_ARGS(1);
	desc.args[0] = virt_to_phys(hypx_data);

	flush_cache_all();
	ret = scm_call2(HYPX_SMC_FUNC_GET_DEBUG_RESULT, &desc);
	if (ret) {
		pr_err("Failed scm_call %d\n", ret);
		goto exit1;
	}
	ret = hypx_copy_from_blob(data->debug_buffer, hypx_data->debug_buffer,
			    data->debug_buffer_size);
	if (ret) {
		pr_err("Failed hypx_copy_from_blob %d\n", ret);
		goto exit1;
	}

exit1:
	hypx_free_blob(hypx_data->debug_buffer);
exit2:
	free_page((unsigned long)hypx_data);
	return ret;
}
