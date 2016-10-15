#include <linux/mm.h>
#include <linux/uio.h>
#include <linux/sched.h>
#include <linux/highmem.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include "servicefs_private.h"

#define MAX_KMALLOC_PAGES  (PAGE_SIZE * 2)
#define MAX_PP_ARRAY_COUNT 16
#define MAX_PAGES_PER_LOOP (MAX_KMALLOC_PAGES / sizeof(struct pages *))

struct vm_transfer {
	struct page *pp_stack[MAX_PP_ARRAY_COUNT];
	struct page **process_pages;
	struct mm_struct *mm;
	struct task_struct *task;

	unsigned long page_address;
	unsigned long page_offset;
	unsigned int page_count;

	int span_pinned; // may be different than page_count if get_user_pages fails
	int span_index;

	void *mapped_page;
	int mapped_index;

	bool vm_write;
	bool *remote_fault;

	ssize_t total_bytes_requested;
	ssize_t total_bytes_copied;

	struct iov_buffer *remote;
	struct iov_buffer *local;
};

/**
 * pages_per_iov - calculate the number of pages an iovec segment spans
 * @vec: iovec segment to calculate the page count for
 * @offset: offset into the segment to discount from the calculation
 * Returns the number of pages an iovec segment spans, accounting for
 * an offset into the segment. An offset of zero will yield the total
 * number of pages the segment spans, while greater than zero will
 * discount any pages that would be skipped by starting at that offset
 * in the segment. It is up to the caller to ensure that the offset is
 * within the segment.
 */
static unsigned long pages_per_iov(const struct iovec *vec, size_t offset)
{
	unsigned long base = (unsigned long) vec->iov_base + offset;
	unsigned long len = vec->iov_len - offset;

	return (base + len) / PAGE_SIZE - base / PAGE_SIZE + 1;
}

static inline void vm_transfer_debug_start(struct vm_transfer *xfer)
{
	pr_debug("------------------------------------------------\n");
	pr_debug("xfer=%p task=%p local_bytes=%zu remote_bytes=%zu\n", xfer, xfer->task,
			iov_buffer_bytes_remaining(xfer->local),
			iov_buffer_bytes_remaining(xfer->remote));
	pr_debug("vm_write=%d total_bytes_requested=%zd total_bytes_copied=%zd\n",
			xfer->vm_write, xfer->total_bytes_requested, xfer->total_bytes_copied);
}

static inline void vm_transfer_debug(struct vm_transfer *xfer)
{
	pr_debug("local_bytes_remaining=%zu remote_bytes_remaining=%zu"
			" total_bytes_copied=%zd\n", iov_buffer_bytes_remaining(xfer->local),
			iov_buffer_bytes_remaining(xfer->remote), xfer->total_bytes_copied);
}

static inline void vm_transfer_debug_stop(struct vm_transfer *xfer)
{
	pr_debug("total_bytes_requested=%zd total_bytes_copied=%zd\n",
			xfer->total_bytes_requested, xfer->total_bytes_copied);
	pr_debug("------------------------------------------------\n");
}

static inline ssize_t vm_transfer_bytes_remaining(struct vm_transfer *xfer)
{
	return xfer->total_bytes_requested - xfer->total_bytes_copied;
}

static inline int vm_transfer_pages_remaining(struct vm_transfer *xfer)
{
	return xfer->span_pinned - xfer->span_index;
}

static inline uint8_t *vm_transfer_map_page(struct vm_transfer *xfer)
{
	if (xfer->mapped_page) {
		if (xfer->span_index == xfer->mapped_index)
			return xfer->mapped_page;
		else
			kunmap(xfer->process_pages[xfer->mapped_index]);
	}

	xfer->mapped_index = xfer->span_index;
	xfer->mapped_page = kmap(xfer->process_pages[xfer->mapped_index]);

	return xfer->mapped_page;
}

static ssize_t vm_transfer_overlapping_segment_bytes(struct vm_transfer *xfer)
{
	return min_t(ssize_t, iov_buffer_cur_len(xfer->local), iov_buffer_cur_len(xfer->remote));
}

static void vm_transfer_advance(struct vm_transfer *xfer, size_t bytes)
{
	iov_buffer_advance(xfer->local, bytes);
	iov_buffer_advance(xfer->remote, bytes);
	xfer->total_bytes_copied += bytes;
}

static int vm_transfer_init(struct vm_transfer *xfer, struct task_struct *task,
		struct iov_buffer *remote, struct iov_buffer *local, bool vm_write,
		bool *remote_fault)
{
	unsigned long nr_pages = 0;
	size_t i;

	/* init the vm transfer description */
	xfer->process_pages = xfer->pp_stack;
	xfer->mm = NULL;
	xfer->task = task;
	xfer->page_address = 0;
	xfer->page_offset = 0;
	xfer->page_count = 0;
	xfer->span_pinned = 0;
	xfer->span_index = 0;
	xfer->mapped_page = NULL;
	xfer->mapped_index = 0;
	xfer->vm_write = vm_write;
	xfer->remote_fault = remote_fault;
	xfer->total_bytes_copied = 0;
	xfer->remote = remote;
	xfer->local = local;

	/* calculate the number of bytes to be transferred */
	xfer->total_bytes_requested = min_t(size_t, iov_buffer_bytes_remaining(remote),
			iov_buffer_bytes_remaining(local));

	/* calculate the number of pages needed to cover the largest remote iovec segment */
	for (i=remote->i_vec_idx; i < remote->i_cnt; i++) {
		ssize_t iov_len = remote->i_uvec[i].iov_len;
		if (iov_len > 0) {
			size_t offset = remote->i_vec_idx == i ? remote->i_vec_off : 0;
			nr_pages = max(nr_pages, pages_per_iov(&remote->i_uvec[i], offset));
		}
	}

	if (nr_pages > MAX_PP_ARRAY_COUNT) {
		/* allocate at most two pages to hold the struct page array */
		xfer->process_pages = kmalloc(min_t(size_t, MAX_KMALLOC_PAGES,
				sizeof(struct pages *) * nr_pages), GFP_KERNEL);

		if (!xfer->process_pages) {
			*(xfer->remote_fault) = false;
			return -ENOMEM;
		}
	}

	xfer->mm = get_task_mm(task);
	if (!xfer->mm) {
		*(xfer->remote_fault) = true;
		return -EACCES;
	}

	vm_transfer_debug(xfer);

	return 0;
}

static void vm_transfer_cleanup_pages(struct vm_transfer *xfer)
{
	int i;

	if (xfer->mapped_page) {
		kunmap(xfer->process_pages[xfer->mapped_index]);
		xfer->mapped_page = NULL;
	}

	/* span_pinned is the actual number of pages to clean up */
	for (i=0; i < xfer->span_pinned; i++) {
		if (xfer->vm_write && i < xfer->span_index)
			set_page_dirty_lock(xfer->process_pages[i]);
		put_page(xfer->process_pages[i]);
	}

	xfer->span_pinned = 0;
	xfer->span_index = 0;
}

static void vm_transfer_cleanup(struct vm_transfer *xfer)
{
	vm_transfer_cleanup_pages(xfer);

	if (xfer->process_pages != xfer->pp_stack)
		kfree(xfer->process_pages);

	if (xfer->mm)
		mmput(xfer->mm);
}

/**
 * vm_transfer_setup_pages - setup/update pinned pages for the current remote segment
 * @xfer: pointer to the vm transfer description to setup/updates pages for
 * Returns 0 on success, negative on error. Sets xfer->remote_fault if the error was
 * caused by a fault in the remote address space.
 */
static int vm_transfer_setup_pages(struct vm_transfer *xfer)
{
	unsigned long address, length, segment_end, pinned_end;

	/* skip past empty segments */
	if (iov_buffer_skip_empty(xfer->remote) || iov_buffer_skip_empty(xfer->local))
		return 0;

	address = iov_buffer_cur_base(xfer->remote);
	length = iov_buffer_cur_len(xfer->remote);
	segment_end = address + length;
	pinned_end = xfer->page_address + xfer->span_pinned * PAGE_SIZE;

	/* see if the current segment is outside of the currently pinned pages */
	if (address < xfer->page_address || segment_end > pinned_end) {
		vm_transfer_cleanup_pages(xfer);

		/* calculate the new page address for the first span page */
		xfer->page_address = address & PAGE_MASK;

		/* calculate how many pages the remote segment covers, limiting to a maximum */
		xfer->page_count = (address + length - 1) / PAGE_SIZE - address / PAGE_SIZE + 1;
		xfer->page_count = min_t(unsigned int, xfer->page_count, MAX_PAGES_PER_LOOP);

		down_read(&xfer->mm->mmap_sem);
		xfer->span_pinned = get_user_pages(xfer->task, xfer->mm, xfer->page_address,
				xfer->page_count, xfer->vm_write, 0, xfer->process_pages, NULL);
		up_read(&xfer->mm->mmap_sem);

		if (xfer->span_pinned != xfer->page_count) {
			*(xfer->remote_fault) = true;
			return -EFAULT;
		}
	}

	/* calculate which pinned page we're starting in and the offset within the page */
	xfer->span_index = (address - xfer->page_address) / PAGE_SIZE;
	xfer->page_offset = address & ~PAGE_MASK;

	return 0;
}

/**
 * vm_transfer_segments - iterate over local and remote segments and transfer data
 * @xfer: pointer to the vm transfer description
 * Returns 0 on success, error code otherwise.
 */
static int vm_transfer_segments(struct vm_transfer *xfer)
{
	int ret;
	uint8_t *target_kaddr;
	void __user *local_addr;
	ssize_t bytes_requested;

	while (vm_transfer_bytes_remaining(xfer) > 0) {
		vm_transfer_debug(xfer);

		ret = vm_transfer_setup_pages(xfer);
		if (ret < 0)
			return ret;

		/* transfer the min of overlapping bytes and bytes remaining in page */
		bytes_requested = vm_transfer_overlapping_segment_bytes(xfer);
		bytes_requested = min_t(ssize_t, bytes_requested, PAGE_SIZE - xfer->page_offset);

		target_kaddr = vm_transfer_map_page(xfer) + xfer->page_offset;
		local_addr = (void __user *) iov_buffer_cur_base(xfer->local);

		if (xfer->vm_write)
			ret = copy_from_user(target_kaddr, local_addr, bytes_requested);
		else
			ret = copy_to_user(local_addr, target_kaddr, bytes_requested);

		/* advance the buffers to account for progress this iteration */
		vm_transfer_advance(xfer, bytes_requested - ret);

		/* if any bytes were not transferred there was a fault in the local segment */
		if (ret) {
			*(xfer->remote_fault) = false;
			return -EFAULT;
		}
	}

	return 0;
}

/**
 * vm_transfer - transfer between local and remote memory
 * @remote: iov buffer of the remote memory
 * @local: iov buffer of the local memory
 * @task: task owning the remote memory
 * @vm_write: whether to write or read from the message payload
 * @remote_fault: when a fault happens, returns whether the fault was in
 *  the remote (true) or local (false) address space
 * Returns the number of bytes copied or an error code on failure.
 *
 * This modifies both remote and local. It is up to the caller to prevent
 * concurrent access to the members of remote and local.
 */
static ssize_t vm_transfer(struct iov_buffer *remote, struct iov_buffer *local,
		struct task_struct *task, bool vm_write, bool *remote_fault)
{
	struct vm_transfer xfer;
	ssize_t ret;

	BUG_ON(remote->i_type != IOV_BUFFER_TYPE_USER || local->i_type != IOV_BUFFER_TYPE_USER);

	ret = vm_transfer_init(&xfer, task, remote, local, vm_write, remote_fault);
	vm_transfer_debug_start(&xfer);
	if (ret < 0)
		goto error;

	ret = vm_transfer_segments(&xfer);
	if (ret < 0)
		goto error;

	ret = xfer.total_bytes_copied;

error:
	vm_transfer_debug_stop(&xfer);
	vm_transfer_cleanup(&xfer);

	return ret;
}

/**
 * vm_transfer_to_remote - transfer from local to remote memory
 * @remote: iov buffer of the remote memory
 * @local: iov buffer of the local memory
 * @task: task owning the remote memory
 * @remote_fault: when a fault happens, returns whether the fault was in
 *  the remote (true) or local (false) address space
 * Returns the number of bytes copied or an error code on failure.
 *
 * This modifies both remote and local. It is up to the caller to prevent
 * concurrent access to the members of remote and local.
 */
ssize_t vm_transfer_to_remote(struct iov_buffer *remote, struct iov_buffer *local,
		struct task_struct *task, bool *remote_fault)
{
	return vm_transfer(remote, local, task, true, remote_fault);
}

/**
 * vm_transfer_from_remote - transfer to local from remote memory
 * @remote: iov buffer of the remote memory
 * @local: iov buffer of the local memory
 * @task: task owning the remote memory
 * @remote_fault: when a fault happens, returns whether the fault was in
 *  the remote (true) or local (false) address space
 * Returns the number of bytes copied or an error code on failure.
 *
 * This modifies both remote and local. It is up to the caller to prevent
 * concurrent access to the members of remote and local.
 */
ssize_t vm_transfer_from_remote(struct iov_buffer *remote, struct iov_buffer *local,
		struct task_struct *task, bool *remote_fault)
{
	return vm_transfer(remote, local, task, false, remote_fault);
}

