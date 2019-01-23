/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/ion.h>
#include <linux/ion_4.12.h>
#include <linux/limits.h>
#include <linux/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

struct dmabuf_info {
	/**
	 * The total number of references held across the system.
	 */
	unsigned int count;
	/**
	 * The size of the buffer
	 */
	size_t size;
	/**
	 * A NULL-terminated name for the buffer.
	 */
	char *name;
};

struct memoryinfo_buffer {
	/**
	 * The buffer's unique inode.
	 */
	ino_t ino;
	/**
	 * The buffer's size, in bytes.
	 */
	size_t size;
	/**
	 * The total number of references held across the system.
	 */
	unsigned int count;
	/**
	 * Number of elements in \ref refs
	 */
	size_t refs_count;
	/**
	 * A NULL-terminated name for the buffer.
	 */
	char *name;
};

enum ion_version {
	ION_VERSION_UNKNOWN,
	ION_VERSION_MODERN,
	ION_VERSION_LEGACY
};

int parse_dmabuf_debug_fs(struct dmabuf_info *info, uint64_t inode_number)
{
	FILE *fp;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;

	fp = fopen("/sys/kernel/debug/dma_buf/bufinfo", "r");
	if (!fp)
		return -ENOENT;

	while ((read = getline(&line, &len, fp)) != -1) {
		long count;
		size_t size;
		unsigned long ino;
		// format: size   flags   mode   count   exp_name   ino   name
		int matched = sscanf(line, "%zu %*x %*x %ld %*s %lu %ms",
				     &size, &count, &ino, &info->name);

		if (matched > 2 && ino == inode_number) {
			info->count = count;
			info->size = size;
			free(line);
			return 0;
		}
	}
	free(line);
	return -ENOENT;
}

int parse_dmabuf_fd(pid_t pid, int fd, struct memoryinfo_buffer *buf)
{
	FILE *fp;
	char *line = NULL;
	ssize_t read;
	struct stat s;
	size_t len = 0;
	char fdinfo_path[100];
	int err = fstat(fd, &s);

	if (err < 0)
		error(1, errno, "fstat(%d) failed", fd);

	buf->ino = s.st_ino;
	buf->size = s.st_blocks * 512;
	if (!buf->size)
		error(1, EINVAL, "unknown size of the fd: %d", fd);

	snprintf(fdinfo_path, 100, "/proc/%d/fdinfo/%d", pid, fd);

	fp = fopen(fdinfo_path, "r");
	if (!fp)
		error(1, ENOENT, "cannot open file at: %s", fdinfo_path);

	while ((read = getline(&line, &len, fp)) != -1) {
		int matched = sscanf(line, "count:\t%u", &buf->count);

		matched = sscanf(line, "name:\t%m[^\n]", &buf->name);
	}
	free(line);
	return 0;
}

int memoryinfo_print(pid_t pid, int fd)
{
	struct dmabuf_info dmabuf;
	struct memoryinfo_buffer buf;
	int ret = parse_dmabuf_fd(pid, fd, &buf);

	ret = parse_dmabuf_debug_fs(&dmabuf, buf.ino);

	printf("dma_buf information parsed from fdinfo:\n");
	printf("inode: %lu, size: %zu, ref_count: %u, name: %s\n\n",
	       buf.ino, buf.size, buf.count, buf.name);
	printf("dma_buf information parsed from dma_buf debug fs:\n");
	printf("inode: %lu, size: %zu, ref_count: %u, name: %s\n\n",
	       buf.ino, dmabuf.size, dmabuf.count, dmabuf.name);
	if (dmabuf.size != buf.size || strcmp(dmabuf.name, buf.name))
		error(1, EINVAL, "dma_buf information is different!");
	return 0;
}

int ion_ioctl(int fd, int req, void *arg)
{
	int ret = ioctl(fd, req, arg);

	if (ret < 0)
		printf("ioctl %x failed with code %d: %s", req, ret,
		       strerror(errno));
	return ret;
}

int ion_free(int fd, ion_user_handle_t handle)
{
	struct ion_handle_data data = {
		.handle = handle,
	};
	return ion_ioctl(fd, ION_IOC_FREE, &data);
}

int get_ion_version(int fd)
{
	int err = ion_free(fd, (ion_user_handle_t)NULL);

	return (err == -ENOTTY) ? ION_VERSION_MODERN : ION_VERSION_LEGACY;
}

int ion_alloc(int fd, size_t len, size_t align, unsigned int heap_mask,
	      unsigned int flags, ion_user_handle_t *handle)
{
	int ret = 0;

	if (!handle)
		return -EINVAL;

	struct ion_allocation_data data = {
		.len = len,
		.align = align,
		.heap_id_mask = heap_mask,
		.flags = flags,
	};

	ret = ion_ioctl(fd, ION_IOC_ALLOC, &data);
	if (ret < 0)
		return ret;

	*handle = data.handle;

	return ret;
}

int ion_share(int fd, ion_user_handle_t handle, int *share_fd)
{
	int ret;
	struct ion_fd_data data = {
		.handle = handle,
	};

	if (!share_fd)
		return -EINVAL;

	ret = ion_ioctl(fd, ION_IOC_SHARE, &data);
	if (ret < 0)
		return ret;
	if (data.fd < 0)
		error(1, EINVAL, "share ioctl returned negative fd");
	*share_fd = data.fd;
	return ret;
}

int ion_alloc_fd(int fd, size_t len, size_t align, unsigned int heap_mask,
		 unsigned int flags, int *handle_fd, int version)
{
	ion_user_handle_t handle;
	int ret;

	if (version == ION_VERSION_MODERN) {
		struct ion_new_allocation_data data = {
			.len = len,
			.heap_id_mask = heap_mask,
			.flags = flags,
		};

		ret = ion_ioctl(fd, ION_IOC_NEW_ALLOC, &data);
		if (ret < 0)
			return ret;
		*handle_fd = data.fd;
	} else {
		ret = ion_alloc(fd, len, align, heap_mask, flags, &handle);
		if (ret < 0)
			return ret;
		ret = ion_share(fd, handle, handle_fd);
		ion_free(fd, handle);
	}
	return ret;
}

int alloc_dma_bufs(const char *name)
{
	int dma_buf_fd, version, err;
	int ion_fd = open("/dev/ion", O_RDONLY | O_CLOEXEC);

	if (ion_fd < 0)
		error(1, errno, "open /dev/ion failed");
	version = get_ion_version(ion_fd);
	err = ion_alloc_fd(ion_fd, 1024, 0, ION_HEAP_SYSTEM_MASK, 0,
			   &dma_buf_fd, version);
	if (err < 0)
		error(1, errno, "allocate ion buffer failed");

	err = ioctl(dma_buf_fd, DMA_BUF_SET_NAME, name);
	if (err < 0)
		error(1, errno, "set dma_buf name failed");

	return dma_buf_fd;
}

int free_dma_bufs(fd)
{
	close(fd);
	return 0;
}

int main(void)
{
	int pid, fd, ret;

	printf("Allocate dma_bufs\n\n");

	fd = alloc_dma_bufs("TEST");
	if (fd < 0)
		error(1, errno, "alloc dma_buf failed");

	pid = getpid();
	printf("print memoryinfo for pid: %d\n\n", pid);

	ret = memoryinfo_print(pid, fd);

	ret = free_dma_bufs(fd);
	return 0;
}
