#ifndef __SERVICEFS_IOV_BUFFER_H
#define __SERVICEFS_IOV_BUFFER_H

#include <linux/types.h>
#include <linux/uio.h>

typedef enum iov_buffer_type {
	IOV_BUFFER_TYPE_USER = 0,
	IOV_BUFFER_TYPE_KERNEL,
} iov_buffer_type_e;

struct iov_buffer {
	union {
		const struct iovec * i_uvec;
		const struct kvec *  i_kvec;
	};
	size_t                   i_cnt;
	size_t                   i_len;

	size_t                   i_vec_idx;
	size_t                   i_vec_off;
	size_t                   i_off;
	iov_buffer_type_e        i_type;
};

static inline size_t kiov_length(const struct kvec *iov, unsigned long nr_segs)
{
	unsigned long seg;
	size_t ret = 0;

	for (seg = 0; seg < nr_segs; seg++)
		ret += iov[seg].iov_len;

	return ret;
}

static inline void iov_buffer_uinit(struct iov_buffer *buf,
		const struct iovec *vec, size_t cnt)
{
	buf->i_uvec = vec;
	buf->i_cnt = cnt;
	buf->i_len = iov_length(vec, cnt);
	buf->i_vec_idx = 0;
	buf->i_vec_off = 0;
	buf->i_off = 0;
	buf->i_type = IOV_BUFFER_TYPE_USER;
}

static inline void iov_buffer_kinit(struct iov_buffer *buf,
		const struct kvec *vec, size_t cnt)
{
	buf->i_kvec = vec;
	buf->i_cnt = cnt;
	buf->i_len = kiov_length(vec, cnt);
	buf->i_vec_idx = 0;
	buf->i_vec_off = 0;
	buf->i_off = 0;
	buf->i_type = IOV_BUFFER_TYPE_KERNEL;
}

static inline void iov_buffer_reset(struct iov_buffer *buf)
{
	buf->i_vec_idx = 0;
	buf->i_vec_off = 0;
	buf->i_off = 0;
}

static inline size_t iov_buffer_bytes_remaining(const struct iov_buffer *buf)
{
	return buf->i_len - buf->i_off;
}

static inline unsigned long iov_buffer_cur_base(const struct iov_buffer *buf)
{
	if (buf->i_type == IOV_BUFFER_TYPE_USER)
		return (unsigned long) buf->i_uvec[buf->i_vec_idx].iov_base + buf->i_vec_off;
	else
		return (unsigned long) buf->i_kvec[buf->i_vec_idx].iov_base + buf->i_vec_off;
}

static inline unsigned long iov_buffer_cur_len(const struct iov_buffer *buf)
{
	if (buf->i_type == IOV_BUFFER_TYPE_USER)
		return buf->i_uvec[buf->i_vec_idx].iov_len - buf->i_vec_off;
	else
		return buf->i_kvec[buf->i_vec_idx].iov_len - buf->i_vec_off;
}

static inline size_t iov_buffer_advance(struct iov_buffer *buf, size_t bytes)
{
	size_t d;
	size_t count = 0;
	size_t off = buf->i_vec_off;

	if (buf->i_type == IOV_BUFFER_TYPE_USER) {
		const struct iovec *iov = &buf->i_uvec[buf->i_vec_idx];

		while (bytes && buf->i_off < buf->i_len) {
			d = min(bytes, iov->iov_len - off);

			bytes -= d;
			count += d;
			off += d;
			buf->i_off += d;

			if (iov->iov_len == off) {
				iov++;
				off = 0;
			}
		}

		buf->i_vec_off = off;
		buf->i_vec_idx = iov - buf->i_uvec;
	} else {
		const struct kvec *iov = &buf->i_kvec[buf->i_vec_idx];

		while (bytes && buf->i_off < buf->i_len) {
			d = min(bytes, iov->iov_len - off);

			bytes -= d;
			count += d;
			off += d;
			buf->i_off += d;

			if (iov->iov_len == off) {
				iov++;
				off = 0;
			}
		}

		buf->i_vec_off = off;
		buf->i_vec_idx = iov - buf->i_kvec;
	}

	return count;
}

/**
 * iov_buffer_skip_empty - skip past zero-length iovecs
 * @buf: iov buffer to advance
 * Returns true if the end of the iov buffer was reached, false otherwise
 */
static inline bool iov_buffer_skip_empty(struct iov_buffer *buf)
{
	while (buf->i_vec_idx < buf->i_cnt
			&& iov_buffer_cur_len(buf) == 0) {
		buf->i_vec_idx++;
	}

	return buf->i_vec_idx == buf->i_cnt;
}

#endif
