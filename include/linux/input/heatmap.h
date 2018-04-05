/* SPDX-License-Identifier: GPL-2.0 */

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

typedef int16_t strength_t;

struct v4l2_heatmap {
	struct device *parent_dev;
	struct v4l2_device device;
	struct v4l2_pix_format format;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex lock;
	unsigned int input_index;

	size_t width;
	size_t height;

	struct v4l2_fract timeperframe;

	/* Used to protect access to the buffer queue */
	spinlock_t heatmap_lock;
	/* guarded by heatmap_lock */
	struct list_head heatmap_buffer_list;

	/*
	 * Function read_frame must be provided by the driver
	 * It should return true on successful heatmap read
	 * and false on failure
	 */
	bool (*read_frame)(struct v4l2_heatmap *v4l2, int16_t *data);
};

int heatmap_probe(struct v4l2_heatmap *v4l2);
void heatmap_remove(struct v4l2_heatmap *v4l2);
void heatmap_read(struct v4l2_heatmap *v4l2, uint64_t timestamp);