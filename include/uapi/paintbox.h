/*
 * Driver interface for the Paintbox Image Processing Unit
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PAINTBOX_H__
#define __PAINTBOX_H__

#include <linux/compiler.h>
#include <linux/ioctl.h>

/* TODO: There are several enumerations and types that are used in
 * C and C++ code across the kernel, QEMU, and Simulator.  A unified header file
 * hierarchy should be created to avoid type duplication.  This also has build
 * system implications as the components using these types are spread out over
 * multiple git projects and build systems.
 *
 * In the interim the following enums are specified here and must be kept in
 * sync with theer equivalent definitions elsewhere.
 *
 * This is being tracked by http://b/23560812
 */

/* Note this enum must be kept in sync with PaddingMethod in ipu_types.h */
enum ipu_padding_method {
	IPU_PADDING_DO_NOT_PAD = 0,
	IPU_PADDING_CONSTANT,
	IPU_PADDING_PERIODIC,
	IPU_PADDING_SYMMETRIC,
	IPU_PADDING_METHOD_SIZE
};

struct ipu_capabilities {
	uint32_t version_major;
	uint32_t version_minor;
	uint32_t version_build;
	uint32_t hardware_id;
	uint32_t num_stps;
	uint32_t num_interrupts;
	uint32_t num_lbps;
	uint32_t num_dma_channels;
	bool is_simulator;
	bool is_fpga;
	bool iommu_enabled;
};

enum sram_target_type {
	SRAM_TARGET_LBP = 0,
	SRAM_TARGET_STP_INSTRUCTION_RAM,
	SRAM_TARGET_STP_CONSTANT_RAM,
	SRAM_TARGET_STP_SCALAR_RAM,
	SRAM_TARGET_STP_VECTOR_RAM
};

enum dma_transfer_type {
	DMA_DRAM_TO_LBP = 0,
	DMA_DRAM_TO_STP,
	DMA_LBP_TO_DRAM,
	DMA_MIPI_TO_LBP,
	DMA_LBP_TO_MIPI,
	DMA_MIPI_TO_DRAM,
	DMA_SRC_DST_PAIRS
};

enum dma_dram_buffer_type {
	DMA_DRAM_BUFFER_UNUSED = 0,
	DMA_DRAM_BUFFER_USER,
	DMA_DRAM_BUFFER_DMA_BUF
};

struct dma_dram_config {
	enum dma_dram_buffer_type buffer_type;
	union {
		void __user *host_vaddr;
		struct {
			int fd;
			size_t offset_bytes;
		} dma_buf;
	};
	uint64_t len_bytes;
};

struct dma_lbp_config {
	uint32_t lbp_id;
	uint32_t lb_id;
	uint32_t read_ptr_id;
	int32_t start_x_pixels;
	int32_t start_y_pixels;
	bool gather;
};

struct dma_stp_config {
	enum sram_target_type sram_target;
	uint32_t stp_id;
	uint32_t sram_addr;
	bool include_halo;
};

enum dma_rgba_format {
	RGBA_FORMAT_DISABLED = 0,
	RGBA_FORMAT_RGBA,
	RGBA_FORMAT_ARGB,
	RGBA_FORMAT_COUNT
};

struct dma_image_config {
	uint64_t plane_stride_bytes;
	uint32_t width_pixels;
	uint32_t height_pixels;
	int32_t start_x_pixels;
	int32_t start_y_pixels;
	uint32_t row_stride_bytes;
	enum dma_rgba_format rgba_format;
	uint8_t bit_depth;
	uint8_t planes;
	uint8_t components;
	bool block4x4;
	bool mipi_raw_format;
};

struct dma_transfer_config {
	uint32_t channel_id;
	enum dma_transfer_type transfer_type;
	struct dma_image_config img;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config stp;
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} src;
	union {
		/* MIPI transfers do not require any additional settings */
		struct dma_stp_config stp;
		struct dma_dram_config dram;
		struct dma_lbp_config lbp;
	} dst;
	uint32_t sheet_width;
	uint32_t sheet_height;
	uint32_t stripe_height;
	uint32_t noc_outstanding;
	uint32_t retry_interval;

	/* Set to true when the runtime will be waiting for a completion
	 * notification.
	 */
	bool notify_on_completion;

	/* When auto_start_transfer is set to true the transfer will begin
	 * immediately if there are no pending transfers ahead of it and there
	 * is space in the active queue.  If the transfer is unable to start
	 * then it is placed on the pending queue.
	 *
	 * When auto_start_transfer is set to false the transfer will be placed
	 * on the pending queue and PB_START_DMA_TRANSFER will need to be
	 * called to start the transfer.
	 */
	bool auto_start_transfer;
};

/* TODO:  We can remove this when b/62371806 is fixed.
 */
struct dma_transfer_read {
	uint32_t channel_id;
	void __user *host_vaddr;
	size_t len_bytes;
};

struct dma_transfer_flush {
	uint32_t channel_id;
	bool flush_pending;
	bool flush_active;
	bool flush_completed;
};

struct padding_params {
	enum ipu_padding_method method;
	uint16_t value_or_period;
};

struct line_buffer_config {
	int32_t x_offset_pixels;
	int32_t y_offset_pixels;
	int32_t fb_offset_pixels;
	uint8_t lb_pool_id;
	uint8_t lb_id;
	uint8_t num_read_ptrs;
	uint8_t num_reuse_rows;
	uint8_t num_channels;
	uint8_t chan_offset_pixels;
	uint16_t fb_rows;
	uint16_t width_pixels;
	uint16_t height_pixels;
	uint16_t sb_rows;
	uint16_t sb_cols;
	uint32_t ipu_fb_base_addr;
	uint32_t ipu_sb_base_addr;
	struct padding_params padding;
};

struct line_buffer_reset {
	uint32_t lbp_id;
	uint32_t lb_id;
};

struct dma_interrupt_config {
	uint32_t channel_id;
	uint32_t interrupt_id;
};

struct paintbox_irq_event {
	/* Event time stamp using the boot time (including idle) */
	int64_t timestamp_ns;
	int error;

	/* Event data
	 * For STP interrupts, this will be the interrupt code.
	 * For MIPI input streams this will be the frame number.
	 */
	uint16_t data;
};

/* When the PB_IRQ_PRIORITY flag is set the irq group will return immediately
 * when the interrupt is triggered.
 */
#define PB_IRQ_PRIORITY (1 << 0)

/* The PB_IRQ_REQUIRED flag is used to indicate that this interrupt is required
 * and the irq group will not return until all required interrupts are
 * triggered.  Note that this flag is superceded by PB_IRQ_PRIORITY.
 */
#define PB_IRQ_REQUIRED (1 << 1)

struct paintbox_irq_wait_entry {
	struct paintbox_irq_event event;
	uint32_t interrupt_id;
	uint8_t flags;  /* PB_IRQ_* flags, see above. */
	bool triggered; /* output */
};

struct paintbox_irq_group_wait_base {
	/* Input only: Total number of IRQs in the in the IRQ group. */
	uint32_t irq_count;

	/* Input only: Number of required IRQs in the IRQ group. */
	uint32_t required_irq_count;

	/* Input: maximum wait period in nanoseconds for the IRQ group.  On
	 * ouput this will be the time remaining in the wait period.
	 */
	int64_t timeout_ns;

	/* Output: When the PB_WAIT_FOR_INTERRUPT ioctl returns this value will
	 * be the number of required IRQs that have triggered.
	 */
	unsigned int required_irqs_triggered;
};

struct paintbox_irq_group_wait {
	struct paintbox_irq_group_wait_base base;
	struct paintbox_irq_wait_entry irqs[0];
};

struct stp_config {
	unsigned int processor_id;
	const void __user *buf;
	size_t len;
};

struct stp_program_state {
	int32_t program_counter;
	uint8_t stp_id;
	bool enabled;
	bool stalled;
};

struct paintbox_all_stp_state {
	uint64_t enabled;
	uint64_t stalled;
};

struct stp_interrupt_config {
	uint32_t stp_id;
	uint32_t interrupt_id;
};

/* this structure must be followed by
 * sizeof(uint32_t) * inst_mem_size_in_instructions bytes
 */
struct stp_pc_histogram {
	/* input parameters */
	uint32_t stp_id;

	/* output parameters */
	uint32_t disabled;
	uint32_t running;
};

struct ipu_sram_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sram_byte_addr;
	uint32_t id;
	enum sram_target_type sram_target;
	bool swap_data;

	/* When set the driver will pad unaligned or short writes with zeros
	 * instead of doing a read-modify-write.  This feature is used for
	 * test and debugging and should not normally be set.
	 */
	bool pad_to_align;
};

struct ipu_sram_write_broadcast {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sram_byte_addr;
};

struct ipu_sram_read {
	void __user *buf;
	size_t len_bytes;
	uint32_t sram_byte_addr;
	uint32_t id;
	enum sram_target_type sram_target;
	bool swap_data;
};

struct sram_vector_coordinate_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t lane_group_x;
	uint32_t lane_group_y;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool write_alu_registers;
};

struct sram_vector_replicate_write {
	const void __user *buf;
	size_t len_bytes;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool write_alu_registers;
	bool write_halo_lanes;
};

struct sram_vector_coordinate_read {
	void __user *buf;
	size_t len_bytes;
	uint32_t lane_group_x;
	uint32_t lane_group_y;
	uint32_t sheet_slot;
	uint32_t byte_offset_in_lane_group;
	uint32_t id;
	bool read_alu_registers;
};

struct ipu_bulk_allocation_request {
	uint64_t stp_mask;
	uint64_t lbp_mask;
	uint64_t dma_channel_mask;
	uint64_t interrupt_mask;
	uint64_t timeout_ns;
};

enum pmon_block_type {
	PMON_BLOCK_BIF = 0,
	PMON_BLOCK_MMU = 1,
	PMON_BLOCK_DMA = 2,
	PMON_BLOCK_LBP = 3,
	PMON_BLOCK_STP = 4,
};

enum pmon_mode {
	PMON_MODE_DISABLED               = 0,
	PMON_MODE_SIMPLE_INCREMENT       = 1,
	PMON_MODE_ACCUMULATE             = 2,
	PMON_MODE_LESS_THAN_THRESHOLD    = 3,
	PMON_MODE_EQUAL_TO_THRESHOLD     = 4,
	PMON_MODE_GREATER_THAN_THRESHOLD = 5,
	PMON_MODE_COUNT_INCREMENT        = 6,
};

struct pmon_op {
	int32_t sel;
	int32_t mask;
	int32_t match;
	bool inv;
};

struct pmon_config {
	enum pmon_block_type block;
	int32_t core_id; /* Used by LBP or STP PMONs only */
	int32_t counter_id;

	enum pmon_mode mode;
	int32_t threshold;
	struct pmon_op inc;
	struct pmon_op dec;
};

struct pmon_data {
	enum pmon_block_type block;
	int32_t core_id; /* Used by LBP or STP PMONs only */
	int32_t counter_id;

	int64_t count;
	int32_t accumulator;
	bool accumulator_overflow;
	bool accumulator_underflow;
	bool count_overflow;
};

struct pmon_enable {
	enum pmon_block_type block;
	int core_id;

	bool enable;

	/* Used by DMA PMON block only, set to 0 otherwise */
	int channel_id_0;
	int channel_id_1;

	/* Used by LBP PMON block only, set to 0 otherwise */
	int rptr_id;
};

struct mipi_input_stream_setup {
	uint32_t seg_start;
	uint32_t seg_words_per_row;

	/* Note that this field is only evaluated if enable_on_setup in
	 * struct mipi_stream_setup is true.
	 *
	 * If disable_on_error is true then the stream will be disabled if an
	 * overflow error occurs.  If it is false then the stream will continue
	 * to run for the confgured number of frames or indefinitely if
	 * configured for free running.
	 */
	bool disable_on_error;
};

struct mipi_output_stream_setup {
	bool enable_row_sync;
};

struct mipi_stream_setup {
	uint32_t stream_id;
	uint32_t virtual_channel;
	/* Received data type */
	uint32_t data_type;

	/* Unpacked data type */
	uint32_t unpacked_data_type;
	uint32_t img_width;
	uint32_t img_height;
	uint32_t seg_end;
	uint32_t segs_per_row;
	uint32_t stripe_height;

	union {
		struct mipi_input_stream_setup input;
		struct mipi_output_stream_setup output;
	};

	/* If enable_on_setup is true then the stream will be enabled after the
	 * configuration is written, otherwise if false then the client will
	 * have to enable the stream.
	 */
	bool enable_on_setup;

	/* If enable_on_setup is true and free_running is true then the stream
	 * will run with this configuration until the client disables it.
	 */
	bool free_running;

	/* If enable_on_setup is true and free_running is false then the stream
	 * will run for the specified number of frames.
	 */
	int32_t frame_count;
};

struct mipi_stream_enable {
	uint32_t stream_id;

	/* If free_running is true then the stream will run with this
	 * configuration until the client disables it.
	 */
	bool free_running;

	/* If free_running is false then the stream will run for the specified
	 * number of frames.
	 */
	int32_t frame_count;

	union {
		struct {
			/* enable_row_sync is only used for output streams. */
			bool enable_row_sync;
		} output;
		struct {
			/* If disable_on_error is true then the stream will be
			 * disabled if an overflow error occurs.  If it is false
			 * then the stream will continue to run for the
			 * confgured number of frames or indefinitely if
			 * configured for free running.
			 */
			bool disable_on_error;
		} input;
	};
};

struct mipi_stream_enable_multiple {
	/* If enable_all is false then the streams specified in the
	 * stream_id_mask will be enabled.
	 */
	uint32_t stream_id_mask;

	/* If free_running is false then the stream will run for the specified
	 * number of frames.
	 */
	int32_t frame_count;

	/* If free_running is true then the stream will run with this
	 * configuration until the client disables it.
	 */
	bool free_running;

	/* If enable_all is true then all streams in the session will be
	 * enabled.  If enable_all is false then only the streams specified in
	 * stream_id_mask will be enabled.
	 */
	bool enable_all;

	union {
		struct {
			/* enable_row_sync is only used for output streams. */
			bool enable_row_sync;
		} output;
		struct {
			/* If disable_on_error is true then the stream will be
			 * disabled if an overflow error occurs.  If it is false
			 * then the stream will continue to run for the
			 * confgured number of frames or indefinitely if
			 * configured for free running.
			 */
			bool disable_on_error;
		} input;
	};
};

struct mipi_stream_disable_multiple {
	/* If disable_all is false then the streams specified in the
	 * stream_id_mask will be enabled.
	 */
	uint32_t stream_id_mask;

	/* If disable_all is true then all streams in the session will be
	 * enabled.  If disable_all is false then only the streams specified in
	 * stream_id_mask will be disabled.
	 */
	bool disable_all;
};

struct mipi_interrupt_config {
	uint32_t stream_id;
	uint32_t interrupt_id;
};

struct mipi_input_wait_for_quiescence {
	int64_t timeout_ns;
	unsigned int stream_id;
};

/* Ioctl interface to IPU driver
 *
 * The following ioctls will return these error codes on error conditions:
 *
 * -EACCES: Resource access error
 * -EEXIST: Resource exists or is already in use
 * -EFAULT: Buffer transfer error (rare)
 * -EINVAL: Invalid Parameter
 * -ENOENT: No entry
 * -ENOMEN: Out of memory
 * -ENOSYS: Unimplemented Functionality
 *
 */
/* TODO:  Reorganize and compress ioctl number space.  b/36068296 */
#define PB_GET_IPU_CAPABILITIES      _IOR('p', 1, struct ipu_capabilities)
#define PB_ALLOCATE_DMA_CHANNEL      _IOW('p', 2, unsigned int)
#define PB_SETUP_DMA_TRANSFER        _IOW('p', 3, struct dma_transfer_config)
#define PB_START_DMA_TRANSFER        _IOW('p', 4, unsigned int)
#define PB_BIND_DMA_INTERRUPT        _IOW('p', 5, struct dma_interrupt_config)
#define PB_UNBIND_DMA_INTERRUPT      _IOW('p', 6, unsigned int)
#define PB_RELEASE_DMA_CHANNEL       _IOW('p', 7, unsigned int)
#define PB_ALLOCATE_INTERRUPT        _IOW('p', 8, unsigned int)
#define PB_WAIT_FOR_INTERRUPT        _IOWR('p', 11, \
		struct paintbox_irq_group_wait)
#define PB_RELEASE_INTERRUPT         _IOW('p', 12, unsigned int)
#define PB_ALLOCATE_LINE_BUFFER_POOL _IOW('p', 13, unsigned int)
#define PB_SETUP_LINE_BUFFER         _IOW('p', 14, struct line_buffer_config)
#define PB_RESET_LINE_BUFFER_POOL    _IOW('p', 15, unsigned int)
#define PB_RESET_LINE_BUFFER         _IOW('p', 16, struct line_buffer_reset)
#define PB_RELEASE_LINE_BUFFER_POOL  _IOW('p', 17, unsigned int)
#define PB_ALLOCATE_PROCESSOR        _IOW('p', 18, unsigned int)
#define PB_SETUP_PROCESSOR           _IOW('p', 19, struct stp_config)
#define PB_START_PROCESSOR           _IOW('p', 20, unsigned int)
#define PB_RELEASE_PROCESSOR         _IOW('p', 21, unsigned int)

/* The STP will be nearly idle when it sends the DMA completion interrupt.
 * Following the interrupt there will be a small amount of cleanup work.
 * PB_GET_PROCESSOR_IDLE and PB_WAIT_FOR_ALL_PROCESSOR_IDLE can be used to
 * determine when the post interrupt cleanup work has completed.
 */

/* Returns 1 if idle, 0 not idle, < 0 error */
#define PB_GET_PROCESSOR_IDLE        _IOW('p', 22, unsigned int)
#define PB_WAIT_FOR_ALL_PROCESSOR_IDLE _IO('p', 23)

#define PB_WRITE_LBP_MEMORY          _IOW('p', 24, struct ipu_sram_write)
#define PB_READ_LBP_MEMORY          _IOWR('p', 25, struct ipu_sram_read)
#define PB_WRITE_STP_MEMORY          _IOW('p', 26, struct ipu_sram_write)
#define PB_READ_STP_MEMORY          _IOWR('p', 27, struct ipu_sram_read)
#define PB_STOP_PROCESSOR            _IOW('p', 28, unsigned int)
#define PB_RESUME_PROCESSOR          _IOW('p', 29, unsigned int)
#define PB_RESET_PROCESSOR           _IOW('p', 30, unsigned int)
#define PB_GET_PROGRAM_STATE        _IOWR('p', 31, struct stp_program_state)
#define PB_WRITE_VECTOR_SRAM_COORDINATES _IOW('p', 32, \
		struct sram_vector_coordinate_write)
#define PB_WRITE_VECTOR_SRAM_REPLICATE _IOW('p', 33,   \
		struct sram_vector_replicate_write)
#define PB_READ_VECTOR_SRAM_COORDINATES _IOW('p', 34, \
		struct sram_vector_coordinate_read)
#define PB_READ_DMA_TRANSFER         _IOW('p', 35, struct dma_transfer_read)
#define PB_ALLOCATE_MIPI_IN_STREAM   _IOW('p', 36, unsigned int)
#define PB_RELEASE_MIPI_IN_STREAM    _IOW('p', 37, unsigned int)
#define PB_SETUP_MIPI_IN_STREAM      _IOW('p', 38, struct mipi_stream_setup)
#define PB_ENABLE_MIPI_IN_STREAM     _IOW('p', 39, unsigned int)
#define PB_DISABLE_MIPI_IN_STREAM    _IOW('p', 40, unsigned int)

/* Returns frame number, < 0 error */
#define PB_GET_MIPI_IN_FRAME_NUMBER  _IOW('p', 41, unsigned int)

#define PB_CLEANUP_MIPI_IN_STREAM    _IOW('p', 42, unsigned int)
#define PB_ALLOCATE_MIPI_OUT_STREAM  _IOW('p', 45, unsigned int)
#define PB_RELEASE_MIPI_OUT_STREAM   _IOW('p', 46, unsigned int)
#define PB_SETUP_MIPI_OUT_STREAM     _IOW('p', 47, struct mipi_stream_setup)
#define PB_ENABLE_MIPI_OUT_STREAM    _IOW('p', 48, unsigned int)
#define PB_DISABLE_MIPI_OUT_STREAM   _IOW('p', 49, unsigned int)
#define PB_CLEANUP_MIPI_OUT_STREAM   _IOW('p', 51, unsigned int)
#define PB_BIND_MIPI_IN_INTERRUPT    _IOW('p', 54, struct mipi_interrupt_config)
#define PB_UNBIND_MIPI_IN_INTERRUPT  _IOW('p', 55, unsigned int)
#define PB_BIND_MIPI_OUT_INTERRUPT   _IOW('p', 56, struct mipi_interrupt_config)
#define PB_UNBIND_MIPI_OUT_INTERRUPT _IOW('p', 57, unsigned int)

/* Returns the number of transfers that have completed and are ready to be
 * read back into the userspace buffer.
 */
#define PB_GET_COMPLETED_UNREAD_COUNT _IOW('p', 58, unsigned int)
#define PB_RESET_ALL_PROCESSORS        _IO('p', 59)
#define PB_GET_ALL_PROCESSOR_STATES   _IOR('p', 60, \
		struct paintbox_all_stp_state)
#define PB_BIND_STP_INTERRUPT         _IOW('p', 62, struct stp_interrupt_config)
#define PB_UNBIND_STP_INTERRUPT       _IOW('p', 63, unsigned int)
#define PB_STOP_DMA_TRANSFER          _IOW('p', 64, unsigned int)
#define PB_FLUSH_DMA_TRANSFERS        _IOW('p', 65, struct dma_transfer_flush)
#define PB_STP_PC_HISTOGRAM_ENABLE    _IOW('p', 66, unsigned long)
#define PB_STP_PC_HISTOGRAM_READ      _IOWR('p', 67, struct stp_pc_histogram)
#define PB_STP_PC_HISTOGRAM_CLEAR     _IOW('p', 68, unsigned long)

/* Flush pending interrupts for an irq by interrupt id. */
#define PB_FLUSH_INTERRUPTS           _IOW('p', 69, unsigned int)

/* Flush pending interrupts for all irqs in the session. */
#define PB_FLUSH_ALL_INTERRUPTS        _IO('p', 70)

/* ioctls for configuring, reading, writing PMON counters */
#define PB_PMON_ALLOCATE              _IOR('p', 71, enum pmon_block_type)
#define PB_PMON_RELEASE               _IOR('p', 72, enum pmon_block_type)
#define PB_PMON_CONFIG_WRITE          _IOR('p', 73, struct pmon_config)
#define PB_PMON_DATA_READ            _IOWR('p', 74, struct pmon_data)
#define PB_PMON_DATA_WRITE            _IOW('p', 75, struct pmon_data)
#define PB_PMON_ENABLE                _IOW('p', 76, struct pmon_enable)

/* Enable or disable multiple MIPI streams */
#define PB_ENABLE_MIPI_IN_STREAMS     _IOW('p', 77, \
		struct mipi_stream_enable_multiple)
#define PB_DISABLE_MIPI_IN_STREAMS    _IOW('p', 78, \
		struct mipi_stream_disable_multiple)
#define PB_ENABLE_MIPI_OUT_STREAMS    _IOW('p', 79, \
		struct mipi_stream_enable_multiple)
#define PB_DISABLE_MIPI_OUT_STREAMS   _IOW('p', 80, \
		struct mipi_stream_disable_multiple)

#define PB_WAIT_FOR_MIPI_INPUT_QUIESCENCE _IOWR('p', 81, \
		struct mipi_input_wait_for_quiescence)

/* The big red button.
 * Allocations and interrupt bindings are preserved, but active and pending DMAs
 * are cancelled, MIPI streams are disabled, STPs are reset, and all setup is
 * cleared.
 * Only permitted on exclusive sessions.
 * Please use responsibly.
 */
#define PB_RESET_IPU                   _IO('p', 82)

/* Bulk resource allocation */
#define PB_BULK_ALLOCATE_IPU_RESOURCES	_IOW('p', 83, \
		struct ipu_bulk_allocation_request)
#define PB_BULK_RELEASE_IPU_RESOURCES	_IOW('p', 84, unsigned int)

#define PB_NUM_IOCTLS 85

/* Test ioctls
 * The following ioctls are for testing and are not to be used for normal
 * operation.  Whether or not the implementation of these ioctls is included in
 * the driver is governed by the PAINTBOX_TEST_SUPPORT kconfig.
 */
#define PB_TEST_DMA_RESET              _IO('t', 1)
#define PB_TEST_DMA_CHANNEL_RESET     _IOW('t', 2, unsigned int)
#define PB_TEST_MIPI_IN_RESET_STREAM  _IOW('t', 3, unsigned int)
#define PB_TEST_MIPI_OUT_RESET_STREAM _IOW('t', 4, unsigned int)
#define PB_TEST_LBP_BROADCAST_WRITE_MEMORY _IOW('t', 5, \
		struct ipu_sram_write_broadcast)

#endif /* __PAINTBOX_H__ */
