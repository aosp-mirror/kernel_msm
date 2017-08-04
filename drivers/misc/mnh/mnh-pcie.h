/*
 *
 * MNH PCIe/DMA HOST Driver
 * Copyright (c) 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __MNH_PCIE_HOST
#define __MNH_PCIE_HOST
#include <linux/dma-mapping.h>
#include <linux/types.h>


/*******************************************************************************
 *
 *	Data structure
 *
 ******************************************************************************/

#define DMA_MAX_CHAN 4

/**
 * DMA channel direction type
 */
enum mnh_dma_chan_dir_t {
	DMA_EP2AP = 0, /**< EP2AP(WRITE_CH):To write data to AP from EP */
	DMA_AP2EP      /**< AP2EP(READ_CH):To read data from AP and send to EP*/

};

/**
 * DMA transfer status type
 * This is to indicate if DMA is transfer is successful or not
 */
enum mnh_dma_trans_status_t {
	DMA_DONE = 0,  /**<  DONE: DMA DONE interrupt */
	DMA_ABORT/**<  ABORT: DMA ABORT interrupt */
};

/**
 * DMA Channel status type
 */
enum mnh_dma_chan_status_t {
	DMA_CHAN_UNKNOWN = 0,	/**< Unknown */
	DMA_CHAN_RUNNING,		/**< Channel is running */
	DMA_CHAN_HALTED,		/**< Channel is halted */
	DMA_CHAN_STOPPED,		/**< Channel is stopped */
	DMA_CHAN_QUEUING	/**< Queuing. Not a real HW status */
};

/**
 * DMA hardware error types.
 * Defined in DMA_READ_ERR_STATUS_LOW_OFF/HIGH_OFF
 */
enum mnh_dma_error_t {
	DMA_ERR_NONE, /**< No DMA error found */
	DMA_ERR_WR,	/**< The DMA Write Channel has received an error
			 * response from the AHB/AXI bus (or RTRGT1 interface
			 * when the AHB/AXI Bridge is not used) while reading
			 * data from it. It's fatal error. */
	DMA_ERR_RD, /**< The DMA Read Channel has received an error response
			 * from the AHB/AXI bus (or RTRGT1 interface when the
			 * AHB/AXI Bridge is not used) while writing data to
			 * it. It's fatal error.*/
	DMA_ERR_FETCH_LL,/**< The DMA Write/Read Channel has received an error
			 * response from the AHB/AXI bus (or RTRGT1 interface
			 * when the AHB/AXI Bridge is not used) while reading
			 * a Linked List Element from local memory. It's fatal
			 * error. */
	DMA_ERR_UNSUPPORTED_RQST,
			/**< The DMA Read Channel has received a PCIe
			 * Unsupported Request CPL status from the remote
			 * device in response to the MRd Request.*/
	DMA_ERR_COMPLETER_ABORT,
			/**< The DMA Read Channel has received a PCIe Completer
			 *Abort CPL status from the remote device in response
			 *to the MRd Request. Non-fatal error.
			 */
	DMA_ERR_CPL_TIME_OUT,
			/**< The DMA Read Channel has timed-out while waiting
			 * for the remote device to respond to the MRd Request,
			 * or a malformed CplD has been received. Non-fatal
			 * error. */
	DMA_ERR_DATA_POISONING,
			/**< The DMA Read Channel has detected data poisoning
			 * in the CPL from the remote device in response to the
			 * MRd Request. Non-fatal error. */
};

/**
 * IRQ sent from AP to MNH
 */
enum mnh_irq_msg_t {
	IRQ_INVAL = 0,
	IRQ_START = 1,
	IRQ_MSG_SENT = IRQ_START,	/**< IRQ Msg generated interrupt */
	IRQ_DMA_STATUS,			/**< IRQ for DMA done/abort */
	IRQ_APPDEFINED_1,               /**< Application-defined */

	/* Add additional IRQ above here */
	IRQ_END = IRQ_APPDEFINED_1,
};

/* Interrupt(MSI) from MNH to AP */
enum mnh_msi_msg_t {
	MSI_INVAL = 0,
	MSI_GENERIC_START,
	MSI_MSG_SEND = MSI_GENERIC_START,
	MSI_PET_WATCHDOG,
	MSI_CRASH_DUMP,
	MSI_BOOTSTRAP_SET,
	MSI_APPDEFINED_1,
	/* Add additional MSIs above here */
	MSI_GENERIC_END = MSI_APPDEFINED_1,

	MSI_DMA_READ = 8,
	MSI_DMA_WRITE,
	/* Add additional DMA related MSIs below here */

	MSI_MAX = 31
};

/** Vendor message between AP to MNH */
struct mnh_pcie_vm {
	u32 vm;  /**< vendor defined message */
};

/**
 * Data structure defined for DMA block element information
 */
struct mnh_dma_element_t {
	uint32_t len;  /**< DMA transfer size */
	uint64_t src_addr; /**< SAR - DMA Source Address */
	uint64_t dst_addr; /**< DAR - DMA Destination Address */
};

/**
 * Structure used for DMA Channel status
 */
struct mnh_dma_state_info_t {
	uint8_t status; /**< dma chan status mnh_dma_chan_status_t) */
	uint32_t err; /**< hw error status if there is any (mnh_dma_error_t) */
	uint64_t xferred; /**< size of data have been transferred on ch */
};

/**
 * Structure used for mnh_set_outbound() API
 */
struct mnh_outb_region {
	uint8_t region; /**< iATU region to be programmed */
	uint64_t base_address; /**< start of src buffer */
	uint32_t limit_address; /**< end of src buffer */
	uint64_t target_pcie_address; /**< dest address */
};

/* TODO: REMOVE BELOW INBOUND enum/structure LATER AFTER BRING UP */
/** enum value used for mode setting of mth_set_inbound() API */
enum mth_iatu_mode_t {
	ADDR_MATCH = 0,
	BAR_MATCH
};

/** memmode values for struct mnh_inb_window */
#define IATU_MEM			0x0
#define IATU_IO				0x02

/** structure used for mth_set_inbound() API */
struct mnh_inb_window {
	uint8_t mode;			/**< type of region */
	uint32_t bar;			/**< BAR ignored in addr mode */
	uint8_t region;              /**< iATU region to be programmed */
	uint8_t memmode;
	uint64_t base_pcie_address;  /**< start of src buffer
					*ignored in BAR mode */
	uint32_t limit_pcie_address; /**< end of src buffer
					*ignored in BAR mode*/
	uint64_t target_mth_address; /**< dest address */
};

/**
 * Structure of scatter gather list entry
 */
struct mnh_sg_entry {
	phys_addr_t paddr; /**< Physical address */
	size_t size;	   /**< size of entry */
};

enum mnh_hotplug_event_t {
	MNH_HOTPLUG_IN = 0,
	MNH_HOTPLUG_OUT,
};

struct mnh_sg_list {
	struct page **mypage;
	struct scatterlist *sc_list;
	int n_num;
	int length;
	enum mnh_dma_chan_dir_t dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg_table;
};

typedef int (*irq_cb_t)(uint32_t irq);
typedef int (*irq_dma_cb_t)(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		enum mnh_dma_trans_status_t status);
typedef int (*hotplug_cb_t)(enum mnh_hotplug_event_t event, void *param);

/*******************************************************************************
 *
 *	APIs
 *
 ******************************************************************************/


/**
 * API to read data from PCIE configuration space
 * @param[in] offset  offset into PCIE configuration space(BAR0)
 * @param[in] len	buffer size : supported size is 4
 * @param[in] data	data to be read into. Client must allocate
 *			the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * an error happens during the process.(mnh_dma_chan_status_t)
 */
int mnh_pcie_config_read(uint32_t offset,  uint32_t len, uint32_t *data);

/**
 * API to write data to PCIE configuration space
 * @param[in] offset offset into PCIE configuration space(BAR0)
 * @param[in] len	 buffer size : supported size is 4
 * @param[in] data	 data to be written
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_pcie_config_write(uint32_t offset, uint32_t len, uint32_t data);

/**
 * API to read data from MNH configuration space
 * @param[in] offset    offset into MNH Address space(BAR2)
 * @param[in] len	buffer size : supported size is 1,2,4
 * @param[in] data	data to be read into. Client must allocate
 *			the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_config_read(uint32_t offset,  uint32_t len, uint32_t *data);

/**
 * API to write data to MNH configuration space
 * @param[in] offset  offset into MNH Address space(BAR2)
 * @param[in] len	buffer size : supported size could be 1,2,4
 * @param[in] data	 pointer to the data to be wrriten.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_config_write(uint32_t offset, uint32_t len, uint32_t data);

/**
 * API to read data from MNH DDR space
 * @param[in] offset offset into MNH DRAM space(BAR4)
 * @param[in] len buffer size
 * @param[in] data data to be read into. Client must allocate
 *			the buffer for reading and pass the pointer
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * @note Usually DMA will be used to read/write from DDR space in MNH
 *		 This maybe only used for debugging purpose
 */
int mnh_ddr_read(uint32_t offset,  uint32_t len, void *data);

/**
 * API to write data to MNH DDR space
 * @param[in] offset  offset into MNH DRAM space(BAR4)
 * @param[in] len	buffer size
 * @param[in] data	 pointer to the data to be wrriten.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 * @note Usually DMA will be used to read/write from DDR space in MNH
 *		 This maybe only used for debugging purpose
 */
int mnh_ddr_write(uint32_t offset, uint32_t len, void *data);

/**
 * API to generate IRQ from AP to MNH
 * @param[in] irq  IRQ ID to be sent (mnh_irq_msg_t)
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_send_irq(enum mnh_irq_msg_t irq);

/**
 * API to register IRQ callback to receive msg delivery from MNH
 * @param[in] msg_cb  interrupt handler for MSI received from MNH, pass NULL
 *			  to de-register
 * @param[in] vm_cb   interrupt handler for Vendor Message received from MNH,
 *			  pass NULL to de-register
 * @param[in] dma_cb  interrupt handler for DMA message(DMA_DONE/ABORT) received
 *			  from MNH, pass NULL to de-register
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_reg_irq_callback(irq_cb_t msg_cb, irq_cb_t vm_cb, irq_dma_cb_t dma_cb);

/**
 * API to send Vendor specific message from AP to MNH
 * @param[in] msg  Vendor message to be sent include msg_id
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_send_vendor_msg(struct mnh_pcie_vm msg);

/**
 * API to get the base ring buffer address of MNH
 * @param[out] rb_base ring buffer address
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_get_rb_base(uint64_t *rb_base);

/**
 * API to set outbound region
 * @param outb[in] - iATU outbound region information
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_set_outbound_iatu(struct mnh_outb_region *outb);

/**
 * API to set inbound region
 * @param outb[in] - iATU outbound region information
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_set_inbound_iatu(struct mnh_inb_window *inb);

/**
 * API to abort DMA transfer on specific channel
 * @param[in] chan	   The channel number for DMA transfer abort
 * @param[in] dir	   Direction of DMA channel, READ or WRITE
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_abort(uint8_t chan, enum mnh_dma_chan_dir_t dir);

/**
 * API to resume DMA transfer on specific channel
 * @param[in] chan	   The channel number for DMA transfer resume
 * @param[in] dir	   Direction of DMA channel, READ or WRITE
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_resume(uint8_t chan, enum mnh_dma_chan_dir_t dir);

/**
 * API to read/write single block on specific channel.
 * @param[in] chan The channel number for DMA transfer to be performed
 * @param[in] dir The channel direction (read/write)
 * @param[in] blk  one dma element info which include SAR(Source Address),
 *			  DAR(Destination Address) and transfer size.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_sblk_start(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		struct mnh_dma_element_t *blk);

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * buffer
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *		   overflow or sg[] not allocated.
 */
int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl);

/**
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_build
 * @return 0 for SUCCESS
 */
int mnh_sg_destroy(struct mnh_sg_list *sgl);

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int mnh_sg_retrieve_from_dma_buf(int fd, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int mnh_sg_release_from_dma_buf(struct mnh_sg_list *sgl);

/**
 * API to read/write multi blocks on specific channel.
 * Multi block read/write are based on Linked List(Transfer List) built by MNH.
 * @param[in] chan The channel number for DMA transfer to be performed
 * @param[in] dir The channel direction (read/write)
 * @param[in] start_addr Physical start_addr(in MNH) where transfer list is
 *			  stored.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */
int mnh_dma_mblk_start(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		phys_addr_t *start_addr);

/**
 * API to get the current status of a specified channel
 * @param[in] chan The channel number
 * @param[in] dir The direction of channel.
 * @param[out] info chan information includes channel status, transferred
 *				 size, error status, and etc.
 * @return 0 if success or -EINVAL or -EFATAL on failure
 */

int mnh_dma_get_status(uint8_t chan, enum mnh_dma_chan_dir_t dir,
		struct mnh_dma_state_info_t *info);

/**
 * Allocate DMA coherent host memory for shared PCIe/CPU access.  Must be
 * called in a sleepable context.
 * @param[in] size requested size of the memory region in bytes
 * @param[out] dma_addr returns the DMA address of the allocated memory
 * @return cpu virtual address of the allocated memory
 */
void *mnh_alloc_coherent(size_t size, dma_addr_t *dma_addr);

/**
 * Free DMA coherent memory allocated by mnh_alloc_coherent()
 * @param[in] size size of the memory region in bytes
 * @param[in] cpu_addr cpu virtual address of the memory region
 * @param[in] dma_addr DMA address of the memory region
 */
void mnh_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr);

/**
 * Map host memory for access by MNH PCIe host
 * @param[in] cpu_addr cpu virtual address of the memory region
 * @param[in] size size of the memory region in bytes
 * @param[in] direction DMA direction DMA_TO_DEVICE, etc.
 * @return DMA address returned by dma_map_single(), or zero for error
 */
dma_addr_t mnh_map_mem(
        void *cpu_addr, size_t size, enum dma_data_direction direction);

/**
 * Unmap host memory from MNH PCIe host access
 * @param[in] dma_addr DMA address of the memory returned by mnh_map_mem()
 * @param[in] size size of the memory region in bytes
 * @param[in] direction DMA direction DMA_TO_DEVICE, etc.
 */
void mnh_unmap_mem(
        dma_addr_t dma_addr, size_t size, enum dma_data_direction direction);

int mnh_pci_suspend(void);
int mnh_pci_resume(void);
#endif /* __MNH_PCIE_HOST */
