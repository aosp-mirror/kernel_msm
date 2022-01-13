/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI_TOUCH_OFFLOAD_H
#define _UAPI_TOUCH_OFFLOAD_H

#define TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION 2
#define TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION 0

#define TOUCH_OFFLOAD_MAGIC '7'

/* Bus interface type */
#define BUS_TYPE_I2C 0
#define BUS_TYPE_SPI 1
#define BUS_TYPE_I3C 2

/* Indicates full heatmap frame vs. partial */
#define HEATMAP_SIZE_PARTIAL  0
#define HEATMAP_SIZE_FULL     1

/* Touch channel data types */
#define TOUCH_DATA_TYPE_COORD	  0x0001
#define TOUCH_DATA_TYPE_RAW	  0x0002
#define TOUCH_DATA_TYPE_FILTERED  0x0004
#define TOUCH_DATA_TYPE_BASELINE  0x0008
#define TOUCH_DATA_TYPE_STRENGTH  0x0010

/* Touch channel scan types */
#define TOUCH_SCAN_TYPE_MUTUAL	  0x0040
#define TOUCH_SCAN_TYPE_SELF	  0x0080

/* "Context" channel types */
#define CONTEXT_CHANNEL_TYPE_DRIVER_STATUS  0x0100
#define CONTEXT_CHANNEL_TYPE_STYLUS_STATUS  0x0200
/* Inclusive range of valid context channels */
#define CONTEXT_CHANNEL_BIT_START 0x0100
#define CONTEXT_CHANNEL_BIT_END   0x0200

//////////////////////////////////////////////////////////////

/* TouchOffloadCaps
 *
 * touch_offload_major_version - Major version for breaking changes
 * touch_offload_minor_version - Minor version for small, compatible changes
 * device_id - device-specific identifier
 * display_width - width of device display in pixels
 * display_height - height of device display in pixels
 * tx_size - number of TX channels
 * rx_size - number of RX channels
 * bus_type - bus interface type
 * bus_speed_hz - bus frequency
 * heatmap_size - partial or full heatmap
 * touch_data_scan_types - channel data types available
 * touch_scan_types - channel scan types available
 * context_channel_types - bitfield of additional supported channels
 * continuous_reporting - driver supports continuous touch reports
 * noise_reporting - driver supports noise status messages
 * cancel_reporting - driver supports sending cancel events
 * size_reporting - driver supports size information
 * rotation_reporting - driver supports rotation information
 * filter_grip - driver supports disabling underlying grip suppression
 * filter_palm - driver supports disabling underlying palm rejection
 * num_sensitivity_settings - number of sensitivity options provided
 * auto_reporting - report heatmap when screen is not touched
 */
struct TouchOffloadCaps {
	/* Version info */
	__u32 touch_offload_major_version;
	__u32 touch_offload_minor_version;
	__u8 reserved1[8];

	/* Device info */
	__u32 device_id;
	__u16 display_width;
	__u16 display_height;
	__u16 tx_size;
	__u16 rx_size;
	__u8 bus_type;
	__u32 bus_speed_hz;
	__u8 reserved2[16];

	/* Algorithm info */
	__u8 heatmap_size;
	__u16 touch_data_types;
	__u16 touch_scan_types;
	__u16 context_channel_types;
	__u8 reserved3[16];

	/* Feature flags */
	__u8 continuous_reporting;
	__u8 noise_reporting;
	__u8 cancel_reporting;
	__u8 size_reporting;
	__u8 rotation_reporting;
	__u8 filter_grip;
	__u8 filter_palm;
	__u8 num_sensitivity_settings;
	__u8 auto_reporting;
	__u8 reserved4[32];
} __attribute__((packed));

/* TouchOffloadConfig
 *
 * continuous_reporting - enable continuous touch reports
 * noise_reporting - enable noise status messages
 * cancel_reporting - enable cancel events
 * filter_grip - enable underlying grip suppression
 * filter_palm - enable underlying palm rejection
 * sensitivity_setting - selected sensitivity
 * auto_reporting - enable reporting when screen is not touched
 * read_coords - allocate a channel to coordinate data
 * mutual_data_types - bitfield of mutual data types to collect
 * self_data_types - bitfield of self data types to collect
 * context_channel_types - bitfield of context channels to collect - overlays
 *                         on channel type bit mask
 */
struct TouchOffloadConfig {
	/* Feature flags */
	__u8 continuous_reporting;
	__u8 noise_reporting;
	__u8 cancel_reporting;
	__u8 filter_grip;
	__u8 filter_palm;
	__u8 sensitivity_setting;
	__u8 auto_reporting;
	__u8 reserved1[16];

	/* Data to read */
	__u8 read_coords;
	__u16 mutual_data_types;
	__u16 self_data_types;
	__u16 context_channel_types;
	__u8 reserved2[16];
} __attribute__((packed));

/* TouchOffloadFrameHeader
 *
 * frame_size - number of bytes in the frame
 * index - unique, sequential frame index
 * timestamp - frame timestamp in nanoseconds
 * num_channels - number of channels included in the frame
 */
struct TouchOffloadFrameHeader {
	__u32 frame_size;
	__u64 index;
	__u64 timestamp;
	__u8 num_channels;
} __attribute__((packed));

/* TouchOffloadChannelHeader
 *
 * channel_type - touch type stored in the channel
 * channel_size - size in bytes of the channel sample
 */
struct TouchOffloadChannelHeader {
	__u32 channel_type;
	__u32 channel_size;
} __attribute__((packed));

/* CoordStatus
 *
 * COORD_STATUS_INACTIVE - slot is unused
 * COORD_STATUS_FINGER - normal finger touch
 * COORD_STATUS_EDGE - edge touch
 * COORD_STATUS_PALM - palm touch
 * COORD_STATUS_CANCEL - canceled touch
 * COORD_STATUS_PEN - stylus pen touch
 */
enum CoordStatus {
	COORD_STATUS_INACTIVE = 0x00,
	COORD_STATUS_FINGER = 0x01,
	COORD_STATUS_EDGE = 0x02,
	COORD_STATUS_PALM = 0x03,
	COORD_STATUS_CANCEL = 0x04,
	COORD_STATUS_PEN = 0x05
};

/* Maximum number of touches that are tracked simultaneously */
#define MAX_COORDS 10

/* TouchOffloadCoord
 *
 * x - x component of touch location
 * y - y component of touch location
 * status - type of touch
 * major - size of the larger axis of the touch blob
 * minor - size of the smaller axis of the touch blob
 * pressure - z-axis or force exerted on touch touch point
 * rotation - signed rotation of major axis from y-axis, where -16384 is a
 *            full rotation to the left and 16384 is a rotation to the right.
 */
struct TouchOffloadCoord {
	__u16 x;
	__u16 y;
	enum CoordStatus status;
	__u32 major;
	__u32 minor;
	__u32 pressure;
	__s16 rotation;
	__u8 reserved1[16];
} __attribute__((packed));

/* TouchOffloadDataCoord
 *
 * header - header shared by all channels in a frame
 * coords - array of MAX_COORD coordinates
 */
struct TouchOffloadDataCoord {
	struct TouchOffloadChannelHeader header;
	struct TouchOffloadCoord coords[MAX_COORDS];
	__u8 reserved1[16];
} __attribute__((packed));
#define TOUCH_OFFLOAD_FRAME_SIZE_COORD (sizeof(struct TouchOffloadDataCoord))

/* TouchOffloadData2d
 *
 * header - header shared by all channels in a frame
 * tx_size - number of tx channels
 * rx_size - number of rx channels
 * data - pointer to raw touch data buffer
 */
struct TouchOffloadData2d {
	struct TouchOffloadChannelHeader header;
	__u16 tx_size;
	__u16 rx_size;
	__u8 reserved1[16];
	__u8 data[1];
} __attribute__((packed));
#define TOUCH_OFFLOAD_DATA_SIZE_2D(rx, tx) (sizeof(__u16)*(rx)*(tx))
#define TOUCH_OFFLOAD_FRAME_SIZE_2D(rx, tx) \
	(sizeof(struct TouchOffloadData2d) - 1 + \
	TOUCH_OFFLOAD_DATA_SIZE_2D((rx), (tx)))

/* TouchOffloadData1d
 *
 * header - header shared by all channels in a frame
 * tx_size - number of tx channels
 * rx_size - number of rx channels
 * data - pointer to raw touch data buffer
 */
struct TouchOffloadData1d {
	struct TouchOffloadChannelHeader header;
	__u16 tx_size;
	__u16 rx_size;
	__u8 reserved1[16];
	__u8 data[1];
} __attribute__((packed));
#define TOUCH_OFFLOAD_DATA_SIZE_1D(rx, tx) (sizeof(__u16)*((rx)+(tx)))
#define TOUCH_OFFLOAD_FRAME_SIZE_1D(rx, tx) \
	(sizeof(struct TouchOffloadData1d) - 1 + \
	TOUCH_OFFLOAD_DATA_SIZE_1D((rx), (tx)))

/* TouchOffloadDriverStatus
 *
 * header - header shared by all channels in a frame
 * contents - bitfield indicating the corresponding data field is valid
 * screen_state - 0 = off, 1 = on
 * display_refresh_rate - display refresh rate in hz
 * touch_report_rate - touch report rate in hz
 * noise_state - 0 = no noise, 1 = noise present
 * water_mode - 0 = normal mode, 1 = water mode
 * charger_state - 0 = no charger, 1 = charger connected
 * hinge_angle - angle in range [0=closed, 32767=open 360 degrees]
 * offload_timestamp - recorded when touch offload frame was generated
 */
struct TouchOffloadDriverStatus {
	struct TouchOffloadChannelHeader header;

	struct {
		__u32 screen_state : 1;
		__u32 display_refresh_rate : 1;
		__u32 touch_report_rate : 1;
		__u32 noise_state : 1;
		__u32 water_mode : 1;
		__u32 charger_state : 1;
		__u32 hinge_angle : 1;
		__u32 offload_timestamp : 1;
	} contents;
	__u8 reserved1[8];

	__u8 screen_state;
	__u8 display_refresh_rate;
	__u8 touch_report_rate;
	__u8 noise_state;
	__u8 water_mode;
	__u8 charger_state;
	__s16 hinge_angle;

	__u64 offload_timestamp;

	__u8 reserved2[32];
} __attribute__((packed));
#define TOUCH_OFFLOAD_FRAME_SIZE_DRIVER_STATUS \
		(sizeof(struct TouchOffloadDriverStatus))

/* TouchOffloadStylusStatus
 *
 * header - header shared by all channels in a frame
 * contents - bitfield indicating the corresponding data field is valid
 * coords - Up to MAX_COORDS active stylus touch points
 * coords_timestamp - timestamp at which stylus coordinates were received
 * pen_paired - 0 = no pen paired, 1 = a pen paired
 * pen_active - 0 = no activity, 1 = a pen is actively communicating
 */
struct TouchOffloadStylusStatus {
	struct TouchOffloadChannelHeader header;

	struct {
		__u32 coords : 1;
		__u32 coords_timestamp : 1;
		__u32 pen_paired : 1;
		__u32 pen_active : 1;
	} contents;
	__u8 reserved1[8];

	struct TouchOffloadCoord coords[MAX_COORDS];
	__u64 coords_timestamp;
	__u8 reserved2[16];

	__u8 pen_paired;
	__u8 pen_active;
	__u8 reserved3[16];
} __attribute__((packed));
#define TOUCH_OFFLOAD_FRAME_SIZE_STYLUS_STATUS \
		(sizeof(struct TouchOffloadStylusStatus))

////////////////////////////////////////////////////////////

/* TouchOffloadIocGetCaps
 *
 * caps - capabilities provided by the touch driver
 */
struct TouchOffloadIocGetCaps {
	struct TouchOffloadCaps caps;
	__u8 reserved1[16];
} __attribute__((packed));

/* TouchOffloadIocConfigure
 *
 * config - features to be used by the touch_offload client
 */
struct TouchOffloadIocConfigure {
	struct TouchOffloadConfig config;
	__u8 reserved1[16];
} __attribute__((packed));

/* TouchOffloadIocReport
 *
 * index - unique, sequential frame index
 * timestamp - frame timestamp in nanoseconds
 * num_coords - number of coordinates contained in "coords"
 * coords - array of coordinates to be reported to the driver
 */
struct TouchOffloadIocReport {
	__u64 index;
	__u64 timestamp;
	__u8 num_coords;
	__u8 reserved1[16];
	struct TouchOffloadCoord coords[MAX_COORDS];
} __attribute__((packed));

/* Ioctl to retrieve the capabilities of the touch driver */
#define TOUCH_OFFLOAD_IOC_RD_GETCAPS \
	_IOR(TOUCH_OFFLOAD_MAGIC, 0, struct TouchOffloadIocGetCaps)

/* Ioctl to set the configuration of the touch driver */
#define TOUCH_OFFLOAD_IOC_WR_CONFIGURE \
	_IOW(TOUCH_OFFLOAD_MAGIC, 1, struct TouchOffloadIocConfigure)

/* Ioctl to start the touch_offload pipeline */
#define TOUCH_OFFLOAD_IOC_START _IOC(TOUCH_OFFLOAD_MAGIC, 2)

/* Ioctl to report coordinates to the driver */
#define TOUCH_OFFLOAD_IOC_WR_REPORT \
	_IOW(TOUCH_OFFLOAD_MAGIC, 3, struct TouchOffloadIocReport)

/* Ioctl to stop the touch_offload pipeline */
#define TOUCH_OFFLOAD_IOC_STOP _IOC(TOUCH_OFFLOAD_MAGIC, 4)

#endif /* _UAPI_TOUCH_OFFLOAD_H */
