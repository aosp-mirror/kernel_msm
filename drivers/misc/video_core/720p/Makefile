
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/ddl
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/dec
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/enc
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/resource_tracker
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/scheduler
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/vcd
EXTRA_CFLAGS  += -Idrivers/misc/video_core/720p/init

obj-$(CONFIG_MSM_VIDEO_CORE_REG) += video_corereg.o
video_corereg-objs :=	ddl/vcd_ddl_firmware.o \
			ddl/vcd_ddl_metadata.o \
			ddl/video_core_720p.o \
			ddl/vcd_ddl_utils.o \
			ddl/vcd_ddl.o \
			ddl/vcd_ddl_helper.o \
			ddl/vcd_ddl_interrupt_handler.o \
			ddl/vcd_ddl_hal.o \
			ddl/vcd_ddl_properties.o \
			init/video_core_init.o \
			resource_tracker/vcd_res_tracker.o \
			scheduler/vid_frame_scheduler_utils.o \
			scheduler/vid_frame_scheduler.o \
			scheduler/vid_frame_scheduler_api.o \
			vcd/vcd_api.o \
			vcd/vcd_power_sm.o \
			vcd/vcd_client_sm.o \
			vcd/vcd_device_sm.o \
			vcd/vcd_sub.o \
			ddl/vcd_ddl_errors.o

obj-$(CONFIG_MSM_VIDEO_CORE_VDEC) += video_decoder.o

video_decoder-objs := dec/vdec.o

obj-$(CONFIG_MSM_VIDEO_CORE_VENC) += video_encoder.o

video_encoder-objs := enc/venc.o \
                    enc/venc_internal.o
