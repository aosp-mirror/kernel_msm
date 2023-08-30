def boot_image_opts(
        boot_image_header_version = 4,
        base_address = 0x80000000,
        page_size = 4096,
        super_image_size = 0x10000000,
        boot_partition_size = 0x6000000,
        lz4_ramdisk = True,
        earlycon_addr = "0x04a98000",
        kernel_vendor_cmdline_extras = ["bootconfig"],
        board_kernel_cmdline_extras = [],
        board_bootconfig_extras = []):
    return struct(
        boot_image_header_version = boot_image_header_version,
        base_address = base_address,
        page_size = page_size,
        super_image_size = super_image_size,
        boot_partition_size = boot_partition_size,
        lz4_ramdisk = lz4_ramdisk,
        earlycon_addr = earlycon_addr,
        kernel_vendor_cmdline_extras = kernel_vendor_cmdline_extras,
        board_kernel_cmdline_extras = board_kernel_cmdline_extras,
        board_bootconfig_extras = board_bootconfig_extras,
    )
