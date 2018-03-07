dtb-$(CONFIG_ARCH_MSM8909) += apq8009w-triton.dtb \
                              apq8009w-triton_1gb.dtb

targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/,$(dtb-y))
clean-files := *.dtb
