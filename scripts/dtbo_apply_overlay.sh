#!/bin/bash

# Treble kerenl requires the SoC device tree and the device specific
# device tree overlay to be built separately. Before the bootloader
# supports applying overlay, the overlay step to form the dtb passed
# to the linux kernel is done on the host side with this script.

set -x

# prepare overlay workspace
overlay_dir=`mktemp -d -t overlay.XXXXXXXXXX`
TOP=${PWD}
KERNEL_DTB=${TOP}/$1
DTBO=${TOP}/$2

trap "rm -rf ${overlay_dir}; set +x; exit" 1 SIGINT

cd ${overlay_dir}

# extract soc dtb
extract_dtb ${KERNEL_DTB} soc.dtb Image.lz4

soc_cnt=$(ls -l soc.dtb* | wc -l)
if [ ${soc_cnt} -ne 1 ]; then
  echo "Error: ${soc_cnt} soc.dtb(s) appended to Image.lz4-dtb" >&2
  exit 1
fi

# extract dtbo, rev and id
mkdtimg dump ${DTBO} -b overlay.dtbo > dtbo.info
id_arr=(`grep dtbo.info -e "id" | sed 's/.*id = \([0-9a-f]\+\)/\1/'`)
rev_arr=(`grep dtbo.info -e "rev" | sed 's/.*rev = \([0-9a-f]\+\)/\1/'`)

# add number if there's only one dtbo entry
if [ ${#id_arr[*]} -eq 1 ]; then
  mv overlay.dtbo overlay.dtbo.0
fi

for idx in ${!id_arr[*]}; do
  echo "apply overlay for device id=${id_arr[$idx]} rev=${rev_arr[$idx]}"
  ufdt_apply_overlay soc.dtb overlay.dtbo.${idx} combined-${idx}.dtb
  dtc -q -O dts -o combined-${idx}.dts combined-${idx}.dtb
  echo "/{qcom,board-id=<0x${id_arr[$idx]} 0x${rev_arr[$idx]}>;};" >> \
    combined-${idx}.dts
  dtc -q -O dtb -o combined-${idx}.dtb combined-${idx}.dts
done

# ls -v is used to make sure dtb is combined as the order specified in
# dtboimg.cfg, where muskie/walley dtbs are before taimen dtbs. Otherwise,
# muskie/walleye's bootloader may give up looking for dtbs once it sees
# an invalid dtb, i.e. taimen dtb.
cat Image.lz4 `ls -v combined-*.dtb` > Image.lz4-dtb
cp Image.lz4-dtb ${KERNEL_DTB}

rm -rf ${overlay_dir}
set +x
