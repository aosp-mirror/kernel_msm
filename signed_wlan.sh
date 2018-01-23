export PATH=$PATH:~/sardine_workspace/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin/

cp drivers/staging/prima/wlan.ko .
perl scripts/sign-file sha512 signing_key.priv signing_key.x509 wlan.ko
arm-linux-androideabi-strip --strip-debug wlan.ko
cp wlan.ko ~/sardine_workspace/device/compal/sardine-kernel/
