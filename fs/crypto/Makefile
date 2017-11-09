obj-$(CONFIG_FS_ENCRYPTION)	+= fscrypto.o

fscrypto-y := crypto.o fname.o policy.o keyinfo.o
fscrypto-$(CONFIG_BLOCK) += bio.o
fscrypto-$(CONFIG_PFK) += fscrypt_ice.o
