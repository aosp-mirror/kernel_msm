#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#
#HTC_AUD
obj-$(CONFIG_AMP_TFA9888)       += tfa9888.o

#NFC ++
obj-$(CONFIG_SENSORS_NFC_PN548) += pn548.o
obj-$(CONFIG_SENSORS_NFC_PN548) += pn548_htc.o
#NFC --

#Laser Sensor
obj-$(CONFIG_LASER_STMVL53L0_SUPPORT)   += stmvl53l0/
