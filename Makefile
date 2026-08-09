obj-m += bme280_driver.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all: module overlay

module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

overlay:
	dtc -@ -Hepapr \
		-I dts \
		-O dtb \
		-o bme280-aos.dtbo \
		dts/bme280-aos-overlay.dts

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(RM) bme280-aos.dtbo
