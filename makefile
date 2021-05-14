PORT=/dev/ttyUSB0

ifndef OBJDIR

.PHONY: default all clean

default: esp03
all: esp03 tinypico nano

clean:
	rm -rf $(shell pwd)/obj

esp03: export OBJDIR=$(shell pwd)/obj/esp03
esp03: export TARGET=esp8266:esp8266:generic
esp03: export BINARY=Payload.ino.bin

tinypico: export OBJDIR=$(shell pwd)/obj/tinypico
tinypico: export TARGET=esp32:esp32:tinypico
tinypico: export BINARY=Payload.ino.bin

nano: export OBJDIR=$(shell pwd)/obj/nano
nano: export TARGET=arduino:avr:nano
nano: export BINARY=Payload.ino.with_bootloader.bin

# recurse now that target-specific variables are defined
esp03 tinypico nano:
	@$(MAKE)

else

esp03 tinypico nano: $(OBJDIR)/$(BINARY)

PAYLOAD_VERSION = $(shell date +%s)

COMPILE = arduino-cli compile --fqbn $(TARGET) --output-dir $(OBJDIR) --build-cache-path $(OBJDIR)/cache --build-path $(OBJDIR)/build --build-property "compiler.cpp.extra_flags=\"-DPAYLOAD_VERSION=$(PAYLOAD_VERSION)\""
PROG = arduino-cli upload -p $(PORT) --fqbn $(TARGET) -v -i $(OBJDIR)/$(BINARY)

$(OBJDIR)/$(BINARY): Payload/Payload.ino makefile Payload/config.h Payload/config_private.h
	#building version $(PAYLOAD_VERSION)
	mkdir -p $(OBJDIR)/build
	mkdir -p $(OBJDIR)/cache
	rm -f $(OBJDIR)/prog.sh $(OBJDIR)/fota_server.sh
	$(COMPILE) $<
	echo $(PROG) > $(OBJDIR)/prog.sh
	chmod +x $(OBJDIR)/prog.sh
	mkdir -p $(OBJDIR)/fota
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat0.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/rocket0.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat1.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat2.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat3.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat4.version
	echo $(PAYLOAD_VERSION) > $(OBJDIR)/fota/pongsat13.version
	cp $(OBJDIR)/$(BINARY) $(OBJDIR)/fota/$(PAYLOAD_VERSION).bin
	echo pushd $(OBJDIR) > $(OBJDIR)/fota_server.sh
	echo python -m http.server 8080 >> $(OBJDIR)/fota_server.sh
	chmod +x $(OBJDIR)/fota_server.sh


endif
