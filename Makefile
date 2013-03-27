include Makefile.common

LDFLAGS = $(COMMONFLAGS)		# default flags, architecure aso.
LDFLAGS += -Tstm32.ld 			# the linker script
LDFLAGS += -nostartfiles		# use custom start file: in this case this one ./libs/startup/*.a 
LDFLAGS += -Wl,--gc-sections	# garbage collection - only links used functions - needs ENTRY in ld-file

ODFLAGS = -S --section=.text --section=.isr_vector --section=.data

all: src libs
	$(LD) $(LDFLAGS) -o $(PROGRAM).elf \
		-Wl,--whole-archive \
			$(LIBDIR)/*.a \
			src/src.a \
		-Wl,--no-whole-archive
	$(OBJCOPY) -O ihex $(PROGRAM).elf $(PROGRAM).hex
	$(OBJCOPY) -O binary $(PROGRAM).elf $(PROGRAM).bin
	$(OBJDUMP) $(ODFLAGS) $(PROGRAM).elf > $(PROGRAM).lst

.PHONY: src libs clean
clean:
	-rm -f *.lst *.o *.elf *.bin *.hex
	$(MAKE) -C src $@
	$(MAKE) -C libs $@

src:
	$(MAKE) -C src $@

libs:
	$(MAKE) -C libs $@

