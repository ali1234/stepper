NAME = stepper
OBJECTS = main.o
TARGET = atmega328
FREQUENCY = 16000000
CC = avr-gcc
AS = avr-as
CFLAGS = -g -O3 -mmcu=$(TARGET) -DF_CPU=$(FREQUENCY)UL
LDFLAGS = -mmcu=$(TARGET) -Wl,--undefined=_mmcu,--section-start=.mmcu=0x910000
ASFLAGS = $(CFLAGS)
AVRDUDE = avrdude -c stk200 -p m168

all: $(NAME).bin $(NAME).hex

%.bin: %.elf
	avr-objcopy -j .text -j .data -O binary $< $@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex -R .eeprom $< $@

$(NAME).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $(NAME).elf $(OBJECTS)


upload: $(NAME).bin
	$(AVRDUDE) -U flash:w:$(NAME).bin

clean:
	rm -f *.bin *.elf *.o

simulate: $(NAME).elf
	simavr -m $(TARGET) -f $(FREQUENCY) $(NAME).elf

reset:
	$(AVRDUDE) -n
