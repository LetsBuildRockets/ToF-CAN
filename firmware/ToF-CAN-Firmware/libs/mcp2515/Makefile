#  Makefile template for Static library. 
# 1. Compile every *.cpp in the folder 
# 2. All obj files under obj folder
# 3. static library .a at lib folder
# 4. run 'make dirmake' before calling 'make'



CC=avr-gcc
CFLAGS= -funsigned-char -funsigned-bitfields -DNDEBUG -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.2.150\include" -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -lstdint -lstdbool -mmcu=atmega328pb -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.2.150\gcc\dev\atmega328pb" -c -mmcu=atmega328pb -DF_CPU=8000000
SOURCES= defaults.h global.h mcp2515.h mcp2515_defs.h
CFILES= mcp2515.c

UTILDIR=./
UTILFLAG=-ltseutil
UTILLIB=$(UTILDIR)libtseutil.a
UTILC=$(UTILDIR)hash.c $(UTILDIR)html.c $(UTILDIR)file.c $(UTILDIR)dictionary.c
UTILH=$(UTILC:.c=.h)

all:$(SOURCES) $(UTILDIR)*.h
	$(CC) $(CFLAGS) $(CFILES) 
	ar rcsv libMCP2515.a *.o

clean:
	rm -f *.o *.a Makefile.bak

rebuild: clean all  