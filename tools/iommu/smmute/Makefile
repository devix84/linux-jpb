CC		= $(CROSS_COMPILE)gcc
include ../../scripts/Makefile.include

ifeq ($(srctree),)
SRC		:= $(CURDIR)/
srctree 	:= $(SRC)../../../
else
SRC		:= $(srctree)/tools/iommu/smmute/
endif

CFLAGS 		+= -Wall -O2
# Order matters: we need list.h first
CFLAGS		+= -I$(srctree)/tools/include
CFLAGS		+= -I$(objtree)/usr/include

CFLAGS		+= -pthread
LIB_LDFLAGS	+= -pthread -lrt

SMMUTE		:= $(OUTPUT)smmute
DRIVERS		:= driver-kernel.o

ALL		+= $(SMMUTE)

all: $(ALL)

$(OUTPUT)%.o: $(SRC)%.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(SMMUTE): $(addprefix $(OUTPUT),main.o lib.o $(DRIVERS))
	$(CC) -o $@ $^ $(LIB_LDFLAGS)

clean:
	rm -vf $(OUTPUT)*.o

distclean: clean
	rm -vf $(ALL)

.PHONY: all clean distclean
