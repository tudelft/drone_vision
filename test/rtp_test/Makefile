include ../../Makefile.include

# Specify used paths
override CFLAGS += -I$(DIR_DV)
override CFLAGS += -I$(DIR_LIB)

executable := rtp_test.x

OBJECTS =  	rtp_test.o \
		$(DIR_LIB)/udp_socket.o \
		$(DIR_LIB)/encoding/rtp.o

$(executable): $(OBJECTS)

upload:
	sb2 make -C ./ all && $(DRONE_TOOL) upload_paparazzi ./$(executable) vision

all: $(executable)

%.o:: %.c
	$(QUIET_CC)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(bins):
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.x::
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.a::
	$(QUIET_LINK)$(AR) rcs $@ $^

clean:
	echo $(PROCESSOR)
	$(QUIET_CLEAN)$(RM) -v $(targets) $(bins) *.o *.d $(OBJECTS)

-include *.d
