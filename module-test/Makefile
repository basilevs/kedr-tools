DIR:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
LIBCAMAC?=$(DIR)../libcamac/
MODULES?=$(LIBCAMAC)FedorovClient/camac/dfmodule


LDFLAGS+=-L/usr/local/lib -lcamacdfmodule -lcamacdf -ldfc
UNITS=ADC333 CamacErrorPrinter CamacAddressParser ADC333_test
VPATH=$(MODULES):$(MODULES)/tools:$(DIR)src
CPPFLAGS+= -I/usr/local/include/camac/dfmodule -I$(MODULES) -I$(MODULES)/tools -I$(DIR)/src 
OBJECTS=$(addsuffix .o,$(UNITS))

DEPGEN=$(CC) -M $(CPPFLAGS) $< | sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' > $@


all: adc333

report:
	@echo $(MAKEFILE_LIST)
	@echo $(VPATH)

%.cpp.d: %.cpp
	@$(DEPGEN)

DEP = $(addsuffix .cpp.d,$(UNITS))
ifeq "$(filter clean,$(MAKECMDGOALS))" ""
	-include $(DEP)
endif

adc333: $(OBJECTS)
	g++ $(CPPFLAGS) $^ $(LDFLAGS) -o $@ 

clean:
	-rm -f adc333
	-rm -f $(DEP)
	-rm -f $(OBJECTS)
