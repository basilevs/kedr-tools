DIR:=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
LIBCAMAC?=$(DIR)../libcamac/
MODULES?=$(LIBCAMAC)FedorovClient/camac/dfmodule

LDFLAGS+=-L/usr/local/lib -lcamacdfmodule -lcamacdf -ldfc
CUNITS=ADC333 CamacErrorPrinter CamacAddressParser  IntegralVoltmeter_C0309 ZIF_L0323 Discriminator_D0302
UNITS=$(CUNITS) ADC333_test failLog
VPATH=$(MODULES):$(MODULES)/tools:$(DIR)src
CPPFLAGS+= -I/usr/local/include/camac/dfmodule -I$(MODULES) -I$(MODULES)/tools -I$(DIR)/src 
OBJECTS=$(addsuffix .o,$(CUNITS))

DEPGEN=$(CC) -M $(CPPFLAGS) $< | sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' > $@


all: failLog

report:
	@echo $(MAKEFILE_LIST)
	@echo $(VPATH)

%.cpp.d: %.cpp
	@$(DEPGEN)

DEP = $(addsuffix .cpp.d,$(UNITS))
ifeq "$(filter clean,$(MAKECMDGOALS))" ""
	-include $(DEP)
endif

adc333: $(OBJECTS) ADC333_test.o
	$(CXX) $(CPPFLAGS) $^ $(LDFLAGS) -o $@ 

failLog: $(OBJECTS) failLog.o
	$(CXX) $(CPPFLAGS) $^ $(LDFLAGS) -o $@ 

clean:
	-rm -f adc333 failLog failLog.o ADC333_test.o
	-rm -f $(DEP)
	-rm -f $(OBJECTS)
