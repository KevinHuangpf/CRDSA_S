#
# OMNeT++/OMNEST Makefile for satnetnew
#
# This file was generated with the command:
#  opp_makemake -f --deep -I../../inet/src -L../../inet/out/$$\(CONFIGNAME\)/src -lINET -DINET_IMPORT -KINET_PROJ=../../inet
#

# Name of target to be created (-o option)
TARGET = satnetnew$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = -I../../inet/src -I.

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../../inet/out/$(CONFIGNAME)/src  -lINET
LIBS += -Wl,-rpath,`abspath ../../inet/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = \
    $O/CRDSA.o \
    $O/satGatewayAccessFwd.o \
    $O/satGatewayPhy.o \
    $O/satTerminalPhy.o \
    $O/simpleFwd.o \
    $O/thruputMeterOffset.o \
    $O/satFrameFwd_m.o \
    $O/satFrameRtn_m.o

# Message files
MSGFILES = \
    satFrameFwd.msg \
    satFrameRtn.msg

# Other makefile variables (-K)
INET_PROJ=../../inet

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS) -DINET_IMPORT $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f satnetnew satnetnew.exe libsatnetnew.so libsatnetnew.a libsatnetnew.dll libsatnetnew.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/CRDSA.o: CRDSA.cc \
	CRDSA.h
$O/satFrameFwd_m.o: satFrameFwd_m.cc \
	satFrameFwd_m.h
$O/satFrameRtn_m.o: satFrameRtn_m.cc \
	satFrameRtn_m.h
$O/satGatewayAccessFwd.o: satGatewayAccessFwd.cc \
	$(INET_PROJ)/src/inet/common/Compat.h \
	$(INET_PROJ)/src/inet/common/INETDefs.h \
	$(INET_PROJ)/src/inet/common/InitStages.h \
	$(INET_PROJ)/src/inet/common/TLVOption.h \
	$(INET_PROJ)/src/inet/common/TLVOption_m.h \
	$(INET_PROJ)/src/inet/linklayer/common/MACAddress.h \
	$(INET_PROJ)/src/inet/networklayer/common/IPProtocolId_m.h \
	$(INET_PROJ)/src/inet/networklayer/common/L3Address.h \
	$(INET_PROJ)/src/inet/networklayer/common/ModuleIdAddress.h \
	$(INET_PROJ)/src/inet/networklayer/common/ModulePathAddress.h \
	$(INET_PROJ)/src/inet/networklayer/contract/INetworkDatagram.h \
	$(INET_PROJ)/src/inet/networklayer/contract/ipv4/IPv4Address.h \
	$(INET_PROJ)/src/inet/networklayer/contract/ipv6/IPv6Address.h \
	$(INET_PROJ)/src/inet/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/inet/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/inet/networklayer/ipv6/IPv6Datagram.h \
	$(INET_PROJ)/src/inet/networklayer/ipv6/IPv6Datagram_m.h \
	satFrameFwd_m.h \
	satGatewayAccessFwd.h
$O/satGatewayPhy.o: satGatewayPhy.cc \
	CRDSA.h \
	satFrameFwd_m.h \
	satFrameRtn_m.h \
	satGatewayPhy.h
$O/satTerminalPhy.o: satTerminalPhy.cc \
	$(INET_PROJ)/src/inet/common/Compat.h \
	$(INET_PROJ)/src/inet/common/INETDefs.h \
	$(INET_PROJ)/src/inet/common/InitStages.h \
	$(INET_PROJ)/src/inet/common/NotifierConsts.h \
	$(INET_PROJ)/src/inet/common/TLVOption.h \
	$(INET_PROJ)/src/inet/common/TLVOption_m.h \
	$(INET_PROJ)/src/inet/linklayer/common/MACAddress.h \
	$(INET_PROJ)/src/inet/networklayer/common/IPProtocolId_m.h \
	$(INET_PROJ)/src/inet/networklayer/common/InterfaceEntry.h \
	$(INET_PROJ)/src/inet/networklayer/common/InterfaceToken.h \
	$(INET_PROJ)/src/inet/networklayer/common/L3Address.h \
	$(INET_PROJ)/src/inet/networklayer/common/L3AddressResolver.h \
	$(INET_PROJ)/src/inet/networklayer/common/ModuleIdAddress.h \
	$(INET_PROJ)/src/inet/networklayer/common/ModulePathAddress.h \
	$(INET_PROJ)/src/inet/networklayer/contract/INetworkDatagram.h \
	$(INET_PROJ)/src/inet/networklayer/contract/IRoute.h \
	$(INET_PROJ)/src/inet/networklayer/contract/IRoutingTable.h \
	$(INET_PROJ)/src/inet/networklayer/contract/ipv4/IPv4Address.h \
	$(INET_PROJ)/src/inet/networklayer/contract/ipv6/IPv6Address.h \
	$(INET_PROJ)/src/inet/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/inet/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/inet/networklayer/ipv6/IPv6Datagram.h \
	$(INET_PROJ)/src/inet/networklayer/ipv6/IPv6Datagram_m.h \
	CRDSA.h \
	satFrameFwd_m.h \
	satFrameRtn_m.h \
	satGatewayAccessFwd.h \
	satTerminalPhy.h
$O/simpleFwd.o: simpleFwd.cc \
	simpleFwd.h
$O/thruputMeterOffset.o: thruputMeterOffset.cc \
	$(INET_PROJ)/src/inet/common/Compat.h \
	$(INET_PROJ)/src/inet/common/INETDefs.h \
	$(INET_PROJ)/src/inet/common/InitStages.h \
	$(INET_PROJ)/src/inet/common/misc/ThruputMeter.h \
	thruputMeterOffset.h

