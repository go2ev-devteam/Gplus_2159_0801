TOPDIR := ../../../../../
include $(TOPDIR)sdk/build/core/config.mak


GOAL = cvr_monitor

OBJS += cvr_monitor.o


CFLAGS += -I$(SDK_DIR)/include
CFLAGS += -I$(PRODUCT_DIR)
CFLAGS += -I.
CFLAGS += -g -ggdb
CFLAGS += -D$(SYSCONFIG_ARCH)

LDFLAGS += -lpthread -lrt -L$(SDK_DIR)/lib -licver
ifeq ($(SYSCONFIG_ARCH), GPL32900B)
LDFLAGS += -ldl -lm -L$(SDK_DIR)/lib -lgpon2 -lcdsp
endif

LIBS += $(SDK_DIR)/lib/libdiag.a

.PHONY: all
all: $(GOAL)
	$(MSG_SPLIT_LINE)
	@$(MKDIR) -p $(PRODUCT_DIR)/system/app
	@$(CP) -u $(GOAL) $(PRODUCT_DIR)/system/app/$(GOAL)

$(GOAL): $(OBJS) $(LIBS) $(EXTLIB)
	$(MSG_SPLIT_LINE)
	$(MSG_LINKING)
	@$(TARGET_CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)
	$(MSG_GOAL_OK)

.PHONY: clean
clean:
	$(MSG_SPLIT_LINE)
	$(RM) -rf $(GOAL) $(OBJS) $(OBJS:.o=.d)
	$(RM) -f $(PRODUCT_DIR)/system/app/$(GOAL)


-include $(OBJS:.o=.d)
