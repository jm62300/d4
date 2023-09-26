TARGET_EXEC ?= testLib

BUILD_DIR ?= build
SRC_DIRS ?= src

SRCS := $(shell find $(SRC_DIRS) -name *.cpp)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

CPPFLAGS ?= -MMD -MP
LIBS := ../../build/libd4.a  -lboost_program_options -lz -O3 -lgmpxx -lgmp -Wl,-R../build/
INCLUDES := -I../../

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	@echo Linking: "$@ ( $(foreach f,$^,$(subst $(BUILD_DIR)/$(SRC_DIRS),,$f)) )"	
	$(CXX) $(OBJS) -o $@ $(LDFLAGS) $(INCLUDES) $(LIBS)
	
$(BUILD_DIR)/%.cpp.o: %.cpp
	@echo Compiling: $<
	@$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CFLAGS) -c $< -o $@ $(INCLUDES) -I../

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
