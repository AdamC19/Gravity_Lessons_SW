INC_PATH=inc
SRC_PATH=src
BUILD_PATH=build
BIN_PATH=bin
CC=gcc
CCFLAGS=-Wall -lm -lrt -lpthread
CXX=g++
CXXFLAGS=-std=c++11 -Wall -lm -lrt -lpthread -O
CXXOBJFLAGS=$(CXXFLAGS) -c

TARGET_NAME=power_monitor
TARGET:=$(BIN_PATH)/$(TARGET_NAME)

SRC:=$(foreach x, $(SRC_PATH), $(wildcard $(addprefix $(x)/*,.cpp)))
OBJ:=$(addprefix $(BUILD_PATH)/, $(addsuffix .o, $(notdir $(basename $(SRC)))))
INC:=$(foreach x, $(INC_PATH), $(wildcard $(addprefix $(x)/*,.h)))

# clean files list
DISTCLEAN_LIST=$(OBJ)
CLEAN_LIST=$(TARGET) $(DISTCLEAN_LIST)

#default rule
default: makedir all

# non-phony targets
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -I$(INC_PATH) -o $@ $(OBJ)

$(BUILD_PATH)/%.o: $(SRC_PATH)/%.cpp
	$(CXX) $(CXXOBJFLAGS) -I$(INC_PATH) -o $@ $<

# phony rules
.PHONY: makedir
makedir:
	@mkdir -p $(BIN_PATH) $(BUILD_PATH) 

.PHONY: all
all:$(TARGET)

.PHONY: clean
clean:
	@echo CLEAN $(CLEAN_LIST)
	@rm -f $(CLEAN_LIST)

.PHONY: distclean
distclean:
	@echo CLEAN $(DISTCLEAN_LIST)
	@rm -f $(DISTCLEAN_LIST)