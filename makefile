# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT=Asservissement
INCDIRS=
LIBS_PREFIX=
LIBS_SUFFIX=
GPFLAGS=-DDATE_COMPIL='"$(shell date)"' -DAUTEUR_COMPIL='"$(USER)"' -DGIT_VERSION='"$(shell git describe --dirty --always)"'
PYTHON?=/usr/bin/python
NEWLIB_NANO=0

# On récupère les options de configs. Si le fichier build_config.mk
# n'existe pas, il doit être créé à partir de build_config.default.mk
CONFIG_FLAGS:=$(shell sed 's/\#.*//' build_config/build_config.mk)
GPFLAGS:=$(GPFLAGS) $(patsubst %, -DCONFIG_%, $(CONFIG_FLAGS))

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

CONFIGS:=$(shell find config -name "*.txt")

configs: $(CONFIGS)
$(CONFIGS): asserv/config/params.h
	@echo Regénération $@
	$(Q) $(PYTHON) gen_config.py $< $@
	

all: $(DEVICES) configs build_config/build_config.mk
term:
	picocom /dev/ttyACM0 -b 230400 --imap lfcrlf

.PHONY: all term configs
