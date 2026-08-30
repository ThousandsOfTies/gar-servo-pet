CPPFLAGS ?=
CFLAGS ?= -O2
LDFLAGS ?=
HOST_CC ?= cc
TARGET_CC ?= aarch64-linux-gnu-gcc
TARGET_LDFLAGS ?= -static

WARNINGS := -Wall -Wextra -Werror -Wpedantic
PROJECT_CPPFLAGS := -Isrc -D_POSIX_C_SOURCE=200809L
PROJECT_CFLAGS := -std=c11 $(WARNINGS)
COMMON_SOURCES := src/config.c src/motion.c src/pca9685.c
APP_SOURCES := src/main.c $(COMMON_SOURCES)
TEST_BINS := \
	build/tests/test_config \
	build/tests/test_motion \
	build/tests/test_pca9685

.PHONY: all host-build target-build test check clean

all: host-build

host-build: build/host/gar-servoctl

target-build: build/target/gar-servoctl

build/host/gar-servoctl: $(APP_SOURCES) src/config.h src/motion.h src/pca9685.h Makefile
	@mkdir -p $(@D)
	$(HOST_CC) $(CPPFLAGS) $(PROJECT_CPPFLAGS) $(CFLAGS) $(PROJECT_CFLAGS) \
		$(APP_SOURCES) $(LDFLAGS) -o $@

build/target/gar-servoctl: $(APP_SOURCES) src/config.h src/motion.h src/pca9685.h Makefile
	@mkdir -p $(@D)
	$(TARGET_CC) $(CPPFLAGS) $(PROJECT_CPPFLAGS) $(CFLAGS) $(PROJECT_CFLAGS) \
		$(APP_SOURCES) $(LDFLAGS) $(TARGET_LDFLAGS) -o $@

build/tests/test_config: tests/test_config.c src/config.c src/config.h Makefile
	@mkdir -p $(@D)
	$(HOST_CC) $(CPPFLAGS) $(PROJECT_CPPFLAGS) $(CFLAGS) $(PROJECT_CFLAGS) \
		tests/test_config.c src/config.c $(LDFLAGS) -o $@

build/tests/test_motion: tests/test_motion.c src/motion.c src/motion.h \
		src/config.h src/pca9685.h Makefile
	@mkdir -p $(@D)
	$(HOST_CC) $(CPPFLAGS) $(PROJECT_CPPFLAGS) $(CFLAGS) $(PROJECT_CFLAGS) \
		tests/test_motion.c src/motion.c $(LDFLAGS) -o $@

build/tests/test_pca9685: tests/test_pca9685.c src/pca9685.c src/pca9685.h Makefile
	@mkdir -p $(@D)
	$(HOST_CC) $(CPPFLAGS) $(PROJECT_CPPFLAGS) $(CFLAGS) $(PROJECT_CFLAGS) \
		tests/test_pca9685.c src/pca9685.c $(LDFLAGS) -o $@

test: $(TEST_BINS) build/host/gar-servoctl tests/test_cli.sh tests/test_run.sh run
	@set -e; for test_binary in $(TEST_BINS); do ./$$test_binary; done
	@tests/test_cli.sh build/host/gar-servoctl
	@tests/test_run.sh ./run

check: host-build test

clean:
	rm -rf build
