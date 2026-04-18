# Makefile for MAX30102 Vital Monitor — Q-eHACK submission
# Targets: vital_resmgr  (driver/resource-manager process)
#          max30102_dashboard (display/UI process)
#
# Build environment: QNX SDP 7.x  (qcc / ntoaarch64-gcc)
# Target:            QNX Neutrino RTOS (aarch64 / armv7)
#
# Usage:
#   make            — build both binaries
#   make clean      — remove build artefacts
#   make deploy     — scp binaries to target board (set TARGET_IP)

CC        := qcc
CFLAGS    := -Wall -Wextra -Wpedantic -O2 -g \
             -D_QNX_SOURCE \
             -I.
LDFLAGS   := -lm -pthread

# QNX Screen library only needed by the dashboard
LDFLAGS_DASH := $(LDFLAGS) -lscreen

TARGET_IP ?=10.42.0.135 
DEPLOY_DIR := /tmp/vital_demo

.PHONY: all clean deploy

all: vital_resmgr max30102_dashboard

vital_resmgr: vital_resmgr.c vitals.h
	$(CC) $(CFLAGS) -o $@ vital_resmgr.c $(LDFLAGS)
	@echo "[BUILD] $@ OK"

max30102_dashboard: max30102_dashboard.c vitals.h
	$(CC) $(CFLAGS) -o $@ max30102_dashboard.c $(LDFLAGS_DASH)
	@echo "[BUILD] $@ OK"

clean:
	rm -f vital_resmgr max30102_dashboard

deploy: all
	ssh root@$(TARGET_IP) "mkdir -p $(DEPLOY_DIR)"
	scp vital_resmgr max30102_dashboard root@$(TARGET_IP):$(DEPLOY_DIR)/
	@echo "[DEPLOY] Binaries copied to $(TARGET_IP):$(DEPLOY_DIR)"
	@echo "[DEPLOY] On target run:  $(DEPLOY_DIR)/vital_resmgr &"
	@echo "[DEPLOY]                 $(DEPLOY_DIR)/max30102_dashboard"
