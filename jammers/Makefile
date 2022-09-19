CONTIKI_PROJECT = constant-jammer sfd-jammer sfd-debugger random-jammer
all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += ble_dtm.c
PROJECT_SOURCEFILES += ble_dtm_hw_nrf52.c
PROJECT_SOURCEFILES += nrf52840-ieee.c

CONTIKI = ../contiki-ng
include $(CONTIKI)/Makefile.include
