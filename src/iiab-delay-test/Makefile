#*************************************************************************
#Title:    
#Authors:  Michael Petersen
#          
#File:     
#
#*************************************************************************

BASE_NAME = iiab_delay_test

BOARD  = arduino:avr:uno

ARDUINO = ~/arduino

help:
	@echo "make verify..... Build $(BASE_NAME)"
	@echo "make upload..... Flash the firmware"
	@echo "make terminal... Open uart terminal"

verify:
	$(ARDUINO) --verify --board $(BOARD) $(BASE_NAME).ino

upload:
	$(ARDUINO) --upload --board $(BOARD) $(BASE_NAME).ino

terminal:
	miniterm.py --filter=direct /dev/ttyUSB1 115200
