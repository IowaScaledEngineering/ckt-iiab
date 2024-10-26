/*************************************************************************
Title:    IIAB Delay Test Fixture
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Nathan D. Holmes & Michael D. Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

// Command to log data:
// (stty raw; cat > delay04.log) < /dev/ttyUSB1

void setup() {
	digitalWrite(2, HIGH);
	digitalWrite(3, HIGH);
	digitalWrite(4, HIGH);
	pinMode(2, OUTPUT);  // Approach A
	pinMode(3, OUTPUT);  // Diamond
	pinMode(4, OUTPUT);  // Approach B
	pinMode(5, INPUT);   // Signal A Green (Common Anode)
	Serial.begin(115200);
	while(!Serial);
}

unsigned long timeA, timeB;

uint32_t i = 0;

void loop() {
	digitalWrite(2, LOW);  // Trigger approach
	timeA = millis();
	while(HIGH == digitalRead(5));  // Wait for green
	timeB = millis();
	Serial.print(i);
	Serial.print(',');
	Serial.print(timeB-timeA);
	Serial.print('\n');
	delay(250);
	digitalWrite(3, LOW);  // Clear out interlocking
	delay(250);
	digitalWrite(4, LOW);
	delay(250);
	digitalWrite(2, HIGH);
	delay(250);
	digitalWrite(3, HIGH);
	delay(250);
	digitalWrite(4, HIGH);
	delay(500);
	i++;
}
