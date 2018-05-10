/*

  Copyright (c) 2017, Embedded Adventures
  All rights reserved.

  Contact us at source [at] embeddedadventures.com
  www.embeddedadventures.com

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  - Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

  - Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  - Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
  THE POSSIBILITY OF SUCH DAMAGE.

*/

// AS3935 MOD-1016 Lightning Sensor Arduino test sketch
// Written originally by Embedded Adventures


#include <Wire.h>
#include <AS3935.h> // https://github.com/dcoredump/AS3935_MOD-1016.git
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal.git
#include <looper.h>

#define IRQ_PIN 2

#define CLEAR_TIMER 2000

volatile bool detected = false;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
looper sched;
uint16_t clear_counter = millis()+CLEAR_TIMER;

void setup() {
  while (!Serial);
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Welcome to the MOD-1016 (AS3935) Lightning Sensor test sketch!");

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("LightningSensor");

  //I2C
  Wire.begin();
  mod1016.init(IRQ_PIN);
  //SPI
  //SPI.begin();
  //mod1016.init(IRQ_PIN, CS_PIN);

  lcd.setCursor(0, 1);
  lcd.print("Calibrating...");

  //Tune Caps, Set AFE, Set Noise Floor
  autoTuneCaps(IRQ_PIN);

  //mod1016.setTuneCaps(7);
  mod1016.setIndoors();
  mod1016.setNoiseFloor(6);


  Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");

  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
  mod1016.getIRQ();
  Serial.println("after interrupt");

  lcd.clear();

  sched.addJob(clear_screen, 1000);
}

void loop() {
  if (detected) {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }
  sched.scheduler();
}

void alert() {
  detected = true;
}

void translateIRQ(uns8 irq)
{
  lcd.setCursor(0, 0);

  switch (irq) {
    case 1:
      lcd.print("NOISE");
      Serial.println("NOISE DETECTED");
      break;
    case 4:
      lcd.print("DISTURBER");
      Serial.println("DISTURBER DETECTED");
      break;
    case 8:
      lcd.print("LIGHTNING");
      Serial.println("LIGHTNING DETECTED");
      printDistance();
      break;
  }
  clear_counter=millis();
}

void printDistance()
{
  lcd.setCursor(1, 0);
  int distance = mod1016.calculateDistance();
  if (distance == -1)
  {
    lcd.print("Out of range");
    Serial.println("Lightning out of range");
  }
  else if (distance == 1)
  {
    lcd.print("Dist. not avail.");
    Serial.println("Distance not in table");
  }
  else if (distance == 0)
  {
    lcd.print("Overhead!");
    Serial.println("Lightning overhead");
  }
  else {
    char buf[3];
    lcd.print(itoa(distance, buf, 2));
    lcd.print(" km");
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");
  }
}

void clear_screen() {
  if (clear_counter + CLEAR_TIMER < millis())
  {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    clear_counter=0;
  }
}

