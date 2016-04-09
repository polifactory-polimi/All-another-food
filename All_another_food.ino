/*
 * All another food
 * Copyright (C) 2016 Nicola Corna <nicola@corna.info>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <DallasTemperature.h>

const uint8_t one_wire_pin = A2;
const uint8_t tec_pin = 3;
const uint8_t potentiometer_pin = A0;
const uint8_t led_pin = LED_BUILTIN;

const uint8_t min_target_temp = 30;   //°C * 10^(-1)
const uint8_t max_target_temp = 130;  //°C * 10^(-1)

const uint8_t hysteresis = 16;  //°C * 10^(-1)

const uint16_t loop_delay_ms = 2000;

OneWire oneWire(one_wire_pin);
DallasTemperature sensors(&oneWire);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("All another food");

  pinMode(tec_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(potentiometer_pin, INPUT);

  digitalWrite(tec_pin, LOW);
  digitalWrite(led_pin, LOW);

  sensors.begin();
  sensors.setResolution(12);
}

void loop(void)
{ 
  int16_t cur_temperature;
  int16_t target_temperature;

  if (sensors.getDeviceCount() == 0) {
    Serial.print("No device found!!! Always on!!");
    cur_temperature = max_target_temp;
  }
  else {
    sensors.requestTemperatures();
    cur_temperature = round(sensors.getTempCByIndex(0) * 10);
    Serial.print("Current temperature: ");
    Serial.println(cur_temperature);
  }

  target_temperature = map(analogRead(potentiometer_pin), 0, 1023, max_target_temp, min_target_temp);

  if (target_temperature <= max_target_temp - 3) {
    Serial.print("Target temperature (plus/minus hysteresis): ");
    Serial.print(target_temperature);
    
    if (digitalRead(tec_pin) == HIGH)
      target_temperature -= hysteresis / 2;
    else
      target_temperature += hysteresis / 2;
  
    Serial.print(" (");
    Serial.print(target_temperature);
    Serial.println(")");
  }
  else {
    Serial.print("Target temperature: ");
    Serial.print(target_temperature);
    Serial.println(" (= OFF mode)");
    target_temperature = cur_temperature + hysteresis / 2 + 1;
  }

  if (cur_temperature >= target_temperature) {
    Serial.println("Turning on");
    digitalWrite(tec_pin, HIGH);
    digitalWrite(led_pin, HIGH);
  }
  else {
    Serial.println("Turning off");
    digitalWrite(tec_pin, LOW);
    digitalWrite(led_pin, LOW);
  }

  Serial.println();

  delay(loop_delay_ms);
}

