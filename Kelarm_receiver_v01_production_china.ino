/* Kelarm Receiver Production Prototype v01
 * All firmware code for the receiver device
 *  
 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include <SPI.h>
#include <LoRa.h>

// pin declarations
const int vibrator_pin = 3;    // attaches vibrator to atmega 328p pins digital pin 3
const int silent_mode_btn = 4; // attaches silent btn to atmega 328p pins digital pin 4
const int btn_pwr = 8;
const int on_led = 6;
const int off_led = 3;
const int buzzer_pin = 5;
const int voltage_measurement_pin = A0;

// variables
int lora_data_recevied = 0;
int vibrator_raw_value = 0;
int interval = 500;
int interval1 = 500;
int buzzer_pwm_val = 0;
int silent_on_btn_value = 0;

float actual_voltage = 0.00;
float raw_volt_val = 0.0;
float calibration_voltage = 4.029;
const float calibration_constant = 867;
const float low_power_threshold = 3.3;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

unsigned long previousMillis1 = 0;
unsigned long currentMillis1 = 0;

int led_state = LOW;
int silent_btn_count = 3;
bool silent_status = false;
bool buzzer_on_state = true;
bool buzzer_state = false;

bool silent_mode_on = false;
bool silent_mode_off = false;
bool low_power = false;

int last_silent_on_btn_value = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(vibrator_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(btn_pwr, OUTPUT);
  pinMode(silent_mode_btn, INPUT);
  pinMode(on_led, OUTPUT);
  pinMode(off_led, OUTPUT);

  digitalWrite(btn_pwr, HIGH);
  while (!Serial)
    ;
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void loop()
{
  button_silent_count();
  receive_lora_message();
  silent_mode_activated();
  take_voltage_levels(); //Function checks the bat voltage and updates the status
}
void takeReading()
{
  raw_volt_val = analogRead(voltage_measurement_pin);
  silent_on_btn_value = digitalRead(silent_mode_btn);
  Serial.println(silent_on_btn_value);
}
void receive_lora_message()
{
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available())
    {
      lora_data_recevied = LoRa.read();
      Serial.print((char)lora_data_recevied);
      //      Serial.print((char)LoRa.read());
      (lora_data_recevied == '7') ? power_led_vibrator_response() : power_off_led_indicators();
    }

    //         print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }

  if ((silent_btn_count % 2) != 0)
  {
    silent_status = true;
    if (silent_status)
    {
      buzzer_on_state = false;
    }
  }
  else
  {
    silent_status = false;
  }
}

int button_silent_count()
{
  takeReading();

  if (silent_on_btn_value != last_silent_on_btn_value)
  {
    if (silent_on_btn_value == 1)
    {
      silent_btn_count++;
      //      power_btn_count = 0;
    }
    delay(10);
  }
  last_silent_on_btn_value = silent_on_btn_value;

  if (silent_btn_count > 50)
  {
    silent_btn_count = 3;
  }

  if ((silent_btn_count % 2) != 0)
  {
    buzzer_on_state = true;
  }
  Serial.print("Silent Count: ");
  Serial.println(silent_btn_count);
}

void power_led_vibrator_response()
{
  unsigned long currentMillis1 = millis();
  if ((unsigned long)(currentMillis1 - previousMillis1) >= interval)
  {
    previousMillis1 = currentMillis1;

    if (led_state == LOW)
    {
      led_state = HIGH;
      buzzer_pwm_val = 100;
    }

    else
    {
      led_state = LOW;
      buzzer_pwm_val = 0;
    }
    digitalWrite(on_led, led_state);
    digitalWrite(vibrator_pin, led_state);
    if (buzzer_on_state)
    {
      buzzer_tone();
    }
  }
}

void power_off_led_indicators()
{
  digitalWrite(on_led, LOW);
  digitalWrite(vibrator_pin, LOW);
  noTone(buzzer_pin); // Stop sound..
}

void buzzer_tone()
{
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval)
  {
    previousMillis = currentMillis;
    if (!buzzer_state)
    {
      buzzer_state = true;
      tone(buzzer_pin, 2700); // Send 2.7KHz sound signal...
    }

    else
    {
      buzzer_state = !buzzer_state;
      noTone(buzzer_pin); // Stop sound..
    }
  }
}

void silent_mode_activated()
{
  if ((silent_btn_count % 2) == 0 && silent_on_btn_value == 1)
  {
    buzzer_on_state = false;
    digitalWrite(vibrator_pin, HIGH);
    delay(100);
    digitalWrite(vibrator_pin, LOW);
    delay(100);
  }
}

void power_on()
{
  take_voltage_levels();
  if (!low_power)
  {
    analogWrite(on_led, 127);
    delay(150);
    analogWrite(on_led, 0);
    delay(150);
    analogWrite(on_led, 127);
    delay(150);
    analogWrite(on_led, 0);
    delay(150);
    analogWrite(on_led, 127);
    delay(150);
    analogWrite(on_led, 0);
    delay(150);
  }
}

void take_voltage_levels()
{
  raw_volt_val = analogRead(voltage_measurement_pin);
  Serial.println(raw_volt_val);
  delay(300);

  actual_voltage = (calibration_voltage / calibration_constant) * raw_volt_val;
  Serial.print("Measured voltage is: ");
  Serial.print(actual_voltage);
  Serial.println(" v");

  if (actual_voltage > low_power_threshold)
  {
    low_power = true;
  }

  if (!low_power)
  {
    if (actual_voltage < low_power_threshold)
    {
      alert_low_pwr();
    }
  }
}

void alert_low_pwr()
{
  analogWrite(off_led, 127);
  delay(100);
  analogWrite(off_led, 127);
  delay(100);
  analogWrite(off_led, 127);
  delay(100);
  analogWrite(off_led, 127);
  delay(100);
  analogWrite(off_led, 127);
  delay(100);
  analogWrite(off_led, 127);
  delay(100);
}

void power_management()
{
  // turn off brown-out enable in software
  MCUCR = bit(BODS) | bit(BODSE);
  MCUCR = bit(BODS);

  interrupts(); // guarantees next instruction executed
  sleep_cpu();

  // cancel sleep as a precaution
  sleep_disable();
  // Turns off I2C
  power_twi_disable(); // TWI (I2C)

  // Turns off unused timers
  power_timer1_disable(); // Timer 1
}
