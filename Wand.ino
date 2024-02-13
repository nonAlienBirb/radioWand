#include <SPI.h>
#include "RF24.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define CE_PIN 8
#define CSN_PIN 10
#define ledCheckPin 2

RF24 radio(CE_PIN, CSN_PIN);

Adafruit_MPU6050 mpu;

uint8_t address[][6] = { "1Node", "2Node" };
int stuff[3];
bool mode = true;
bool notPressed = true;

void setAndCheckMPU() {

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      digitalWrite(ledCheckPin, HIGH);
      delay(250);
      digitalWrite(ledCheckPin, LOW);
      delay(250);
    }
  }
  Serial.println("MPU6050 Found!");

  digitalWrite(ledCheckPin, HIGH);
  delay(200);
  digitalWrite(ledCheckPin, LOW);
  delay(200);
  digitalWrite(ledCheckPin, HIGH);
  delay(200);
  digitalWrite(ledCheckPin, LOW);

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void setAndCheckRadio() {

  if (!radio.begin()) {
    Serial.println("radio hardware is not responding!!");
    while (1) {
      digitalWrite(ledCheckPin, HIGH);
      delay(100);
      digitalWrite(ledCheckPin, LOW);
      delay(200);
    }  // hold in infinite loop
  }
  Serial.println("radio hardware is found.");
  delay(500);
  digitalWrite(ledCheckPin, HIGH);
  delay(100);
  digitalWrite(ledCheckPin, LOW);
  delay(100);
  digitalWrite(ledCheckPin, HIGH);
  delay(100);
  digitalWrite(ledCheckPin, LOW);

  radio.openWritingPipe(address[1]);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}

void setup() {

  Serial.begin(115200);
  pinMode(4, INPUT_PULLUP);
  pinMode(ledCheckPin, OUTPUT);

  setAndCheckMPU();
  setAndCheckRadio();

  Serial.println("Starting...");
  delay(1000);
}

void loop() {

  int s = digitalRead(4);

  if (s == 0 && notPressed) {
    notPressed = false;
    mode = !mode;
    delay(5);
  } else if (s == 1) {
    notPressed = true;
  }

  switch (mode) {
    case 1:
      digitalWrite(ledCheckPin, LOW);
      stuff[0] = map(analogRead(A0), 0, 1023, -255, 255);
      stuff[1] = map(analogRead(A1), 0, 1023, -100, 100);
      break;
    case 0:
      digitalWrite(ledCheckPin, HIGH);
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      stuff[0] = map(a.acceleration.x, -9, 9, -255, 255);
      stuff[1] = map(a.acceleration.y, -9, 9, -100, 100);
      
      break;
  }
  stuff[0] = stuff[0]>255 ? 255 : stuff[0]<-255 ? -255 : (((stuff[0]<=10)&&(stuff[0]>0)) || ((stuff[0]>=-10)&&(stuff[0]<0))) ? 0 : stuff[0];
  stuff[1] = stuff[1]>100 ? 100 : stuff[1]<-100 ? -100 : (((stuff[1]<=10)&&(stuff[1]>0)) || ((stuff[1]>=-10)&&(stuff[1]<0))) ? 0 : stuff[1];
  stuff[2] = mode;


  bool report = radio.write(&stuff, sizeof(stuff));

  if (!report) {
    Serial.print("transmition faild.");
  } else{
  Serial.print("transmited:  ");
  }

  Serial.print("x: ");
  Serial.print(stuff[0]);

  Serial.print("    y :");
  Serial.print(stuff[1]);

  Serial.print("    mode :");
  Serial.println(stuff[2]);

  delay(50);
}
