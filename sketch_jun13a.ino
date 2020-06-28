#include <Wire.h>

#include <MPU6050_tockn.h>
const uint8_t LED_PIN_0 = 2;
const uint8_t LED_PIN_1 = 3;
const uint8_t LED_PIN_2 = 4;
const uint8_t LED_PIN_CLEAN = 5;
const uint8_t LED_PIN_SW = 7;
const uint8_t H_BRIDGE_1A = 8;    // left motor red
const uint8_t H_BRIDGE_2A = 9;    // left motor black
const uint8_t H_BRIDGE_3A = 10;   // right motor black
const uint8_t H_BRIDGE_4A = 11;   // right motor red
const uint8_t H_BRIDGE_12EN = 12; // left motor enable
const uint8_t H_BRIDGE_34EN = 13; // right motor enable
const int GYRO_THRESHHOLD = 10;   //in degree
MPU6050 mpu6050(Wire);
enum state{waiting, flat, ascent, clean, clean_r, ascent_r, flat_r};
enum state state_cur;
float gyro_data[3];
int sw_state;

void update_state_led(enum state show_state){
  digitalWrite(LED_PIN_0, (show_state & 0b1));
  digitalWrite(LED_PIN_1, ((show_state >> 1) & 0b1));
  digitalWrite(LED_PIN_2, ((show_state >> 2) & 0b1));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(LED_PIN_0,OUTPUT);
  pinMode(LED_PIN_1,OUTPUT);
  pinMode(LED_PIN_2,OUTPUT);
  pinMode(H_BRIDGE_12EN,OUTPUT);
  pinMode(H_BRIDGE_34EN,OUTPUT);
  pinMode(H_BRIDGE_1A,OUTPUT);
  pinMode(H_BRIDGE_2A,OUTPUT);
  pinMode(H_BRIDGE_3A,OUTPUT);
  pinMode(H_BRIDGE_4A,OUTPUT);
  pinMode(LED_PIN_CLEAN,OUTPUT);
  pinMode(LED_PIN_SW,INPUT);
  state_cur = waiting;
}

void loop() {
  mpu6050.update();
  gyro_data[0] = mpu6050.getAngleX();
  gyro_data[1] = mpu6050.getAngleY();
  gyro_data[2] = mpu6050.getAngleZ();
  update_state_led(state_cur);
  switch (state_cur)
  {
    case waiting:
      digitalWrite(H_BRIDGE_12EN, LOW);
      digitalWrite(H_BRIDGE_34EN, LOW);
      sw_state = digitalRead(LED_PIN_SW);
      if (sw_state == LOW){
        //switch button is pressed
        state_cur = flat;
      }
      break;
    case flat:
      digitalWrite(H_BRIDGE_1A, HIGH);
      digitalWrite(H_BRIDGE_2A, LOW);
      digitalWrite(H_BRIDGE_3A, LOW);
      digitalWrite(H_BRIDGE_4A, HIGH);
      digitalWrite(H_BRIDGE_12EN, HIGH);
      digitalWrite(H_BRIDGE_34EN, HIGH);
      if (gyro_data[0] > GYRO_THRESHHOLD) {
        state_cur = ascent;
      }
      break;
    case ascent:
      if (gyro_data[0] < -GYRO_THRESHHOLD) {
        state_cur = clean;
      }
      break;
    case clean:
      digitalWrite(LED_PIN_CLEAN, HIGH);
      sw_state = digitalRead(LED_PIN_SW);
      if (sw_state == LOW){
        //switch button is pressed
        state_cur = clean_r;
      }
      break;
    case clean_r:
      digitalWrite(H_BRIDGE_1A, LOW);
      digitalWrite(H_BRIDGE_2A, HIGH);
      digitalWrite(H_BRIDGE_3A, HIGH);
      digitalWrite(H_BRIDGE_4A, LOW);
      if (gyro_data[0] < -GYRO_THRESHHOLD) {
        state_cur = ascent_r;
      }
      break;
    case ascent_r:
      digitalWrite(LED_PIN_CLEAN, LOW);
      if ((gyro_data[0] < GYRO_THRESHHOLD) && (gyro_data[0] > -GYRO_THRESHHOLD)) {
        state_cur = flat_r;
      }
      break;
    case flat_r:
      sw_state = digitalRead(LED_PIN_SW);
      if (sw_state == LOW){
        //switch button is pressed
        state_cur = waiting;
        delay(300);
      }
      break;
    default:
      break;
  }
  Serial.print("angleX : ");
  Serial.print(gyro_data[0]);
  Serial.print("\tangleY : ");
  Serial.print(gyro_data[1]);
  Serial.print("\tangleZ : ");
  Serial.println(gyro_data[2]);
}
