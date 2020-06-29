#include <Wire.h>
#include <ArduinoBLE.h>
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
const uint8_t H_BRIDGE_34EN = 14; // right motor enable
const int GYRO_THRESHHOLD = 10;   //in degree
int message = 3;
BLEService gutterService("2e3ce4dd-7100-42ba-af41-39744c08ad15");
BLECharCharacteristic gutterModeChar("166d7175-3dcf-4967-9f9e-bba83a82ec6e", (BLERead | BLENotify | BLEWrite)); 
MPU6050 mpu6050(Wire);
enum state{waiting, flat, ascent, clean, clean_r, ascent_r, flat_r};
enum state state_cur;
enum state state_prev = flat;
float gyro_data[3];
bool stop_flag = 0;
bool next_flag = 0;
void update_state_led(enum state show_state){
  digitalWrite(LED_PIN_0, (show_state & 0b1));
  digitalWrite(LED_PIN_1, ((show_state >> 1) & 0b1));
  digitalWrite(LED_PIN_2, ((show_state >> 2) & 0b1));
}
void checkButtonAction(){
  switch (message)
  {
  case 0:  //stop
    state_prev = state_cur;
    state_cur = waiting;
    stop_flag = 1;
    break;
  case 1: //continue
    if (stop_flag == 1){
      state_cur = state_prev;
    }
    else {
      next_flag = 1;
    }
    stop_flag = 0;
    break;
  case 2: //reverse
    state_cur = flat_r;
    break;
  default:
    break;
  }
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
  //BLE initialization
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
  Serial.println("starting BLE failed!");
  while (1);
  }
  BLE.setLocalName("AIGutter");
  BLE.setAdvertisedService(gutterService);
  gutterService.addCharacteristic(gutterModeChar);
  BLE.addService(gutterService);
  
}

void loop() {
  //make sure central device is connected
  digitalWrite(LED_BUILTIN, LOW);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      if (gutterModeChar.written()) {
        message = gutterModeChar.value();
        //Serial.println(message);
      } else {
        message = 3;
      }
      //Serial.println(message);
      checkButtonAction();
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
          if (next_flag == 1){
            state_cur = flat;
            next_flag = 0;
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
          if (next_flag == 1){
            state_cur = clean_r;
            next_flag = 0;
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
          if (next_flag == 1){
            state_cur = waiting;
            next_flag = 0;
          }
          break;
        default:
          break;
      }
      Serial.println("Bluetooth connected");
      /*
      Serial.print("angleX : ");
      Serial.print(gyro_data[0]);
      Serial.print("angleY : ");
      Serial.print(gyro_data[1]);
      Serial.print("angleZ : ");
      Serial.println(gyro_data[2]);
      */
      delay(500);
    }
  }
  delay(500);
}
