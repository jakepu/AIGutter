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
const int LINE_TRACKER_LEFT_PIN = A7;
const int LINE_TRACKER_RIGHT_PIN = A6;
int BGLevel_left = 640;
int BGLevel_right = 800;
int detectLevel_right = 960;
int detectLevel_left = 910;
int message = 3;
BLEService gutterService("2e3ce4dd-7100-42ba-af41-39744c08ad15");
BLECharCharacteristic gutterModeChar("166d7175-3dcf-4967-9f9e-bba83a82ec6e", (BLERead | BLENotify | BLEWrite));
MPU6050 mpu6050(Wire);
enum state{waiting, flat, ascent, clean, clean_r, ascent_r, flat_r};
enum state state_cur;
enum state state_prev = flat;
enum motor_state{full_stop, forward, forward_right, forward_left, reverse, reverse_left, reverse_right};
enum motor_state motor_state_cur;
float gyro_data[3];
bool stop_flag = 0;
bool next_flag = 0;
void update_state_led(enum state show_state){
  digitalWrite(LED_PIN_0, (show_state & 0b1));
  digitalWrite(LED_PIN_1, ((show_state >> 1) & 0b1));
  digitalWrite(LED_PIN_2, ((show_state >> 2) & 0b1));
}
bool is_online(int tracker_pin){
  int BGLevel;
  int detectLevel;
  if (tracker_pin == LINE_TRACKER_LEFT_PIN){
    BGLevel = BGLevel_left;
    detectLevel = detectLevel_left;
  }
  else{
    BGLevel = BGLevel_right;
    detectLevel = detectLevel_right;
  }
  int level = analogRead(tracker_pin);
    // Are we looking for something that is darker than our normal level (say,
  //  a table edge, or a black stripe on a white surface) or something that
  //  is brighter than our normal level (say, a piece of copper tape? on a
  //  dark floor)?
  // Remember, the darker the surface, the higher the value returned.
  if (BGLevel < detectLevel) // Light-on-dark situation
  {
    // For a light-on-dark detection, we're looking to see if the level is
    //  higher than _BGLevel. Our threshold will be a rise above the BGLevel
    //  of 1/4 the difference between background and detect levels.
    int threshold = (detectLevel - BGLevel)>>2;
    if (level-threshold > BGLevel) return true;
    else                            return false;
  }
  else // Dark-on-light situation
  {
    // For a dark-on-light detection, we'll do exactly the opposite: check to
    //  see if the level is lower than _BGLevel by at least 1/4 the difference
    //  between the levels.
    int threshold = (BGLevel - detectLevel)>>2;
    if (level+threshold < BGLevel) return true;
    else                            return false;
  }
}
void update_motor_state(uint8_t mode){
  switch (mode)
  {
  case 0: // full stop
    motor_state_cur = full_stop;
    break;
  case 1: // forward
    Serial.print("left:");
    Serial.print(analogRead(LINE_TRACKER_LEFT_PIN));
    Serial.print(" right:");
    Serial.println(analogRead(LINE_TRACKER_RIGHT_PIN));
    if (is_online(LINE_TRACKER_LEFT_PIN) && !is_online(LINE_TRACKER_RIGHT_PIN)){
      motor_state_cur = forward_left;
    }
    else if (!is_online(LINE_TRACKER_LEFT_PIN) && is_online(LINE_TRACKER_RIGHT_PIN)){
      motor_state_cur = forward_right;
    }
    else {
      motor_state_cur = forward;
    }
    break;
  case 2: // reverse
    if (is_online(LINE_TRACKER_LEFT_PIN)){
      motor_state_cur = reverse_right;
    }
    else if (is_online(LINE_TRACKER_RIGHT_PIN)){
      motor_state_cur = reverse_left;
    }
    else{
      motor_state_cur = reverse;
    }
    break;
  default:
    motor_state_cur = full_stop;
    break;
  }

  switch (motor_state_cur)
  {
  case full_stop:
    digitalWrite(H_BRIDGE_12EN, LOW);
    digitalWrite(H_BRIDGE_34EN, LOW);
    break;
  case forward:
    digitalWrite(H_BRIDGE_12EN, HIGH);
    digitalWrite(H_BRIDGE_34EN, HIGH);
    digitalWrite(H_BRIDGE_1A, HIGH);
    digitalWrite(H_BRIDGE_2A, LOW);
    digitalWrite(H_BRIDGE_3A, LOW);
    digitalWrite(H_BRIDGE_4A, HIGH);
    break;
  case forward_left:
    digitalWrite(H_BRIDGE_12EN, HIGH);
    digitalWrite(H_BRIDGE_34EN, LOW);
    digitalWrite(H_BRIDGE_1A, HIGH);
    digitalWrite(H_BRIDGE_2A, LOW);
    break;
  case forward_right:
    digitalWrite(H_BRIDGE_12EN, LOW);
    digitalWrite(H_BRIDGE_34EN, HIGH);
    digitalWrite(H_BRIDGE_3A, LOW);
    digitalWrite(H_BRIDGE_4A, HIGH);
    break;
  case reverse:
    digitalWrite(H_BRIDGE_12EN, HIGH);
    digitalWrite(H_BRIDGE_34EN, HIGH);
    digitalWrite(H_BRIDGE_1A, LOW);
    digitalWrite(H_BRIDGE_2A, HIGH);
    digitalWrite(H_BRIDGE_3A, HIGH);
    digitalWrite(H_BRIDGE_4A, LOW);
    break;
  case reverse_right:
    digitalWrite(H_BRIDGE_12EN, LOW);
    digitalWrite(H_BRIDGE_34EN, HIGH);
    digitalWrite(H_BRIDGE_3A, HIGH);
    digitalWrite(H_BRIDGE_4A, LOW);
    break;
  case reverse_left:
    digitalWrite(H_BRIDGE_12EN, HIGH);
    digitalWrite(H_BRIDGE_34EN, LOW);
    digitalWrite(H_BRIDGE_1A, LOW);
    digitalWrite(H_BRIDGE_2A, HIGH);
    break;
  default:
    digitalWrite(H_BRIDGE_12EN, LOW);
    digitalWrite(H_BRIDGE_34EN, LOW);
    break;
  }
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
          update_motor_state(0);
          if (next_flag == 1){
            state_cur = flat;
            next_flag = 0;
          }
          break;
        case flat:
          update_motor_state(1);
          if (gyro_data[0] > GYRO_THRESHHOLD) {
            state_cur = ascent;
          }
          break;
        case ascent:
          update_motor_state(1);
          if (gyro_data[0] < -GYRO_THRESHHOLD) {
            state_cur = clean;
          }
          break;
        case clean:
          update_motor_state(1);
          digitalWrite(LED_PIN_CLEAN, HIGH);
          if (next_flag == 1){
            state_cur = clean_r;
            next_flag = 0;
          }
          break;
        case clean_r:
          update_motor_state(2);
          if (gyro_data[0] < -GYRO_THRESHHOLD) {
            state_cur = ascent_r;
          }
          break;
        case ascent_r:
          update_motor_state(2);
          digitalWrite(LED_PIN_CLEAN, LOW);
          if ((gyro_data[0] < GYRO_THRESHHOLD) && (gyro_data[0] > -GYRO_THRESHHOLD)) {
            state_cur = flat_r;
          }
          break;
        case flat_r:
          update_motor_state(2);
          if (next_flag == 1){
            state_cur = waiting;
            next_flag = 0;
          }
          break;
        default:
          break;
      }
      //Serial.println("Bluetooth connected");
      
      /*
      Serial.print("angleX : ");
      Serial.print(gyro_data[0]);
      Serial.print("angleY : ");
      Serial.print(gyro_data[1]);
      Serial.print("angleZ : ");
      Serial.println(gyro_data[2]);
      */
      
    }
  }

}
