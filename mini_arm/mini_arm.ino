#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <map>
#include <stdlib.h>
#include <vector>


template<class T>
const T& clamp(const T& v, const T& lo, const T& hi) {
  return v < lo ? lo : hi < v ? hi : v;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct __attribute__ ((packed)) arm_data{
  uint8_t head;
  uint8_t data[32];
  uint8_t reserved;
  uint8_t tail;
};

// Use OpenRB-150.
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
#define USART Serial2
const int DXL_DIR_PIN = -1;

#define SERVOS 6  // Arm has 6 servos.
#define STATIC_CURRENT 300
#define MOVING_CURRENT 8

#define upper_channel 1935
#define middle_channel 1509
#define lower_channel 1087



const float DXL_PROTOCOL_VERSION = 2.0;
const std::map<uint8_t, std::pair<float, float>> MECHANICAL_ANGLES = {
  {0, std::make_pair(0,360)},
  {1, std::make_pair(272-40, 272+17)},
  {2, std::make_pair(123.7,274)},
  {3, std::make_pair(-180,180)},
  {4, std::make_pair(-120,120)},
  {5, std::make_pair(-120,120)}
};
std::map<uint8_t,float> ZERO_ANGLES = {
  {0,180},{1,260},{2,198},{3,90},{4,0},{5,135}
};

std::map<uint8_t, std::pair<float, float>> MIN_MAX_ANGLES;

bool threshold_crossed[SERVOS] = {false, false, false, false, false, false};

float level_angles[SERVOS];
float horizon_angles[SERVOS];

float torque_threshold_constants[SERVOS];
float torque_thresholds[SERVOS];

LiquidCrystal_I2C lcd(0x27,16,2);

float angle_offset[SERVOS] = {0, 0, 0, 0, 0, 0};

const std::vector<uint8_t> DXL_IDS = {0, 1, 2, 3, 4, 5};
// Note: Masking an id freezes its value.
//       Masking an id does not skip instances of disabling torque/led.
std::vector<bool> mask = {1, 1, 1, 1, 1, 1};
int counter;  // For filling in angles. Ignores when > SERVOS.

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);



//This namespace is required to use Control table item names.
using namespace ControlTableItem;

unsigned long prev_time;
unsigned long now;
#define PUMP_BUTTON_PIN A2
int pump_pin_state = 0;
bool pump_on = false;
bool lcd_pump_msg = false;
float degrees[6];

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  while (!USART);
  pump_on = false;
  USART.begin(115200);
  lcd.init();
  lcd.backlight();
  pinMode(PUMP_BUTTON_PIN,INPUT);
  //USB_SERIAL.println("SETUP BEGIN");

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Initialize our motors.
  for (auto DXL_ID : DXL_IDS) {
    // Get DYNAMIXEL information and mask unresponsive ids.
    if (!dxl.ping(DXL_ID)) mask[DXL_ID] = 0;
    if (!mask[DXL_ID]) continue;
  }
  
  reset_posture();
  
  // USB_SERIAL.println("SETUP END");
}

// this part is for preparing package for UART communication

struct arm_data packup(float* degrees){
  arm_data pack;
  pack.head = 0x0F;
  pack.reserved = 0x0C;
  counter = -1;
  for (auto DXL_ID : DXL_IDS){
    counter++;
    float percent;
    if (degrees[counter] >= 0){
      percent = degrees[counter]/MIN_MAX_ANGLES.at(DXL_ID).second;
      if (DXL_ID == 4) percent *= -1;
    } else {
      percent = -(degrees[counter]/MIN_MAX_ANGLES.at(DXL_ID).first);
      if (DXL_ID == 4) percent *= -1;
    }
    
    uint16_t channel = percent > 0 ? (uint16_t)(middle_channel + percent * (upper_channel-middle_channel))  : (uint16_t)(middle_channel + percent * (middle_channel-lower_channel));
   
    // Serial.print(DXL_ID);
    // Serial.print(" ");
    // Serial.print(percent*100);
    // Serial.print(" ");
    // Serial.print(channel);
    // Serial.print(" || ");
    pack.data[counter*2] = channel >> 8;
    pack.data[counter*2+1] = channel & 0xFF;
  }
  Serial.println();
  counter++;
  for (; counter < 16; counter++){
    pack.data[counter*2] = 0;
    pack.data[counter*2+1] = 0;
  }
  uint8_t bcc = 0;
  for (int i = 0; i < 32; i++){
    bcc ^= pack.data[i];
  }
  bcc^=pack.reserved;
  pack.tail = bcc;
  return pack;
}


int time_lasting = 0;
void loop() {
  
  // Read angles.
  counter = -1;
  now = millis();
  for (auto DXL_ID : DXL_IDS) {
    counter++;
    if (counter > SERVOS - 1) break;
    else if (!mask[DXL_ID]) continue;
      degrees[counter] = clamp(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - angle_offset[counter], MIN_MAX_ANGLES.at(counter).first, MIN_MAX_ANGLES.at(counter).second);
  }
  
  if (USART.availableForWrite()){
    arm_data pack = packup(degrees);

    pump_pin_state = digitalRead(PUMP_BUTTON_PIN);
    int time_start = now;
    if (pump_pin_state == HIGH){
      delay(300);
      pump_on = !pump_on;
      lcd_pump_msg = true;
    }
    if (pump_on){
      pack.data[13*2] = upper_channel >> 8;
      pack.data[13*2+1] = upper_channel & 0xFF;
    } else {
      pack.data[13*2] = lower_channel >> 8;
      pack.data[13*2+1] = lower_channel & 0xFF;
    }

    if (lcd_pump_msg){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Pump: ");
      if (pump_on){
        lcd.print("ON");
      } else {
        lcd.print("OFF");
      }
      lcd.setCursor(0,1);
      if (pump_on){
        lcd.print("Press for OFF...");
      } else {
        lcd.print("Press for ON...");
      }
      lcd_pump_msg = false;
    }

    /*
    for (int i = 0; i < 6; i++){
      Serial.print("[");
      Serial.print( dxl.getPresentPosition(i, UNIT_DEGREE));
      Serial.print(" ");
      Serial.print(degrees[i]);
      Serial.print(", ");
      uint16_t data = pack.data[i*2] << 8 + pack.data[i*2+1];
      Serial.print(data);
      
      Serial.print(" ");  
      Serial.print(MIN_MAX_ANGLES.at(i).first);
      Serial.print(" ");
      Serial.print(MIN_MAX_ANGLES.at(i).second);
      Serial.print("]");
  }
    Serial.println();
    */
    if ((now - prev_time)*0.001 <=15){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("GIPSY!!");
      lcd.setCursor(0,1);
      lcd.print("AT YOUR SERVICE!");
    } 
    if ((now - prev_time)*0.001 > 15 && (now - prev_time)*0.001 <= 16){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ILLINI_RM!");
      lcd.setCursor(0,1);
      lcd.print("Ready to Roll!");
    }
    USART.write((uint8_t*)&pack, sizeof(arm_data));
  }
  if ((dxl.getPresentPosition(5, UNIT_DEGREE) - angle_offset[5])>MIN_MAX_ANGLES.at(5).second||(dxl.getPresentPosition(5, UNIT_DEGREE) - angle_offset[5])<MIN_MAX_ANGLES.at(5).first){
    dxl.torqueOn(5);
  } else {
    dxl.torqueOff(5);
  }
  delay(5);
  
  }

  void reset_posture(){

    for (auto DXL_ID : DXL_IDS) {
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.torqueOn(DXL_ID);
  }
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  dxl.setGoalPosition(0,ZERO_ANGLES.at(0),UNIT_DEGREE);
  angle_offset[0] = ZERO_ANGLES.at(0);
  delay(100);
  dxl.setGoalPosition(3,ZERO_ANGLES.at(3),UNIT_DEGREE);
  angle_offset[3] = ZERO_ANGLES.at(3);
  delay(100);
  if (dxl.getPresentPosition(4,UNIT_DEGREE) > 180){
    dxl.setGoalPosition(4,360,UNIT_DEGREE);
    angle_offset[4] = 360;
  } else {
    dxl.setGoalPosition(4,0,UNIT_DEGREE);
    angle_offset[4] = 0;
  }
  dxl.setGoalPosition(5,ZERO_ANGLES.at(5),UNIT_DEGREE);
  angle_offset[5] = ZERO_ANGLES.at(5);
  
  delay(100);
  
  // dxl.setGoalCurrent(DXL_ID, MOVING_CURRENT);

  delay(1000);
  for (auto DXL_ID : DXL_IDS) {
    dxl.torqueOff(DXL_ID);
  }
  bool id_1_calibrated = false;
  bool id_2_calibrated = false;
  float id_1_start_angle = 272;
  float id_2_start_angle = 217;
  while (!id_1_calibrated || !id_2_calibrated){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Main_Arm: ");
    if (std::fabs(dxl.getPresentPosition(1,UNIT_DEGREE) - id_1_start_angle) < 3){
      lcd.setCursor(10,0);
      angle_offset[1] = ZERO_ANGLES.at(1);
      id_1_calibrated = true;
    }
    if (id_1_calibrated){
      lcd.setCursor(10,0);
      lcd.print("Set!");
    } else {
      lcd.setCursor(10,0);
      lcd.print("Lift!");
    }
    lcd.setCursor(0,1);
    lcd.print("Fore_Arm: ");
    if (std::fabs(dxl.getPresentPosition(2,UNIT_DEGREE) - id_2_start_angle) < 3){
      angle_offset[2] = ZERO_ANGLES.at(2);
      id_2_calibrated = true;
    }
    if (id_2_calibrated){
      lcd.setCursor(10,1);
      lcd.print("Set!");
    } else {
      lcd.setCursor(10,1);
      lcd.print("Lift!");
    }
    delay(100);
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibration Done!");
  delay(500);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("ILLINI_RM!");
  lcd.setCursor(0,1);
  lcd.print("Ready to Roll!");

  MIN_MAX_ANGLES = {
  {0, std::make_pair(MECHANICAL_ANGLES.at(0).first - angle_offset[0], MECHANICAL_ANGLES.at(0).second - angle_offset[0])},
  {1, std::make_pair(MECHANICAL_ANGLES.at(1).first - angle_offset[1], MECHANICAL_ANGLES.at(1).second - angle_offset[1])},
  {2, std::make_pair(MECHANICAL_ANGLES.at(2).first - angle_offset[2], MECHANICAL_ANGLES.at(2).second - angle_offset[2])},
  {3, std::make_pair(MECHANICAL_ANGLES.at(3).first, MECHANICAL_ANGLES.at(3).second)},
  {4, std::make_pair(MECHANICAL_ANGLES.at(4).first, MECHANICAL_ANGLES.at(4).second)},
  // ID 4 somehow has a different range.
  {5, std::make_pair(MECHANICAL_ANGLES.at(5).first, MECHANICAL_ANGLES.at(5).second)}
};
  counter = -1;

  for (auto DXL_ID : DXL_IDS) {
    counter++;
    if (counter > SERVOS - 1) break;
    else if (!mask[DXL_ID]) continue;
      degrees[counter] = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - angle_offset[counter];
  }
  prev_time = millis();
}
