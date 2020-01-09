/* Sixense Buttons */

#define SIXENSE_BUTTON_BUMPER   (0x01<<7)
#define SIXENSE_BUTTON_JOYSTICK (0x01<<8)
#define SIXENSE_BUTTON_1        (0x01<<5)
#define SIXENSE_BUTTON_2        (0x01<<6)
#define SIXENSE_BUTTON_3        (0x01<<3)
#define SIXENSE_BUTTON_4        (0x01<<4)
#define SIXENSE_BUTTON_START    (0x01<<0)

typedef struct ArduinoData{
  float yaw;
  float pitch;
  float roll;
  long unsigned int buttons;
  float trigger;
  float joyx;
  float joyy;
}ArduinoData;

float maxV = 0.0f ;
float minV = 1024.0f;

// the setup function runs once when you press reset or power the board
void init_controller(void) {
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

// the loop function runs over and over again until power down or reset
void readButtons(ArduinoData &arduino_data) {
  arduino_data.buttons = 0;
  if ( digitalRead(7) == LOW)
     arduino_data.buttons = arduino_data.buttons | SIXENSE_BUTTON_BUMPER;
  if ( digitalRead(2) == LOW)
     arduino_data.buttons = arduino_data.buttons | SIXENSE_BUTTON_JOYSTICK;
  if ( digitalRead(8) == LOW)
     arduino_data.buttons = arduino_data.buttons | SIXENSE_BUTTON_4;
  if ( digitalRead(12) == LOW)
     arduino_data.buttons = arduino_data.buttons | SIXENSE_BUTTON_START;
  if ( digitalRead(4) == LOW)
    arduino_data.trigger = 1.0f;
  else
    arduino_data.trigger = 0.0f;

   arduino_data.joyx = mapf(analogRead(A1) , 0.0f , 1024.0f , -1.0f , 1.0f );
   arduino_data.joyy = mapf(analogRead(A2) , 0.0f , 1024.0f , -1.0f , 1.0f );
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
