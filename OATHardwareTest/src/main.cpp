// Standard includes
#include <Arduino.h>
#include <Wire.h>

// Include the AccelStepper Library
#include <AccelStepper.h>   // http://www.airspayce.com/mikem/arduino/AccelStepper/index.html

// Include the display Library
#include <U8x8lib.h>        // https://github.com/olikraus/u8g2

// Include the accelerometer Library
#include <MPU6050_light.h>  // https://github.com/rfetick/MPU6050_light

// Include the accelerometer Library
#include <TinyGPS++.h>      // https://github.com/mikalhart/TinyGPSPlus

// Serial port usage
#define USB_SERIAL_PORT Serial
#define GPS_SERIAL_PORT Serial2

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper RAStepper(AccelStepper::HALF4WIRE, 13, 14, 12, 27);
AccelStepper DECStepper(AccelStepper::HALF4WIRE, 26, 33, 25, 32);
int32_t const RAMaxSteps(25000);    // steps
int32_t const DECMaxSteps(25000);
int32_t const RAMaxSpeed(1000);     // steps per second
int32_t const DECMaxSpeed(500);
int32_t const RAAcceleration(100);  // steps per second per second
int32_t const DECAcceleration(50);

// Display parameters
U8X8_SSD1306_128X32_UNIVISION_HW_I2C display;

// Keypad parameters
uint8_t const LCD_KEY_SENSE_X_PIN(34);
uint8_t const LCD_KEY_SENSE_Y_PIN(39);
uint8_t const LCD_KEY_SENSE_PUSH_PIN(36);
uint8_t getKey();

// Gyro parameters
MPU6050 mpu(Wire);

// GPS engine
TinyGPSPlus gps;

// Task control
TaskHandle_t movementTaskHandle(0);
TaskHandle_t userInterfaceTaskHandle(0);
void movementTask(void * pvParameters);
void userInterfaceTask(void * pvParameters);
void runMotorControl();
void runUserInterface();

// Other vars
enum Mode { STOP_DISABLED, MOVE_MANUAL, MOVE_PATTERN };
static Mode currentMode(STOP_DISABLED); // Current mode
static uint8_t scannedKey(0);           // The last-pressed key
static char buffer[64];                 // Text formatting

void setup() {

  // Initialise the serial port
  USB_SERIAL_PORT.begin(115200);
  USB_SERIAL_PORT.println();
  USB_SERIAL_PORT.println();
  USB_SERIAL_PORT.println("OAT Hardware Test.");

	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	RAStepper.setMaxSpeed(RAMaxSpeed);
	RAStepper.setAcceleration(RAAcceleration);

	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	DECStepper.setMaxSpeed(DECMaxSpeed);
	DECStepper.setAcceleration(DECAcceleration);

  // Initialize I2C
  Wire.begin();

  // Initialize gyro
  mpu.begin();
  mpu.calcGyroOffsets();
  if ((mpu.readData(0x75) & 0x7E) == 0x68)  // WHO_AM_I register
    USB_SERIAL_PORT.println("Found MPU6050.");
  else
    USB_SERIAL_PORT.println("ERROR: Tilt sensor not detected!");

  // Initialize GPS
  GPS_SERIAL_PORT.begin(9600, SERIAL_8N1);

  // Initialise display
  display.begin();
  display.setPowerSave(0);
  display.setContrast(255);
  display.clear();
  //display.setFont(u8x8_font_amstrad_cpc_extended_f);  // Works 2x16 ugly
  display.setFont(u8x8_font_7x14_1x2_f);              // Works 2x16 nice
  display.drawString(0,0,"* Hello OAT HW *");
  sprintf(buffer, "Found display: %d rows, %d columns.", display.getRows(), display.getCols());
  USB_SERIAL_PORT.println(buffer);

  display.setFont(u8x8_font_open_iconic_arrow_1x1); 
  for (int i=0; i<16; i++) {
    display.draw1x2Glyph(i,2,64+i);
  }

  // Initialize keypad
  pinMode(LCD_KEY_SENSE_X_PIN, INPUT);
  pinMode(LCD_KEY_SENSE_Y_PIN, INPUT);
  pinMode(LCD_KEY_SENSE_PUSH_PIN, INPUT);

  // Start the pattern
  // RAStepper.moveTo(RAMaxSteps);
  // DECStepper.moveTo(DECMaxSteps);
  // currentMode = MOVE_PATTERN;
  currentMode = STOP_DISABLED;

  // Start tasks
  // Note that loop() is at priority 1 by default
  xTaskCreatePinnedToCore(movementTask, "Motor", 8192, NULL, 10, &movementTaskHandle, 0);
  xTaskCreatePinnedToCore(userInterfaceTask, "UI", 8192, NULL, 5, &userInterfaceTaskHandle, 1);
}

void loop() {
  // Nothing to do
  vTaskDelay(1000);  // 1000 ms
}

//==============================================================================

void movementTask( void * pvParameters ) {
  while(true) {
    runMotorControl();
    vTaskDelay(1);    // 1 ms
  };
}

void runMotorControl() {
  //----------------------------------------------------------------------------
  // Service the motor motion task

  switch(currentMode) {
  case STOP_DISABLED:
    // Do nothing
  break;
  case MOVE_PATTERN:
    // Change direction once the motor reaches target position
    if (RAStepper.distanceToGo() == 0) 
      RAStepper.moveTo(-RAStepper.currentPosition());
  	RAStepper.run();
    if (DECStepper.distanceToGo() == 0) 
      DECStepper.moveTo(-DECStepper.currentPosition());
  	DECStepper.run();
  break;
  case MOVE_MANUAL:
    // Manual seek mode
    RAStepper.runSpeed();
    DECStepper.runSpeed();
  break;
  }
}

//==============================================================================

void userInterfaceTask( void * pvParameters ) {
  while(true) {
    runUserInterface();
    vTaskDelay(100);   // 100 ms
  };
}

void runUserInterface() {

  //----------------------------------------------------------------------------
  // Check for commands from the serial port/keypad
  //  A     Decrement the RA axis (clockwise from front)
  //  D     Increment the RA axis (clockwise from front)
  //  S     Decrement the DEC axis (decrease elevation)
  //  W     Increment the DEC axis (increment elevation)
  //  Q     Stop all movement 
  //  E     Toggle moving in pattern through full movement
  //  X     Disable motors

  // For serial port commands action will continue to run until a new key is pressed
  uint8_t keys(0); 
  if (USB_SERIAL_PORT.available() > 0)
    keys = tolower(USB_SERIAL_PORT.read());
  else {
    keys = scannedKey;
    scannedKey = 0;
  }

  // USB_SERIAL_PORT.print(F("Serial: 0x"));
  // USB_SERIAL_PORT.println(keys, HEX);

  if (keys) {
    switch(keys) {
      case 's':   // Move down
        DECStepper.setSpeed(DECMaxSpeed);
        currentMode = MOVE_MANUAL;
      break;
      case 'w':   // Move up
        DECStepper.setSpeed(-DECMaxSpeed);
        currentMode = MOVE_MANUAL;
      break;
      case 'a':   // Move left
        RAStepper.setSpeed(-RAMaxSpeed);
        currentMode = MOVE_MANUAL;
      break;
      case 'd':   // Move right
        RAStepper.setSpeed(RAMaxSpeed);
        currentMode = MOVE_MANUAL;
      break;
      case 'e':   // Toggle moving through limited pattern
        if (currentMode == MOVE_PATTERN) {
          // Stop moving
          RAStepper.setSpeed(0);
          DECStepper.setSpeed(0);
          RAStepper.stop();
          DECStepper.stop();
          currentMode = MOVE_MANUAL;
        } else {
          // Start moving
          RAStepper.setCurrentPosition(0);
          RAStepper.moveTo(RAMaxSteps);
          DECStepper.setCurrentPosition(0);
          DECStepper.moveTo(DECMaxSteps);
          currentMode = MOVE_PATTERN;
        }
      break;
      case 'x':   // Disable motors
        RAStepper.disableOutputs();
        DECStepper.disableOutputs();
        currentMode = STOP_DISABLED;
      break;
      case 'q':   // Stop moving
      default:
        RAStepper.setSpeed(0);
        DECStepper.setSpeed(0);
        RAStepper.stop();
        DECStepper.stop();
        currentMode = MOVE_MANUAL;
      break;
    }
  }

  //----------------------------------------------------------------------------
  // Read new data from the GPS receiver

  static uint32_t gpsTimer(0);

  while (GPS_SERIAL_PORT.available() > 0) {
    gps.encode(GPS_SERIAL_PORT.read());
  }

  if((millis()-gpsTimer) > 1000) { 
    if (gps.location.isUpdated()) {
      sprintf(buffer, "GPS: time=%u, lat:%.3f, lng=%.3f, height=%.1f", gps.time.value(),gps.location.lat(),gps.location.lng(),gps.altitude.meters());
      USB_SERIAL_PORT.println(buffer);
    }
    sprintf(buffer, "GPS: Passed=%d, Failed=%d, Fixes=%d", gps.passedChecksum(), gps.failedChecksum(),gps.sentencesWithFix());
    USB_SERIAL_PORT.println(buffer);
    gpsTimer = millis();  
  }

  //----------------------------------------------------------------------------
  // Read new data from tilt sensor

  static uint32_t mpuTimer(0);

  mpu.update();
  
  if((millis()-mpuTimer) > 1000) { 
    sprintf(buffer, "Tilt: X=%.3f, Y=%.3f, Z=%.3f, T=%.1f.", mpu.getAngleX(),mpu.getAngleY(),mpu.getAngleZ(),mpu.getTemp());
    USB_SERIAL_PORT.println(buffer);
    mpuTimer = millis();  
  }

  //----------------------------------------------------------------------------
  // Read the keypad

  static uint32_t keypadTimer(0);
  static uint8_t lastKey(0);

  if((millis()-keypadTimer) > 100) { 
    uint8_t key(getKey());
    if (key != lastKey) {
      sprintf(buffer, "Key: '%c'.", key);
      USB_SERIAL_PORT.println(buffer);
      lastKey = key;    // Debounce
      scannedKey = key; // Forward for processing
    }
    keypadTimer = millis();  
  }

  //----------------------------------------------------------------------------
  // Display the current position of the tracker

  static uint32_t displayTimer(0);

  if((millis()-displayTimer) > 1000) { 
    display.setCursor(0,2);
    display.printf("R%+06ld D%+06ld",RAStepper.currentPosition(),DECStepper.currentPosition());
    displayTimer = millis();  
  }

  //----------------------------------------------------------------------------
}

//==============================================================================

/**
 * Return the current key pressed (1-9), or 0 if no key currently pressed.
 * This is a hacked 3x3 matrix keypad. Probably not reusable.
*/
uint8_t getKey() {
  uint16_t x(analogRead(LCD_KEY_SENSE_X_PIN));
  uint16_t y(analogRead(LCD_KEY_SENSE_Y_PIN));
  bool push(digitalRead(LCD_KEY_SENSE_PUSH_PIN) == LOW);  // Active low

  // Assumes analogReadResolution(12) (the default)
  int16_t const MIDSCALE = 4096 / 2;
  int16_t const DEADBAND = 500;

  byte keyState = 0;
  if (x > (MIDSCALE + DEADBAND)) keyState = 'd';
  if (x < (MIDSCALE - DEADBAND)) keyState = 'a';
  if (y > (MIDSCALE + DEADBAND)) keyState = 's';  // Y appears reversed
  if (y < (MIDSCALE - DEADBAND)) keyState = 'w';
  if (push) keyState = 'e';

  //sprintf(buffer,"Keyscan: X=%4d, Y=%4d, P=%4d -> key='%c'",x,y,push,keyState ? keyState : ' ');
  //USB_SERIAL_PORT.println(buffer);

  return keyState;
}

// Obsolete code for TM1637 display
#if 0
void displayValue(int32_t value) {

  TM1637 display(2, 15);

  // Initialize display
  display.begin();
  display.displayOff();
  display.clear();
  display.setBrightness(3);
  display.displayOn();

  static uint8_t const digitSpace = 0b00000000;
  static uint8_t const digitNegative = 0b01000000;
  static uint8_t const digitPoint = 0b10000000;
  static uint8_t const digitMap[16] = {
      0b00111111, // 0
      0b00000110, // 1
      0b01011011, // 2
      0b01001111, // 3
      0b01100110, // 4
      0b01101101, // 5
      0b01111101, // 6
      0b00000111, // 7
      0b01111111, // 8
      0b01101111, // 9
      0b01110111, // A
      0b01111100, // B
      0b00111001, // C
      0b01011110, // D
      0b01111001, // E
      0b01110001  // F
  };

  // Print negative sign in first position if required
  if (value < 0) {
    display.writeData(0, digitNegative);
    value = -value;
  }
  // Print right-justified positive value of up to 5 digits
  for (uint8_t i=0; i<5; i++) {   // Skip left-most display position
    if ((i > 0) && (value == 0))
      // Replace leading zeros with spaces
      display.writeData(5-i, digitSpace);    
    else {
      // Print other digits
      uint8_t digit = value % 10;
      display.writeData(5-i, digitMap[digit]);
    }
    value /= 10;
  }
}
#endif
