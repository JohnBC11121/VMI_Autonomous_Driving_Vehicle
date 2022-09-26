
#include <SPI.h>
#include <mcp2515.h>

//***define the connections of the board*********************************************************

#define brake_switch A4 //the input for the brake switch
#define brake_sensor A2 // the input for the brake sensor
#define brake_enable 8 // the brake switch output
#define brake_magnitude 6 // the brake senosr pwm output

#define throttle_switch A5 // the input for the throttle switch
#define throttle_sensor A3 // the input for the throttle sensor
#define throttle_enable 4 // the ouput for the throttle switch
#define throttle_magnitude 5 // the ouput for the throttle sensor

#define keyswitch_one A0 // the input for the keyswitch
#define keyswitch_two A1 // the output for the keyswitch

#define myID 2

//****variables used in the code**********************************************************

struct can_frame canIn;
struct can_frame canOut;
MCP2515 mcp2515(10);

struct {
  int id;
  int dlc;
  int data[7];
} command;

struct {
  bool _switch = false;
  int sensor = 0;
} throttle;

struct {
  bool _switch = false;
  int sensor = 0;
} brake;

struct {
  int one = 0;
  int two = 0;
} keyswitch;


//*******set up inputs and outputs*********************************************************************

void setup() {
  Serial.begin(115200); // used for debugging

  // setting up all of the outputs
  pinMode(brake_enable, OUTPUT);// brake switch
  pinMode(brake_magnitude, OUTPUT);// brake pwm
  pinMode(throttle_enable, OUTPUT);// throttle switch
  pinMode(throttle_magnitude, OUTPUT);// throttle pwm

  // setting up all of the inputs
  pinMode(brake_switch, INPUT);
  pinMode(brake_sensor, INPUT);
  pinMode(throttle_switch, INPUT);
  pinMode(throttle_sensor, INPUT);
  pinMode(keyswitch_one, INPUT);
  pinMode(keyswitch_two, INPUT);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  // set up the interrupt for the CAN 
  attachInterrupt(digitalPinToInterrupt(2), MCP2515_ISR, FALLING);// CANbus Interrupt
}

//****Main Loop***************************************************************************

void loop() {
  test(1, 1, 0, 128); // overide the system and output 2.5V to the throttle
}

///*****Test loop*************************************************************************
void test(int Override, int control, int Bmagnitude, int Tmagnitude){
  command.data[0] = 0;
  command.data[1] = Override;
  command.data[2] = !control;
  command.data[3] = Bmagnitude;
  command.data[4] = control;
  command.data[5] = Tmagnitude;
  command.data[6] = 0;
  command.data[7] = 0;

  _update();
  _read();
  debug();
  delay(500);
}

//****read incoming CAN messages**********************************************************

void MCP2515_ISR() {
  if (mcp2515.readMessage(&canIn) == MCP2515::ERROR_OK) {
    if (canIn.data[0] == myID) {
      for (int i = 0; i < canIn.can_dlc; i++){
        command.data[i] = canIn.data[i];
        // data[0] = Target Device ID
        // data[1] = override
        // data[2] = Brake enable control bit
        // data[3] = brake magnitude
        // data[4] = Throttle enable control bit
        // data[5] = Throttle magnitude
      }
    }
  }
}

//****send feedback info to the executive proscessor***************************************

void CANfeedback() {
  canOut.can_id  = myID;
  canOut.can_dlc = 8;
  canOut.data[0] = 0;
  canOut.data[1] = 0;
  canOut.data[2] = 0;
  canOut.data[3] = 0;
  canOut.data[4] = 0;
  canOut.data[5] = 0;
  canOut.data[6] = 0;
  canOut.data[7] = 0;

  mcp2515.sendMessage(&canOut);
}

//****update all of the outputs based on the most recent CAN message************************

void _update(){
digitalWrite(brake_switch, command.data[2]);// brake switch
analogWrite(brake_magnitude, command.data[3]);// brake pwm
digitalWrite(throttle_switch, command.data[4]);// throttle switch
analogWrite(throttle_magnitude, command.data[5]);// throttle pwm
}

//****Read all of the inputs****************************************************************

void _read() {
  brake._switch = digitalRead(brake_switch);
  brake.sensor = analogRead(brake_sensor);

  throttle._switch = digitalRead(throttle_switch);
  throttle.sensor = analogRead(throttle_sensor);

  keyswitch.one = digitalRead(keyswitch_one);
  keyswitch.two = digitalRead(keyswitch_two);
}

//****serial print debug info**************************************************************

void debug() {
  Serial.print("CAN ID: ");
  Serial.println(canIn.can_id, BIN);

  Serial.print("CAN Data: ");
  Serial.print(command.data[0], BIN);
  Serial.print(" ");
  Serial.print(command.data[1], BIN);
  Serial.print(" ");
  Serial.print(command.data[2], BIN);
  Serial.print(" ");
  Serial.print(command.data[3], BIN);
  Serial.print(" ");
  Serial.print(command.data[4], BIN);
  Serial.print(" ");
  Serial.print(command.data[5], BIN);
  Serial.print(" ");
  Serial.print(command.data[6], BIN);
  Serial.print(" ");
  Serial.println(command.data[7], BIN);
  Serial.println("");

  Serial.print("Target ID: ");
  Serial.println(command.data[0], BIN);
  Serial.println("");

  Serial.print("Override: ");
  Serial.println(command.data[1], BIN);
  Serial.println("");

  Serial.print("Brake Switch: ");
  Serial.print(brake._switch, BIN);
  Serial.print("  ");
  Serial.println(command.data[2], BIN);

  Serial.print("Brake Sensor: ");
  Serial.print(brake.sensor);
  Serial.print("  ");
  Serial.println(command.data[3]);
  Serial.println("");

  Serial.print("Throttle Switch: ");
  Serial.print(throttle._switch, BIN);
  Serial.print("  ");
  Serial.println(command.data[4], BIN);

  Serial.print("Throttle Sensor: ");
  Serial.print(throttle.sensor);
  Serial.print("  ");
  Serial.println(command.data[5]);
  Serial.println("");

  Serial.print("Keyswitch 1: ");
  Serial.println(throttle._switch, BIN);

  Serial.print("Keyswitch 2: ");
  Serial.println(throttle.sensor, BIN);
  Serial.println("");
  Serial.println("");
  Serial.println("");
}
