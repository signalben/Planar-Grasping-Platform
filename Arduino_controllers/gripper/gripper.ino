#include <AltSoftSerial.h>

bool enable = false; //control of arm and gripper disabled on startup
AltSoftSerial Serial1; //additionial serial port
int limit = 100; //max PWM value (0-255)
int sensor; //current potentiometer value (0-1023)
float Pg = 20, Ig = 0.025, Dg = 130;// PID gains
//Variables keep recieved target velocities, positional error, current velocities, and previous velocity error (needed to calculate change of velocity error)
float target = 0, PosE = 0, OldVelE = 0, Vel = 0, OldSensor;

//max, min sensor values allowed, prevents damage by attempting to reach outside the travel of the jaws
int maxreach = 855;
int minreach = 85;
float rolling_read;

//Returns analogue position using two separate wires and rolling average filter 
int filtered_read() {
  float new_read = (analogRead(A0) + analogRead(A1)) / 2;
  rolling_read = (0.9 * rolling_read) + (0.1 * new_read);
  return int(rolling_read);
}

//Sets PWM output, limited to prevent damage to gears, overheating of wires
void set_speed(int input) {
  int speed = input;
  if (speed > limit) {
    speed = limit;
  }
  if (speed < -limit) {
    speed = -limit;
  }
  if (speed < 0) {
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    speed = -speed;
    analogWrite(5, 0);
    analogWrite(6, speed);
  }
  else {
    digitalWrite(4, HIGH);
    digitalWrite(3, HIGH);
    analogWrite(6, 0);
    analogWrite(5, speed);
  }
}

//setup, starts rolling average read of jaw position sensor
void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  Serial.begin(57600);
  Serial1.begin(57600);
  set_speed(0);
  rolling_read = (analogRead(A0) + analogRead(A1)) / 2;
  sensor = filtered_read();
  Serial.println("Ready - Use &E to enable control");
}

//Produces target velocities based on positional error
float planner(int error) {

  if (error == 0) {
    Serial.println("&T"); //Gripper target reached
    return 0;
  }

  float setspeed = 7;

  if (abs(error) < 60) {
    setspeed = 1 + ((abs(error)) / 10);
  }
  if (error > 0) {
    return setspeed;
  }
  if (error < 0) {
    return -setspeed;
  }
  Serial.println("Gripper planner error");
  return 0;
}

//empties everything in serial buffer
void  clear_serial_buffer() {
  while (Serial.available() > 0) {
    char junk = Serial.read();
  }
}

//Determines type of gripper command
void read_gripper_msg() {
  char inByte = Serial.read();
  if (inByte == 'C') {
    control_gripper();
  }
  else {
    if (inByte == 'D') {
      Serial.println("Disabled control of arm and gripper");
      enable = false;
    }
    else if (inByte == 'L') {
      Serial.print("Gripper value offset: ");
      Serial.println(minreach);
    }
    else if (inByte == 'H') {
      Serial.print("Maximum gripper value: ");
      Serial.println(maxreach - minreach);
    }
    else if (inByte == 'S') {
      Serial.print("&S"); //Current gripper value 
      Serial.println(sensor - minreach);
    }
    else if (inByte == 'P') {
      Serial.println("Direct PWM");
      direct_pwm();
    }
    clear_serial_buffer();
  }
}

//Intrpret bytes as digits of integer
int get_int() {
  int uPos = 0;
  while (Serial.available() > 0 ) {
    delay(1);
    char inByte = Serial.read();
    int capnum = int(inByte) - 48; //acsi code offet to 0-9

    if ((capnum < 10) && (capnum > -1)) { //if digit is 0-90
      if (uPos == 0) { //first digit so far
        uPos = capnum;
      }
      else {
        uPos =  (uPos * 10) + capnum; //previous digits x10 + next digit
      }
    }
    else {
      break; //no more valid digits return int
    }
  }
  return uPos;
}

//Function closes gripper jaws using recieved PWM value 
void direct_pwm() {
  int pwm_val = get_int();
  set_speed(-pwm_val);
  sensor = filtered_read(); //current position
  int previous = sensor;
  int jammed = 0;
  int jamtime = 20;
  Serial.print("&S"); //Current gripper value 
  Serial.println(sensor - minreach);
  delay(100);

  //enter direct pwm control loop
  while (true) {
    sensor = filtered_read(); //current position

    if (sensor == previous) {
      jammed++;
    }

    if (jammed > jamtime) {
      if (sensor < minreach) {
        Serial.println("&F"); //closure failed
        set_speed(100);
        while (filtered_read() < minreach) {
          delay(1);
        }
        set_speed(0);
        break;
      }
      Serial.println("&C"); //closure sucessfull
      set_speed(0);
      break;
    }
    previous = sensor;
    delay(10);
  }
  delay(100);
  Serial.print("&S"); //Current gripper value 
  Serial.println(sensor - minreach);
}

//Function positions gripper jaws to recieved value using PID
void control_gripper() {

  int uPos = get_int();
  Serial.print("Recieved: ");
  Serial.println(uPos);

  //target position is user specified number above minimum
  int tPos = uPos + minreach;

  //prevent specification of dangerously high target
  if ((tPos > maxreach) || (tPos < minreach)) {
    Serial.print("&F Gripper target out of bounds: ");
    Serial.println(uPos);
    set_speed(0);
  }

  //target accepted
  else {
    Serial.print("&S"); //Current gripper value 
    Serial.println(sensor - minreach);
    delay(100);

    //enter main control loop
    while (true) {
      sensor = filtered_read(); //current position
      int error = (tPos - sensor); //positional error
      target = planner(error); //velocity target

      //break if target reached
      if (target == 0) {
        set_speed(0);
        break;
      }

      //emergancy disable motor if bounds exceeded
      if ((sensor > maxreach) || (sensor < minreach)) {
        Serial.print("Gripper out of bounds: ");
        Serial.println(sensor - minreach);
        while (1) {
          set_speed(0);
        }
      }
      
      //use PID control to set PWM speed
      Vel = sensor - OldSensor;
      float VelE = target - Vel; //proportional error
      PosE += VelE; //update integral/positional error
      PosE = constrain(PosE, -1024, 1024); //prevent too much integral windup
      float AccE = VelE - OldVelE; //derivative/acceleration error
      float output = ((Pg * VelE) + (Ig * PosE) + (Dg * AccE)); //Sum errors, weighted by their gains, scale down to 8 bit range
      set_speed(lround(output));
      delay(10);
      OldVelE = VelE; //Store for use in next call
      OldSensor = sensor;
    }
      delay(100);
      Serial.print("&S"); //Current gripper value 
      Serial.println(sensor - minreach);
  }
}

//functions forward messages between arm and pc
void pass_to_scara() {
  while (Serial.available() > 0) {
    delay(2);
    Serial1.write(Serial.read());
  }
}

void pass_from_scara() {
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
}

//function determines if message is for arm, gripper, enables communications, request for gripper position
void read_msg_from_pc() {
  char inByte = Serial.read();

  if (enable == true) {
    if (inByte == '#') {//CMD for scara
      pass_to_scara();
    }
    else  if (inByte == '&') {//CMD for gripper
      read_gripper_msg(); 
    }
  }

  else {
    if (inByte == '&') {
      char inByte = Serial.read();
      if (inByte == 'E') {
        Serial.println("Enabled control of arm and gripper");
        enable = true;
        clear_serial_buffer();
        Serial.print("&S"); //Current gripper value 
        Serial.println(sensor - minreach);
      }
    }
  }
}

//main loop - checks for serial input 
void loop() {
  sensor = filtered_read();
  if (Serial.available() > 0) {
    delay(1);
    read_msg_from_pc();
  }
  if (Serial1.available() > 0) {
    delay(1);
    pass_from_scara();
  }
  delay(1);
}

/* References-
 *  
 *  Code used for implimentation of PID control:
 *  https://github.com/signalben/Versatile-Arduino-Network/blob/main/libraries/van_dev_pid/van_dev_pid.cpp
 *  
 *  Library used for adding second serial port:
 *  https://www.arduino.cc/reference/en/libraries/altsoftserial/
 */
