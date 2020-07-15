#include <VarSpeedServo.h>

#define MESSAGE_RECEIVE_LENGTH 4 // length of message  PC -> Arduino
#define MESSAGE_SEND_LENGTH 3 // length of message  Arduino -> PC

// define Arduino pins
#define PIN_RYAW 8
#define PIN_LYAW 9
#define PIN_PITCH 7

#define LOWER_LIMIT 90-40
#define UPPER_LIMIT 90+40

VarSpeedServo ryaw_servo;
VarSpeedServo lyaw_servo;
VarSpeedServo pitch_servo;

byte input;

// set initial values
int lyaw=90;
int ryaw=90;
int pitch=90;

int bufferLength = 0;
byte serialBuffer[MESSAGE_RECEIVE_LENGTH]; // buffer that stores received serial data.

void setup() {
  // setup servos
  ryaw_servo.attach(PIN_RYAW);
  lyaw_servo.attach(PIN_LYAW);
  pitch_servo.attach(PIN_PITCH);

  //begin serial communication
  Serial.begin(9600, SERIAL_8N1);
}

void parse_received(byte buf[MESSAGE_RECEIVE_LENGTH]){
  //  analyze received data and saves it to variables. See arduino.py Python code for protocol.
  char tmp;

  Serial.print(buf[0]);
  //memcpy(&buf[0], &tmp, 1);

  lyaw = constrain( buf[0], LOWER_LIMIT, UPPER_LIMIT);
  //memcpy(&buf[1], &tmp, 1);
  ryaw = constrain( buf[1], LOWER_LIMIT, UPPER_LIMIT);
  //memcpy(&buf[2], &tmp, 1);
  pitch = constrain( buf[2], LOWER_LIMIT, UPPER_LIMIT);
}

/*
void serialEvent(){
  // this function is called when data is available

  byte serialBuffer[MESSAGE_RECEIVE_LENGTH];
  Serial.readBytes(serialBuffer, MESSAGE_RECEIVE_LENGTH);
  byte check_sum = 0;

  // add all received bytes to the serialBuffer
  while(Serial.available()){
    serialBuffer[bufferLength] = Serial.read();
    bufferLength ++;
    if (bufferLength>=30){
      //when buffer is overflowing
      memcpy(serialBuffer, &serialBuffer[1], 30-1);
      bufferLength = 29;
    }
  }

  Serial.println(serialBuffer[0],HEX);

  // find and parse messages in the buffer, and delete messages already read from buffer
  for(int i=0; i>MESSAGE_RECEIVE_LENGTH; i++){
    Serial.println(serialBuffer[0],HEX);

    if (serialBuffer[i]==0xAA && serialBuffer[i+1]==0xAA){
      // data is found
      Serial.println("OK?");
      parse_received(&serialBuffer[i]);
      // delete the already read message from the buffer
      memcpy(serialBuffer, &serialBuffer[i+MESSAGE_RECEIVE_LENGTH], bufferLength-i-MESSAGE_RECEIVE_LENGTH);
      bufferLength -= MESSAGE_RECEIVE_LENGTH;
    }
  }
}*/

void serialEvent(){
  //called when data is available
  byte buf[MESSAGE_RECEIVE_LENGTH];
  Serial.readBytes(buf, MESSAGE_RECEIVE_LENGTH);
  
  //checksum 
  byte check_sum = 0;

  for (int i=0; i<MESSAGE_RECEIVE_LENGTH-1; i++){
    check_sum += buf[i];
  }
  if (check_sum != buf[MESSAGE_RECEIVE_LENGTH-1]){
    // Ignore if there is error with check sum.
    return;
  }

  parse_received(buf);
}


void send_serial(){
  // send data to PC
  // Please use if you want to extend the program to send sensor data to PC, etc.
  // currently just sends dummy data (a single byte)
  
  // construct message to send
  byte send_msg[MESSAGE_SEND_LENGTH];

  send_msg[0] = 0xbb; send_msg[1]=0xbb;
  byte dummy_data = 0;
  memcpy(&send_msg[2], &dummy_data, 1);
  Serial.write(send_msg, MESSAGE_SEND_LENGTH);
}
void loop() {
  // put your main code here, to run repeatedly:
  ryaw_servo.write(ryaw, 255, false);
  lyaw_servo.write(lyaw, 255, false);
  pitch_servo.write(pitch, 255, false);
  //delay(15);
  
  //send_serial();
}
