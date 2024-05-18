#include <Arduino.h>
#include "lander_functions.hpp"
#include <avr/wdt.h>

void software_reset(){
  Serial.println("retrying in 5s");
  Serial.end();
  delay(5000);
  wdt_enable(WDTO_15MS);
  while(1);
}

void recieve(char* buffer){
  buffer[Serial.readBytesUntil('\0', buffer, 32)] = '\0';
  
  Serial.print("R ");
  Serial.println(buffer);
}

Lander L;

char response[32];

void setup() {
  // pinMode(3, OUTPUT);
  // digitalWrite(3, 0);
  L.state = START_STATE;
  Wire.begin();
  
  Serial.begin(112500);

  // String message = ;
  Serial.println("Serial Connected");

  recieve(response);


  // while(!Serial.available());
  // delayMicroseconds(1280);

  
  recieve(response);

  
  if(!strcmp(response,"working")){
    Serial.println("starting Init");
    L.state = INITILISE_STATE;
  }else{
    L.print_error("incorrect response");
    software_reset();
  }
  bool init_status;//, tof_status;//, imu_status;
  init_status = L.tof.begin();
  L.cal_tof(8);
  // Serial.print("TOF status:");
  // Serial.println(tof_status);

  // imu_status = L.IMU.begin();
  // Serial.print("IMU status:");
  // Serial.println(imu_status);

  // init_status = tof_status;// && imu_status;

  Serial.print("initilisation status:");
  Serial.println(init_status);

  if(init_status == true){
    Serial.println("Ready for commands");
    L.state = READY_STATE;
  }

}

void loop() {
  // Serial.println("looing");

  if(Serial.available()){
    recieve(response);
  }

  if(strcmp(response, "calibrate ESC") && L.state == READY_STATE){
    Serial.println("Calibrating ESC");
    L.esc.attach(10);
    L.esc.writeMicroseconds(2000);
    delay(1000);
    L.esc.writeMicroseconds(1000);
    delay(1000);
    L.state = ARMED_STATE;

  }else if((strcmp(response, "run") && L.state == ARMED_STATE) || (response[0] == '\0' && L.state == RUNNING_STATE)){
    L.update_lander();
    L.calc_PID();
    L.esc.writeMicroseconds(L.cLander.throttle + 1000);
    Serial.println("sending Telemetry");
    // Serial.write((char*)&(L.cLander), sizeof(lander));

  }else{
    if(L.esc.attached()){
      L.esc.writeMicroseconds(1000);
      L.esc.detach();
    }
    L.print_error("no command");
    L.state = READY_STATE;
  }
  

  // put your main code here, to run repeatedly:
}
