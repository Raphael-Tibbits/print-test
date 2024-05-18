
#include "lander_functions.hpp"

// #include "Adafruit_Sensor.h"

// Function Definitions

// Constructor
Lander::Lander() {
  this->cLander.prev = &(this->pLander);
  this->state = START_STATE;
}

// Destructor
Lander::~Lander() {
    // Cleanup code here if needed
}

float low_pass_filter(float measured,  unsigned long dt,
                     float prev_filtered)
{
  // low_pass_filter
  // // first order low pass filter function
  return (prev_filtered + (measured-prev_filtered));

}

// void recieve(char* dest){
//   if(Serial){ 
//     Serial.flush();

//     memccpy(dest, Serial.readString().c_str(), 64, sizeof(char));
    
//     delayMicroseconds(1);
//     // String response = "R " + message;
//     Serial.print("R ");
//     Serial.flush();
//     Serial.println(dest);

//   }
  
// }

void Lander::print_error(const char* description){
  if(Serial){
    Serial.print("Error ");
    Serial.print(this->state);
    Serial.print(" ");
    Serial.print(description);
    Serial.print("\n");
  }
}


float Lander::calc_PID(){


  float altErr = this->cLander.s_tgt - this->cLander.s;

  long unsigned dt = (this->cLander.now - this->cLander.prev->now);
  // Serial.println(dt);

  float P = altErr;
  float I = this->cLander.prev->i_error + (P*dt)/1000000.0;
  float D = this->cLander.v * (-1.0f);

  if(I > Max_throttle/k.i){
    I =  this->cLander.prev->i_error;
  }
  if(I < Min_throttle/k.i){
    I =  this->cLander.prev->i_error;
  }

  this->cLander.i_error = I;

  float actuator = (k.p * P) + (k.i * I) + (k.d * D);

  if(actuator > Max_throttle){
    actuator = Max_throttle;
  }
  if(actuator < Min_throttle){
    actuator = Min_throttle;
  }
  this->cLander.throttle = actuator;
  return actuator;
}


void Lander::update_lander(){
  // copy current values to previous
  this->pLander = this->cLander;

  // this->cLander.IMU_time = micros();
  // handleIMU();
  
  // this->cLander.a = this->cLander.raw_IMU.acceleration.y;
  // this->cLander.IMU_dt = this->cLander.IMU_time - this->cLander.prev->IMU_time;
  
  // predict_step(this->cLander);

  
  this->cLander.tof_time = micros();
  handleTOF();

  this->cLander.tof_dt = this->cLander.tof_time - this->cLander.prev->tof_time;
  this->cLander.s = low_pass_filter(this->cLander.tof_s, this->cLander.tof_dt, this->cLander.prev->s);
  // update_step(this->cLander);
  
  this->cLander.now = this->cLander.tof_time;

  this->cLander.v = (this->cLander.s - this->cLander.prev->s) * 1000 / this->cLander.tof_dt;


}

// // returns the IMU reading in the z direction
// void Lander::handleIMU(){
//   // deals with IMU operations each iteration
//   sensors_event_t g, temp;
//   // average 16 IMU readings
  
//   IMU.getEvent(&(this->cLander.raw_IMU), &g, &temp);
//   // *aout = a.acceleration;
  
// }

float Lander::cal_tof(uint8_t numSamples){
  // calculate TOF offset (height at the bottom of the rail)
    float tempoffset = 0.0;
    for(uint8_t i = 0; i<numSamples; i++){
      handleTOF();
      tempoffset += this->cLander.tof_s;
      // Serial.println(tempoffset);
    }

    this->tOffset = tempoffset/numSamples;
    return this->tOffset;
}

void Lander::handleTOF(){
  VL53L0X_RangingMeasurementData_t measure;
  
  // Serial.print("Reading a measurement... ");
  this->tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  float alt;

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    alt = measure.RangeMilliMeter;
  } else {
    Serial.print("Error 1 failed to read TOF value");
  }

  this->cLander.tof_s = alt - this->tOffset;
}

// void Lander::prep_for_test(){

//   this->cLander.s = 0; // lander true position
//   this->cLander.v = 0; // lander true velocity
//   this->cLander.a = 0; // lander true acceleration
//   this->cLander.now = 0; // lander sample time

//   this->cLander.s_tgt = target_height; // current target altitude

//   this->cLander.tof_s = 0; // position according to tof
//   this->cLander.tof_time = 0; // time of tof reading
//   // this->cLander.tof_flag = 0;

//   // this->cLander.IMU_time = 0; // time of IMU reading
//   // this->cLander.IMU_flag = 0;

//   this->cLander.i_error = 0; // PID step intergrated value

//   this->cLander.throttle = 0; // current motor throttle, value 0 to 1000 inclusive
//   // this->cLander.prev->i_error = 0; // initial I error value
//   Serial.print("initial I:");
//   Serial.println(this->cLander.i_error);

// }
