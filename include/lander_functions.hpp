#ifndef LANDER_FUNCTIONS_H
#define LANDER_FUNCTIONS_H

// #include "lander_structs.hpp"
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>
#include <Arduino.h>

// definitions
#define START_STATE 0
#define INITILISE_STATE 1
#define READY_STATE 2
#define ARMED_STATE 3
#define RUNNING_STATE 4

// struct definitions
// lander struct
struct lander{
    float s; // lander true position
    float v; // lander true velocity
    float a;
    long now; // lander sample time

    float s_tgt; // current target altitude

    float tof_s; // position according to tof
    long tof_time; // time of tof reading
    int tof_dt;
    // int tof_flag;

    sensors_event_t raw_IMU;
    long IMU_time; // time of IMU reading
    int IMU_dt;
    // int IMU_flag;

    float P[4] = {1.0, 0.0, 0.0, 1.0};

    float i_error; // PID step intergrated value

    int throttle; // current motor throttle, value 0 to 1000 inclusive

    struct lander* prev; // pointer to the previous set of lander values
};

struct PID_consts{
    float p;
    float i;
    float d;
};

// function declarations

// void recieve(char* dest);

float low_pass_filter(float measured, unsigned long dt, float prev_filtered);

// Lander Class object with all the methods called by the main
class Lander{
    
public:
    Lander();
    ~Lander();

    
    void update_lander();
    float calc_PID();
    void prep_for_test();
    void print_error(const char* description);

    float cal_tof(uint8_t numSamples);

    float target_height = 150;
    
    PID_consts k;

    Servo esc;
    
    uint8_t state;

    Adafruit_VL53L0X tof;
    Adafruit_MPU6050 IMU;

    lander cLander;
    
private:
    void handleIMU();
    void handleTOF();

    float Q[4] = {0.0005, 0, 0, 0.0005};
    float R = 0.05f;
    float C[2] = {1.0, 0.0};
    float P0[4] = {1.0, 0.0, 0.0, 1.0};
    float Max_throttle = 700;
    float Min_throttle = 0;
    float tOffset = 0;
    
    
    lander pLander;
    
};

#endif // LANDER_FUNCTIONS_