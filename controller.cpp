#include "controller.hpp"
#include <algorithm>

//Constructor
PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), prev_error(0.0), integral(0.0), prev_measured_value(0.0), is_initialized(false) {}

double PIDController::compute_step(double desired_value, double measured_value){
    double error, derivative, output;
    double P,I = 0.0, D=0.0;

    if(!is_initialized){
        prev_measured_value = measured_value;
        is_initialized = true;
    }

    //Calculate error integral and derivative
    error = desired_value - measured_value;
    P = kp*error;

    
    if(dt > 0){
        //Calculate integral
        integral += error *dt;
        integral = std::clamp(integral, min_integral, max_integral);
        I = ki * integral;

        //calculate derrivative
        derivative = -(measured_value - prev_measured_value) / dt;
        D = kd*derivative;
    }

    //Calculate output
    output = P+I+D;

    //Clamp output if needed
    output = std::clamp(output, min_output, max_output);

    //Update prev_measured value 
    prev_measured_value = measured_value;

    return output;
}
void PIDController::reset(){
    integral = 0.0;
    is_initialized = false;
    return;
}

void PIDController::SetOutputLimits(double min, double max){
    min_output = min;
    max_output = max;
    return;
}
void PIDController::SetIntegralLimits(double min, double max){
    min_integral = min;
    max_integral = max;
    return;
}
