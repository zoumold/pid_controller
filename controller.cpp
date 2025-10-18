#include "controller.hpp"

//Constructor
PIDController::PIDController(double kp, double ki, double kd, double dt)
    : kp(kp), ki(ki), kd(kd), dt(dt), prev_error(0.0), integral(0.0), prev_measured_value(0.0) {}

double PIDController::compute_step(double desired_value, double measured_value){
    double error, derivative, output;
    double P,I,D;

    //Calculate error integral and derivative
    error = desired_value - measured_value;
    P = kp*error;

    integral += error * dt;
    
    derivative = -(measured_value - prev_measured_value)/ dt;
    D = kd*derivative;

    //Clamp integral if needed
    integral = clamp(integral, min_integral, max_integral);
    I = ki * integral;

    //Calculate output
    output = P+I+D;

    //Clamp output if needed
    output = clamp(output, min_output, max_output);

    //Update error
    prev_error = error;
    prev_measured_value = measured_value;

    return output;
}
void PIDController::reset(){
    prev_error=integral=0.0;
    return;
}
double PIDController::clamp(double value, double min, double max){
    if(value < min){
        value = min;
    }
    else if(value > max){
        value = max;
    }
    return value;
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
