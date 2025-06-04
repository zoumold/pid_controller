#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt);

    double compute_step(double desired_value, double measured_value);
    void reset();
    double clamp(double value, double min, double max);
    void SetIntegralLimits(double min, double max);
    void SetOutputLimits(double min, double max);

private:
    double kp;
    double ki;
    double kd;
    double dt;

    double prev_error;
    double integral;
    double min_integral, max_integral;
    double min_output, max_output;

};

#endif