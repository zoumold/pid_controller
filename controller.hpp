#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

class PIDController {
public:
    PIDController(double kp, double ki, double kd);

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
    double prev_measured_value;
    double integral;
    double min_integral=-1.0, max_integral=1.0;
    double min_output=1.0, max_output=1.0;

    bool is_initialized;

};

#endif