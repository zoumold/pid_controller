#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include "controller.hpp"

// Simulate motor with imperfection
double simulate_motor(double measured, double control_sig) {
    // Simulate motor response with some delay + noise
    double noise = ((rand() % 1000) / 1000.0 - 0.5) * 0.1; // small noise [-0.05, 0.05]
    return measured + 0.8 * control_sig + noise;
}

int main() {
    std::srand(static_cast<unsigned int>(std::time(nullptr))); // Seed rand

    const double tol = 1e-5;
    double measured = 0.0;
    double control_sig = 0.0;
    double err = 100.0;
    int iter = 0;

    PIDController controller(1.0, 0.1, 0.02, 0.1); // kp, ki, kd, dt
    controller.SetOutputLimits(-10.0, 10.0);
    controller.SetIntegralLimits(-5.0, 5.0);

    // Initialize target positions
    std::vector<double> posMust;
    for (int i = 0; i < 100; ++i) {
        posMust.push_back(10.0 + i * 2.0);
    }

    std::cout << "Step\tTarget\tMeasured\tControl Output\tError\n";

    while (err >= tol && iter < 100) {
        double target = posMust[iter];

        control_sig = controller.compute_step(target, measured);
        measured = simulate_motor(measured, control_sig);
        err = std::abs(target - measured);

        std::cout << iter << "\t" << target << "\t" << measured << "\t\t" << control_sig << "\t" << err <<'\n';
        iter++;
    }

    return 0;
}
