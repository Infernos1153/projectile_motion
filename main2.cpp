#include <iostream>
#include <iomanip>
#include <cmath>

/*
time clang++ -std=c++11 -o main2 main2.cpp
*/

// Global constant
const double GRAVITY = -9.8;

// Struct to hold the projectile state
struct ProjectileState {
    double x = 0.0;   // Position in x-direction
    double y = 0.0;   // Position in y-direction
    double vx = 0.0;  // Velocity in x-direction
    double vy = 0.0;  // Velocity in y-direction
    double ax = 0.0;  // Acceleration in x-direction
    double ay = 0.0;  // Acceleration in y-direction
    double t = 0.0;   // Time
};

// Function prototypes
void updateYPosition(ProjectileState& state, double thrust_time, double dt);
void updateXPosition(ProjectileState& state, double dt);
void runSimulation(ProjectileState& state, double dt, double thrust_time, double sim_time, double display_interval, bool display);
void getInputs(ProjectileState& state, double& thrust_time, double& sim_time, double& display_interval);
void print_header();

int main() {
    ProjectileState state;
    double thrust_time = 0.0, sim_time = 0.0, display_interval = 0.0;
    bool simulate = false;
    char choice;

    // Get inputs
    getInputs(state, thrust_time, sim_time, display_interval);

    std::cout << "Simulate (y/n): ";
    std::cin >> choice;
    if (choice == 'y' || choice == 'Y') {
        simulate = true;
    }

    // Time increment
    double dt = 0.001;

    // Run simulation
    runSimulation(state, dt, thrust_time, sim_time, display_interval, simulate);

    return 0;
}

// Function to update the y-position and y-velocity of the projectile
void updateYPosition(ProjectileState& state, double thrust_time, double dt) {
    if (state.t <= thrust_time) {
        // Thrust is on
        state.y += state.vy * dt + 0.5 * (state.ay + GRAVITY) * dt * dt;
        state.vy += (state.ay + GRAVITY) * dt;
    } else {
        // Thrust is off
        state.y += state.vy * dt + 0.5 * GRAVITY * dt * dt;
        state.vy += GRAVITY * dt;
    }
}

// Function to update the x-position and x-velocity of the projectile
void updateXPosition(ProjectileState& state, double dt) {
    state.x += state.vx * dt;
    state.vx += state.ax * dt;
}

// Function to run the simulation
void runSimulation(ProjectileState& state, double dt, double thrust_time, double sim_time, double display_interval, bool display) {
    double next_display_time = 0.0;
    double max_height = state.y;

    if (display) {
        std::cout << "Simulating..." << std::endl;
        print_header();
        // std::cout << std::setw(10) << "Time"
        //           << std::setw(15) << "Pos (x)"
        //           << std::setw(15) << "Pos (y)"
        //           << std::setw(15) << "Vel (vx)"
        //           << std::setw(15) << "Vel (vy)"
        //           << std::setw(15) << "Angle (º)"
        //           << std::endl;
    }

    while (state.y >= 0 && (sim_time == 0 || state.t <= sim_time)) {
        state.t += dt;
        updateXPosition(state, dt);
        updateYPosition(state, thrust_time, dt);

        if (display && state.t >= next_display_time) {
            double angle_deg = 0.0;
            if (state.vx != 0) {
                angle_deg = std::atan(state.vy / state.vx) * 180.0 / M_PI;
            } else {
                angle_deg = (state.vy >= 0) ? 90.0 : -90.0;
            }
            std::cout << std::setw(10) << std::fixed << std::setprecision(3) << state.t
                      << std::setw(15) << state.x
                      << std::setw(15) << std::max(0.0, state.y)
                      << std::setw(15) << state.vx
                      << std::setw(15) << state.vy
                      << std::setw(15) << angle_deg
                      << std::endl;
            next_display_time += display_interval;
        }

        if (state.y > max_height) {
            max_height = state.y;
        }

        // Turn off thrust after the specified thrust time
        if (state.t >= thrust_time) {
            state.ax = 0.0;
            state.ay = 0.0;
        }
    }

    if (display) {
        print_header();
        std::cout << "Simulation complete." << std::endl;
    }

    std::cout << "Final position (x, y):\t(" << state.x << ", " << std::max(0.0, state.y) << ")" << std::endl;
    std::cout << "Time of Flight (seconds):\t" << state.t << std::endl;
    std::cout << "Apex height (m):\t\t" << max_height << std::endl;
}

// Function to get inputs from the user
void getInputs(ProjectileState& state, double& thrust_time, double& sim_time, double& display_interval) {
    double initial_velocity = 0.0, mass = 0.0, acceleration = 0.0, angle_deg = 0.0;
    
    std::cout << "Initial position (x y): ";
    std::cin >> state.x >> state.y;

    std::cout << "Initial velocity (m/s): ";
    std::cin >> initial_velocity;
    
    std::cout << "Mass (kg): ";
    std::cin >> mass;
    
    std::cout << "Thrust (N): ";
    std::cin >> acceleration;
    
    std::cout << "Thrust time (seconds): ";
    std::cin >> thrust_time;
    
    std::cout << "Initial angle (degrees): ";
    std::cin >> angle_deg;
    
    std::cout << "Simulation time (seconds): ";
    std::cin >> sim_time;
    
    if (sim_time <= 0) {
        std::cout << "Invalid simulation time. Using default value of 50 seconds." << std::endl;
        sim_time = 50.0;
    }
    
    std::cout << "Output display interval (seconds): ";
    std::cin >> display_interval;
    
    if (display_interval <= 0) {
        std::cout << "Invalid display interval. Using default value of 0.01 seconds." << std::endl;
        display_interval = 0.1;
    }

    // Calc Initial Values
    double angle_rad = angle_deg * M_PI / 180.0;
    state.vx = initial_velocity * std::cos(angle_rad);
    state.vy = initial_velocity * std::sin(angle_rad);
    state.ax = (acceleration/mass) * std::cos(angle_rad);
    state.ay = (acceleration/mass) * std::sin(angle_rad);
    
    if ((int)angle_deg%90 == 0) {
        state.ax = 0.0;
        state.ay = acceleration/mass;
    }
    
    // Print Initial Values
    std::cout << "X Velocity (m/s): " << state.vx << std::endl;
    std::cout << "Y Velocity (m/s): " << state.vy << std::endl;
    std::cout << "X Acceleration (m/s^2): " << state.ax << std::endl;
    std::cout << "Y Acceleration (m/s^2): " << state.ay + GRAVITY << std::endl;
}

void print_header() {
    std::cout << std::setw(10) << "Time"
            << std::setw(15) << "Pos (x)"
            << std::setw(15) << "Pos (y)"
            << std::setw(15) << "Vel (vx)"
            << std::setw(15) << "Vel (vy)"
            << std::setw(15) << "Angle (deg)"
            << std::endl;
}