#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>

/*
time clang++ -o main main.cpp
./main
*/
using namespace std;

// Global constant
const double g = -9.8;

// Function prototypes
void y_position(double *y, double *vy, double *ay, double *t, double thrust_time, double dt);
void x_position(double *x, double *vx, double *ax, double dt);
void run_simulation(double *x, double *y, double *vx, double *vy, double *ax, double *ay, double *t, double dt, double thrust_time, double sim_time, double display_interval, bool sim);
void get_inputs(double *x, double *y, double *vx, double *vy, double *ax, double *ay, double &thrust_time, double &sim_time, double &display_interval);

int main() {
    double arr[8] = {0};
    double *x = &arr[0];
    double *y = &arr[1];
    double *vx = &arr[2];
    double *vy = &arr[3];
    double *ax = &arr[4];
    double *ay = &arr[5];
    double *t = &arr[6];
    double sim_time, display_interval, thrust_time;
    bool sim = false;
    char choice;

    // Get inputs
    get_inputs(x, y, vx, vy, ax, ay, thrust_time, sim_time, display_interval);

    cout << "Simulate (y/n): ";
    cin >> choice;

    if (choice == 'y' || choice == 'Y') {
        sim = true;
    }

    // Time increment
    double dt = 0.001;

    // Run simulation
    run_simulation(x, y, vx, vy, ax, ay, t, dt, thrust_time, sim_time, display_interval, sim);

    return 0;
}

void y_position(double *y, double *vy, double *ay, double *t, double thrust_time, double dt) {
    if (*t <= thrust_time) {
        *y = *y + *vy * dt + 0.5 * (*ay + g) * pow(dt, 2);
        *vy = *vy + (*ay + g) * dt;  // Update velocity
    } else {
        *y = *y + *vy * dt + 0.5 * g * pow(dt, 2);  // Thrust is off
        *vy = *vy + g * dt;  // Only gravity affects velocity
    }
}

void x_position(double *x, double *vx, double *ax, double dt) {
    *x = *x + *vx * dt;
    *vx = *vx + *ax * dt;
}

void run_simulation(double *x, double *y, double *vx, double *vy, double *ax, double *ay, double *t, double dt, double thrust_time, double sim_time, double display_interval, bool sim) {
    double next_display_time = 0;  // Track when to display the next output
    double max_height = *y;

    if (sim) {
        cout << "Simulating..." << endl;
        cout << setw(10) << "Time" << setw(15) << "Pos (x)" << setw(15) << "Pos (y)"
             << setw(15) << "Vel (vx)" << setw(15) << "Vel (vy)" << setw(15) << "Angle (º)" << endl;
    }

    while (*y >= 0 && (sim_time == 0 || *t <= sim_time)) {
        *t += dt;  // Increment simulation time
        x_position(x, vx, ax, dt);  // Update x position
        y_position(y, vy, ay, t, thrust_time, dt);  // Update y position and velocity

        // Check if it's time to display output
        if (sim && *t >= next_display_time) {
            double angle = (vx != 0) ? atan(*vy / *vx) * 180 / M_PI : 90.0;  // Avoid division by zero
            cout << setw(10) << fixed << setprecision(3) << *t
                 << setw(15) << *x
                 << setw(15) << max(0.0, *y)  // Ensure y doesn't go negative
                 << setw(15) << *vx
                 << setw(15) << *vy
                 << setw(15) << angle
                 << endl;

            next_display_time += display_interval;  // Schedule next display
        }

        if (*y > max_height) {
            max_height = *y;
        }

        // Turn off thrust after the specified thrust time
        if (*t >= thrust_time) {
            *ay = 0;
            *ax = 0;
        }
    }

    if (sim) {
        cout << "Simulation complete." << endl;
    }
    cout << "Final position (x, y):\t(" << *x << ", " << max(0.0, *y) << ")" << endl;
    cout << "Final time (seconds):\t" << *t << endl;
    cout << "Max height (m):\t\t" << max_height << endl;
}

void get_inputs(double *x, double *y, double *vx, double *vy, double *ax, double *ay, double &thrust_time, double &sim_time, double &display_interval) {
    cout << "Initial position (x, y): ";
    cin >> *x >> *y;

    // Variables for velocity and angle which are only important for getting initial values
    double v, a, angle;
    cout << "Initial velocity (m/s): ";
    cin >> v;
    cout << "Acceleration (m/s^2): ";
    cin >> a;
    cout << "Thrust time (seconds): ";
    cin >> thrust_time;
    cout << "Initial angle (deg): ";
    cin >> angle;

    *vx = v * cos((angle * 3.14159) / 180);
    *vy = v * sin((angle * 3.14159) / 180);
    *ax = a * cos((angle * 3.14159) / 180);
    *ay = a * sin((angle * 3.14159) / 180);

    cout << "X Velocity (m/s): " << *vx << endl;
    cout << "Y Velocity (m/s): " << *vy << endl;
    cout << "X Acceleration (m/s^2): " << *ax << endl;
    cout << "Y Acceleration (m/s^2): " << *ay << endl;

    cout << "Simulation time (seconds): ";
    cin >> sim_time;

    cout << "Output display interval (seconds): ";
    cin >> display_interval;  // New input for display interval
    if (display_interval <= 0) {
        cout << "Invalid display interval. Using default value of 0.01 seconds." << endl;
        display_interval = 0.01;
    }
}