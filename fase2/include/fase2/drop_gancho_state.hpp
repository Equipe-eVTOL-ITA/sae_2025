#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <chrono>

class DropGanchoState : public fsm::State {
public:
    DropGanchoState() : fsm::State(), claw_opened_(false), gpio_initialized_(false), claw_opening_started_(false) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: DROP GANCHO");

        start_time = std::chrono::steady_clock::now();
        claw_opened_ = false;
        gpio_initialized_ = false;
        claw_opening_started_ = false;
        claw_open_start_time = std::chrono::steady_clock::now();
        
        // Initialize GPIO for claw control
        initializeGPIO();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        // Always send velocity commands to maintain offboard mode
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        // Start opening claw after 1 second delay (to ensure drone is stable)
        if (elapsed.count() >= 1000 && !claw_opening_started_ && gpio_initialized_) {
            startOpeningClaw();
            claw_opening_started_ = true;
            claw_open_start_time = current_time;
        }
        
        // Check if claw opening is complete (after 3 seconds of opening)
        if (claw_opening_started_ && !claw_opened_) {
            auto claw_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - claw_open_start_time);
            if (claw_elapsed.count() >= 3000) {  // 3 seconds for claw to fully open
                finishOpeningClaw();
                claw_opened_ = true;
            }
        }
        
        // Exit after claw has been opened and additional delay
        if (elapsed.count() > 6000) {  // 6 seconds total
            return "GANCHO_DROPPED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting DROP GANCHO state - Claw opened successfully");
    }

private:
    Drone* drone;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point claw_open_start_time;
    bool claw_opened_;
    bool gpio_initialized_;
    bool claw_opening_started_;
    
    static constexpr int GPIO_PIN = 13;  // GPIO 13 corresponds to physical pin 33
    
    void initializeGPIO() {
        try {
            // Export the GPIO pin
            std::ofstream export_file("/sys/class/gpio/export");
            if (export_file.is_open()) {
                export_file << GPIO_PIN;
                export_file.close();
                drone->log("GPIO pin " + std::to_string(GPIO_PIN) + " (pin 33) exported");
            }
            
            // Set GPIO direction to output (no usleep - non-blocking)
            std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(GPIO_PIN) + "/direction";
            std::ofstream direction_file(direction_path);
            if (direction_file.is_open()) {
                direction_file << "out";
                direction_file.close();
                drone->log("GPIO direction set to output");
            }
            
            // Initialize claw to closed position (LOW signal)
            std::string value_path = "/sys/class/gpio/gpio" + std::to_string(GPIO_PIN) + "/value";
            std::ofstream value_file_init(value_path);
            if (value_file_init.is_open()) {
                value_file_init << "0";  // LOW signal (closed position)
                value_file_init.close();
                drone->log("Claw initialized to CLOSED position (LOW signal)");
                gpio_initialized_ = true;
            }
            
        } catch (const std::exception& e) {
            drone->log("Error initializing GPIO: " + std::string(e.what()));
            gpio_initialized_ = false;
        }
    }
    
    void startOpeningClaw() {
        try {
            std::string value_path = "/sys/class/gpio/gpio" + std::to_string(GPIO_PIN) + "/value";
            
            // Open the claw fully by setting GPIO to HIGH
            std::ofstream value_file_open(value_path);
            if (value_file_open.is_open()) {
                value_file_open << "1";  // HIGH signal (fully open)
                value_file_open.close();
                drone->log("CLAW OPENING: HIGH signal sent to GPIO pin 33 - Starting claw opening");
            }
            
        } catch (const std::exception& e) {
            drone->log("Error starting claw opening: " + std::string(e.what()));
        }
    }
    
    void finishOpeningClaw() {
        try {
            drone->log("Claw fully opened - Gancho dropped successfully");
            
            // Optional: Set back to LOW if your claw latches in the open position
            // Uncomment the following lines if your claw mechanism holds the open position
            /*
            std::string value_path = "/sys/class/gpio/gpio" + std::to_string(GPIO_PIN) + "/value";
            std::ofstream value_file_maintain(value_path);
            if (value_file_maintain.is_open()) {
                value_file_maintain << "0";  // Return to LOW if claw latches open
                value_file_maintain.close();
                drone->log("GPIO signal returned to LOW (claw latched open)");
            }
            */
            
        } catch (const std::exception& e) {
            drone->log("Error finishing claw opening: " + std::string(e.what()));
        }
    }
};