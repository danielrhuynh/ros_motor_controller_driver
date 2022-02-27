// header guards
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <map>
#include <string>

using namespace std;

// Driver for a motor speed controller
class MotorDriver
{
    private:
        int max_speed;
        int speed;
        int voltage;
        int temperature;
    public:
        // Initial communication. Set default settings
        // Default constructor
        MotorDriver(int max_speed0 = 10)
        {
            max_speed = max_speed0;
            speed = 0;
            voltage = 12;
            temperature = 47;
        }

        // Setters
        // Setting motor speed
        void set_speed (int speed0)
        {
            if (speed0 < max_speed)
            {
                speed = speed0;
            }
            else
            {
                speed = max_speed;
            }
        }

        // Getters
        // Get motor speed
        int get_speed()
        {
            return speed;
        }

        // Uses a key, value map to return hardware information to motor
        map<string, string> get_status()
        {
            map<string, string> status;
            status["temperature"] = to_string(temperature);
            status["voltage"] = to_string(voltage);

            return status;
        }

        // Stop the motor
        void stop()
        {
            speed = 0;
        }
};

#endif