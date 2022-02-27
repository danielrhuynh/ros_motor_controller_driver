#include <ros/ros.h>
#include "robot_driver/motor_controller_driver.hpp"
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace std;

// Create wrapper with OOP so we can use a smart pointer to hold the MotorDriver instance
class MotorDriverROSWrapper
{
    private:
        // Smart pointer to hold MotorDriver instance
        unique_ptr<MotorDriver> motor;

        // Subscriber for setting motor speed
        ros::Subscriber speed_command_subscriber;
        ros::ServiceServer stop_motor_server;

        // Defining publishers, timers and frequencies for 'speed', and 'status'
        // Publishers
        ros::Publisher speed_publisher;
        ros::Publisher status_publisher;
        // Timers to publish data (schedule a callback) at a fixed rate
        ros::Timer speed_timer;
        ros::Timer status_timer;
        // Frequencies
        double publish_speed_frequency;
        double publish_status_frequency;

    public:
        MotorDriverROSWrapper(ros::NodeHandle *nh)
        {
            // Declaring max_speed
            int max_speed;
            // Checking for the max_speed param and if DNE
            // Setting default value to 8
            if (!ros::param::get("~max_speed", max_speed))
            {
                max_speed = 8;
            }

            // Checking for the publish_speed_frequency and publish_status_frequency params and if DNE
            // Setting default values
            if (!ros::param::get("~publish_speed_frequency", publish_speed_frequency))
            {
                publish_speed_frequency = 5.0;
            }
            if (!ros::param::get("~publish_status_frequency", publish_status_frequency))
            {
                publish_status_frequency = 1.0;
            }

            // Saves copy of current pointer, overwrites current pointer to new MotorDriver
            // If the old pointer that was set to the current pointer is non-empty, delete previously managed object
            // Sort of like changing a link in a node
            motor.reset(new MotorDriver (max_speed));

            // Using a roscpp subscriber to wrap the set_speed method from driver
            speed_command_subscriber = nh->subscribe // Equivalent to nh.subscribe()
            (
                // subscribed is subscribed to the 'speed_command topic' with a max queue size of 10
                // Speeds are passed to the callbackSpeedCommand function and the speed of the motor instance
                // Is set to the value of the message
                "speed_command", 10, &MotorDriverROSWrapper::callbackSpeedCommand, this);

            // Provide a service name and a callback to be invoked when the service is called
            // Use the NodeHandle to create the Service server
            stop_motor_server = nh->advertiseService
            (
                "stop_motor", &MotorDriverROSWrapper::callbackStop, this);

            // Initializing the publishers created in private scope
            // Publishers have message type of Int32, topics of 'speed' and 'status' and queue sizes of 10
            speed_publisher = nh->advertise<std_msgs::Int32>("speed", 10);
            status_publisher = nh->advertise<diagnostic_msgs::DiagnosticStatus>("status", 10);

            // Initializing the timers created in private scope
            // Duration is set to 1.0s / frequency and the callback function will publish the motor
            // Speed or status
            speed_timer = nh->createTimer(ros::Duration(1.0/publish_speed_frequency), &MotorDriverROSWrapper::publishSpeed, this);
            status_timer = nh->createTimer(ros::Duration(1.0/publish_status_frequency), &MotorDriverROSWrapper::publishStatus, this);
        }

        // Publishing motor speed based on set frequency
        // Passes in timer information from speed and status timers (from TimerEvent structure)
        void publishSpeed (const ros::TimerEvent &event)
        {
            // Create msg of type Int32
            std_msgs::Int32 msg;

            // get speed from MotorDriver and sets msg data
            msg.data = motor->get_speed();

            // Publish msg on speed topic
            speed_publisher.publish(msg);
        }

        void publishStatus (const ros::TimerEvent &event)
        {
            // msg holds the status of an individual component (KeyValue values)
            // msg in this case contains diagnostic data about the motor (voltage, temperature)
            diagnostic_msgs::DiagnosticStatus msg;
            // Getting the status map from MotorDriver
            map<string, string> status = motor->get_status();
            // Making a vector of type KeyValue from diagnostic_msgs namespace 
            // That has a key which is the label of the value when viewing, has a value which is tracked over time
            vector<diagnostic_msgs::KeyValue> values;

            // Range based for loop that iterates over the status map from MotorDriver
            // Letting x be the 'auto' type lets the compiler decide its data type, this increases the flexibillity of x
            // Since the type of x will be the type of the key (string) or value (int or string based on implementation of driver) from the get_status map from MotorDriver
            for (auto const &x : status)
            {
                // Create a KeyValue object to hold the key and value (both strings) from the status pair
                diagnostic_msgs::KeyValue value;
                // Setting the key of value to the first value in pair (the key)
                value.key = x.first;
                // Setting the value of value to the second value in pair (the value)
                value.value = x.second;

                // Adds new value to the end of the values vector
                values.push_back(value);
            }

            // Sets the msg array of values associated with the status to 'values'
            msg.values = values;
            // Publish msg on status topic
            status_publisher.publish(msg);
        }

        // Stops the motor using the drivers stop method
        void stop()
        {
            motor->stop();
        }

        // Setting motor speed with message from subscriber of "speed_command"
        void callbackSpeedCommand(const std_msgs::Int32 &msg)
        {
            motor->set_speed(msg.data);
        }

        // Callback for 'stop_motor' 
        // Trigger is a service type of std_srvs that adds the possibility to check if triggering was successful
        // The trigger service contains an empty request and a success flag + message
        // Calls stop() to stop the motor when the 'stop_motor' service is called and the req / res
        // Types match what is provided by stop_motor_server
        bool callbackStop (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
        {
            // call stop function
            stop();
            res.success = true;
            res.message = "Suessfully stopped motor.";
            return true;
        }
};

int main (int argc, char **argv)
{
    
    // Initializing the node with node name
    ros::init (argc, argv, "motor_controller_driver");

    // Start roscpp node
    ros::NodeHandle nh;

    // Implement AsyncSpinner to avoid delayed callback issues
    ros::AsyncSpinner spinner(4); // Uses 4 threads
    spinner.start();

    // Create a wrapper instance and pass NodleHandle from constructor
    MotorDriverROSWrapper motor_driver_ros_wrapper(&nh);
    // Log info
    ROS_INFO("Motor driver has been started");

    // Keep driver alive
    ros::waitForShutdown();
    // Stop driver
    motor_driver_ros_wrapper.stop();
}