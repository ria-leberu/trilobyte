/**
 * @file mcu_data_processor.cpp
 * @author rialia (Jean Nadela) (you@domain.com)
 * @brief Processes serial data to and from the MCU. Recieves odometry data and sends movement commands.
 * -----
 * Subscription Topics:
 * 
 * -----
 * Publishing Topics:
 * 
 * -----
 * @version 0.1
 * @date 2024-08-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ client library for node creation and management
#include "std_msgs/msg/string.hpp" // Standard message type for string messages
#include <string>



using namespace std::chrono_literals; // Enables the specification of a time duration
 
/**
 * @class MinimalPublisher
 * @brief Defines a minimal ROS 2 publisher node.
 *
 * This class inherits from rclcpp::Node and demonstrates creating a publisher,
 * publishing messages, and using a timer callback in ROS 2.
 */
class MinimalPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a MinimalPublisher node.
     *
     * Sets up a publisher for 'std_msgs::msg::String' messages on the "topic_cpp" topic
     * and initializes a timer to call the timerCallback method.
     */
    MinimalPublisher() : Node("minimal_publisher"), count_(0) 
    {
        // Create a publisher object for sending string messages on the "topic_cpp" topic 
        // with a queue size of 10.
        publisher_ = create_publisher<std_msgs::msg::String>("topic_cpp", 10);
     
        // Set up a timer to call the timerCallback function 
        timer_ = create_wall_timer(500ms, std::bind(&MinimalPublisher::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 2 Hz");
    }
 
    /**
     * @brief Timer callback function.
     *
     * This method is called at a fixed interval. It publishes a string message
     * containing "Hello World" followed by a sequence number.
     * 
     *  @return Void.
     */
    void timerCallback() 
    {
       
        // Create a new String message object.
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " ;

        // Publish the message
        publisher_->publish(message);
    }
     
private:
    // Member variables.
    size_t count_; // A counter to keep track of the number of messages published.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // The publisher object.
    rclcpp::TimerBase::SharedPtr timer_; // Timer for scheduling the publishing task.
};
 
/**
 * @brief Main function.
 *
 * Initializes the ROS 2 system and runs the minimal_publisher node. It keeps the node
 * alive until it is manually terminated.
 */
int main(int argc, char * argv[])
{

 
  // Initialize ROS 2.
  rclcpp::init(argc, argv); 
   
  // Create an instance of the MinimalPublisher node and keep it running.
  auto minimal_publisher_node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(minimal_publisher_node);
 
  // Shutdown ROS 2 upon node termination.
  rclcpp::shutdown(); 
 
  // End of program.
  return 0; 

}