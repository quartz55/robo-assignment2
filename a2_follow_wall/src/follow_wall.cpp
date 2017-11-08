#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>

enum Wall {
    NONE,
    LEFT,
    RIGHT
};

struct State {
    sensor_msgs::Range left;
    sensor_msgs::Range right;
    Wall following;
};

State state;

void left_sensor_callback(const sensor_msgs::Range range)
{
    state.left = range;
}

void right_sensor_callback(const sensor_msgs::Range range)
{
    state.right = range;
}

float distance_percentage(const sensor_msgs::Range range)
{
    return (range.range - range.min_range) / (range.max_range - range.min_range);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follow_wall");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
    ros::Subscriber left_sub = nh.subscribe("robot0/sonar_left", 1000, left_sensor_callback);
    ros::Subscriber right_sub = nh.subscribe("robot0/sonar_right", 1000, right_sensor_callback);

    ros::Rate rate(10);

    state.following = Wall::NONE;
    while(ros::ok()) {
        // Check sensors
        ros::spinOnce();

        //// Subsumption behaviours
        geometry_msgs::Twist msg;
        float distance_left = distance_percentage(state.left);
        float distance_right = distance_percentage(state.right);
        // Avoid collision
        if (distance_left + distance_right < 0.5) {
            if (state.following == Wall::RIGHT) {
                msg.angular.z = 1;
                msg.linear.x = 0.2;
            } else {
                msg.angular.z = -1;
                msg.linear.x = 0.2;
            }
        } else if (distance_left < 0.1) {
            msg.angular.z = -1;
        } else if (distance_right < 0.1) {
            msg.angular.z = 1;
        // Follow wall
        } else if (distance_left >= 0.1 && distance_left < 1 && state.following != Wall::RIGHT) {
            state.following = Wall::LEFT;
            msg.linear.x = 0.6;
            msg.angular.z = -4 * (0.25 - distance_left);
        } else if (distance_right >= 0.1 && distance_right < 1 && state.following != Wall::LEFT) {
            state.following = Wall::RIGHT;
            msg.linear.x = 0.6;
            msg.angular.z = 4 * (0.25 - distance_right);
        // Wander            
        } else {
            state.following = Wall::NONE;
            msg.linear.x = 1;
            msg.angular.z = 2 * double(rand())/double(RAND_MAX) - 1;
        }

        pub.publish(msg);
        
        rate.sleep();
    }
    
    return 0;
}