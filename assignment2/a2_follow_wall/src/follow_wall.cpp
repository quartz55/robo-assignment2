#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>

enum Wall
{
    NONE,
    LEFT,
    RIGHT
};

struct State
{
    sensor_msgs::Range left;
    sensor_msgs::Range right;
    Wall following;
    Wall is_searching;
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

void give_up(const ros::TimerEvent &)
{
    ROS_INFO("Stopped searching");
    state.is_searching = Wall::NONE;
    state.following = Wall::NONE;
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
    ros::Timer search_timer;

    ros::Rate rate(10);

    state.following = Wall::NONE;
    state.is_searching = Wall::NONE;
    while (ros::ok())
    {
        // Check sensors
        ros::spinOnce();

        //// Subsumption behaviors
        geometry_msgs::Twist msg;
        float distance_left = distance_percentage(state.left);
        float distance_right = distance_percentage(state.right);
        // Avoid collision
        if (distance_left < 0.1)
        {
            msg.angular.z = -1;
        }
        else if (distance_right < 0.1)
        {
            msg.angular.z = 1;
        }
        // Follow wall
        else if (distance_left >= 0.1 && distance_left < 1 && (state.following != Wall::RIGHT || state.is_searching != Wall::NONE))
        {
            state.is_searching = Wall::NONE;
            state.following = Wall::LEFT;
            msg.linear.x = 0.6;
            msg.angular.z = -4 * (0.25 - distance_left);
        }
        else if (distance_right >= 0.1 && distance_right < 1 && (state.following != Wall::LEFT || state.is_searching != Wall::NONE))
        {
            state.is_searching = Wall::NONE;
            state.following = Wall::RIGHT;
            msg.linear.x = 0.6;
            msg.angular.z = 4 * (0.25 - distance_right);
        }
        else
        {
            // Wall lost
            if (state.following != Wall::NONE)
            {
                state.is_searching = state.following;
                state.following = Wall::NONE;
                search_timer = nh.createTimer(ros::Duration(1), give_up, true);
            }
            // Try to recover wall
            if (state.is_searching != Wall::NONE)
            {
                msg.angular.z = state.is_searching == Wall::LEFT ? 1 : -1;
            }
            // Wander
            else
            {
                msg.linear.x = 1;
                msg.angular.z = 4 * double(rand()) / double(RAND_MAX) - 2;
            }
        }

        pub.publish(msg);

        rate.sleep();
    }

    return 0;
}