#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#define WHEEL_BASE 1.6
#define WHEEL_RADIUS 0.5
double lin_vel_current, ang_vel_current;

long map1(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    lin_vel_current = odom -> twist.twist.linear.x;
}

void gyroCallback(const sensor_msgs::Imu::ConstPtr& gyro){
    ang_vel_current = gyro -> angular_velocity.z;
}

void match_vel(const geometry_msgs::Twist::ConstPtr& vel_final){
    
    double vel_left_cmd = (lin_vel_current - ang_vel_current * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
    double vel_right_cmd = (lin_vel_current + ang_vel_current * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

    vel_left_cmd = map1(vel_left_cmd, -1.8, 1.8, -7999, 7999);
    vel_right_cmd = map1(vel_right_cmd, -1.8, 1.8, -7999, 7999);

    double lin_vel_final = vel_final -> linear.x;
    double ang_vel_final = vel_final -> angular.z;
    for(int i = 0; i <= 800; i++){

        if(ang_vel_current - 0.1 > ang_vel_final){

            if(vel_right_cmd > 7989 && vel_left_cmd > -7989)
                vel_left_cmd -= 10;

            else if(vel_right_cmd <= 7989)
                vel_right_cmd += 10;

            else ROS_ERROR("Angular Velocity Unattainable. %f,%f at max",vel_left_cmd,vel_right_cmd);
        }

        else if(ang_vel_current + 0.1 < ang_vel_final){

            if(vel_left_cmd > 7989 && vel_right_cmd > -7989)
                vel_right_cmd -= 10;

            else if(vel_left_cmd <= 7989)
                vel_left_cmd += 10;
                
            else ROS_ERROR("Angular Velocity Unattainable");
        }

        else ROS_INFO("Angular Velocity attained");

        if(lin_vel_current + 0.1 <= lin_vel_final){
        
            if(vel_left_cmd > 7989 || vel_right_cmd > 7989){
                ROS_ERROR("Linear Velocity Unattainable");
            }
            else{
                vel_right_cmd += 10;
                vel_left_cmd += 10;
            }
        }

        else if(lin_vel_current - 0.1 >= lin_vel_final){

            if(vel_left_cmd < 7989 || vel_right_cmd < 7989){
                ROS_ERROR("Linear Velocity Unatainable");
            }
            else{
            vel_right_cmd -= 10;
            vel_left_cmd -= 10;
            }
        }
        else ROS_INFO("Linear Velocity attained");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_cmd_publisher");
	ros::NodeHandle nh;
    ros::Subscriber odom_in = nh.subscribe("odometry/r_odom", 100, odomCallback);
    ros::Subscriber imu_in = nh.subscribe("imu_data/raw", 100, gyroCallback);
    ros::Subscriber cmd_vel = nh.subscribe("cmd_vel", 100, match_vel);
    ros::spin();
    return 0;
}