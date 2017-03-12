#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Pose2D.h"

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
void DPCallback(const PointMatcher<float>::DataPoints message);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ICP_talker");//node name: ICP_talker
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

    ros::NodeHandle ICPNode;

    global DP ref;

    ros::Subscriber DP_sub = ICPNode.subscribe<sensor_msgs::PointCloud>("ibeo_point_cloud", 1, DPCallback); //topic: ICP_Output

    ros::Publisher ICP_pub = ICPNode.advertise<geometry_msgs::Pose2D>("ICP_Output", 1000); //topic: ICP_Output

    ros::Rate loop_rate(25);//frequency

    geometry_msgs::Pose2D lidar_pose;
    int count = 0;
    // int d_psi, d_x, d_y;

    while (ros::ok())
    {
        // Printing count for testing
        std_msgs::String msg;
        std::stringstream ss;
        ss <<count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        ++count;

        // Receive the current scan and store it into data

        // DP data(DP::load(argv[2]));

        // Create the default ICP algorithm
        PM::ICP icp;

        // See the implementation of setDefault() to create a custom ICP algorithm
        icp.setDefault();

        // Compute the transformation to express data in ref
        PM::TransformationParameters T = icp(data, ref);

        // Update pose info
        lidar_pose.theta = acos(T(0,0));
        lidar_pose.x = T(0,3);
        lidar_pose.y = T(1,3);

        //Publish pose info
        ICP_pub.publish(lidar_pose);

        //Replace the previous lidar scan with the current lidar scan
        ref = data;

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}

<<<<<<< HEAD
void DPCallback(const sensor_msgs::PointCloud2ConstPtr & message)
{
  global DP data;
  sensor_msgs::PointCloud2 point_cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(*message,point_cloud2)
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud2, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    data.addFeature("x", *iter_x);
    data.addFeature("y", *iter_y);
    data.addFeature("z", *iter_z);
=======
void DPCallback(const sensor_msgs::PointCloudConstPtr& cloud_msg)
{
  global DP data;
  sensor_msgs::PointCloud2 cloud_msg2;
  bool dummy = sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud_msg2);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  for ()
  {


>>>>>>> 0f401281065bc430b13a00c454c5eeab5fee7238
  }

}
