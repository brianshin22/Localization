#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Pose2D.h"

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>
#include "boost/filesystem.hpp"
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

void readCSV(filename, const sensor_msgs::PointCloudConstPtr& cloud_msg)
{
    ifstream file(filename);
    //vector<float> x,y,z;


    if(file.is_open())
    {
        string line;
        int index = 0;

        cloud_msg->header.frameId = "0";
            
        while(getline(file,line))
        {
            typedef boost::escaped_list_separator<char> Separator;
            typedef boost::tokenizer<Separator> Tokenizer;
            vector<string> tokens;
            Tokenizer tokenizer(line);
            for(auto &token:tokenizer)
            {
                tokens.push_back(token);
            }
            if (tokens.size() == 12)
            {
                if (index > 0)
                {
                    geometry_msgs::Point32 current_point;
                    current_point.x = tokens[4];
                    current_point.y = tokens[5];
                    current_point.z = tokens[6];
                    cloud_msg->points.push_back(current_point);
                    //x.push_back(tokens[4]);
                    //y.push_back(tokens[5]);
                    //z.push_back(tokens[6]);
                }
            }
            else
                cerr << "Illegal line\n";
            index = index + 1;
    
        }

    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_sender");//node name: ICP_talker
    ros::NodeHandle n;

    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("ibeo_point_cloud", 1);

    int rate = 25;

    ros::Rate loop_rate(rate);//frequency
    
    sensor_msgs::PointCloud cloud_msg;
    
    filepath = "../../../../Data/pointcloud_test/";

    int count = 0;

    int fileidx = 24523;

    while (ros::ok() && fileidx <= 24586)
    {
        string filename = filepath + "default_log_" + to_string(fileidx++) + ".csv";
        readCSV(filename,cloud_msg);
        
 
        //Publish pose info
        cloud_pub.publish(cloud_msg);


        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}

