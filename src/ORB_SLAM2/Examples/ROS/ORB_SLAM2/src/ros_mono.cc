/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
// OyukiRojas
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include"../../../include/System.h"

// OyukiRojas
#include <geometry_msgs/PoseStamped.h>
#include "Converter.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    // OyukiRojas
	void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    // OyukiRojas
	ros::Publisher* pPosPub;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

	//Twist Pose - OyukiRojas
    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("/ORB2/pose", 5);
    igb.pPosPub = &(PosPub);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
	// OyukiRojas
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    PublishPose(Tcw);
}

// OyukiRojas
void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {
		
		//~ std::cout<<"===== ORB SLAM Tcw only: "<<Tcw<<std::endl;
		//~ std::cout<<"===== ORB SLAM Tcw "<<Tcw.size()<<std::endl;
		
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
   
        poseMSG.pose.position.x = twc.at<float>(2);
        poseMSG.pose.position.y = -twc.at<float>(0);
        poseMSG.pose.position.z = -twc.at<float>(1);

        poseMSG.pose.orientation.x = q[2];
        poseMSG.pose.orientation.y = -q[0];
        poseMSG.pose.orientation.z = -q[1];
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "world";
        poseMSG.header.stamp = ros::Time::now();
        
        (pPosPub)->publish(poseMSG);

    }
}
