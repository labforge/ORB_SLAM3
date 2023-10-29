/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg, const orb_slam3_ros_wrapper::KeyPoints& msg, const orb_slam3_ros_wrapper::Descriptor& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoAcc");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/image_raw", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::KeyPoints> keypoints_sub(nh, "/camera/keypoints", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::Descriptor> descriptors_sub(nh, "/camera/descriptors", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, orb_slam3_ros_wrapper::KeyPoints, orb_slam3_ros_wrapper::Descriptor> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), image_sub,keypoints_sub, descriptors_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

cv::KeyPoint ImageGrabber::keypointFromROS(const orb_slam3_ros_wrapper::KeyPoint & msg)
{
	return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

std::vector<cv::KeyPoint> ImageGrabber::GetKpts(const orb_slam3_ros_wrapper::KeyPoints &kpts_msg)
{
    std::vector<cv::KeyPoint> v(kpts_msg.keypoints.size());
	for(unsigned int i=0; i<kpts_msg.keypoints.size(); ++i)
	{
		v[i] = keypointFromROS(kpts_msg.keypoints[i]);
	}
	return v;
}

cv::Mat ImageGrabber::GetDesc(const orb_slam3_ros_wrapper::Descriptor & desc_msg)
{
	cv::Mat data;
	if(!desc_msg.descdata.empty())
	{
		if(desc_msg.cols > 0 && desc_msg.rows > 0 && desc_msg.type >= 0)
		{
			data = cv::Mat(desc_msg.rows, desc_msg.cols, desc_msg.type, (void*)desc_msg.descdata.data()).clone();
		}
		else
		{
			if(desc_msg.cols != (int)desc_msg.descdata.size() || desc_msg.rows != 1 || desc_msg.type != CV_8UC1)
			{
				cout << "cols, rows and type fields of the UserData msg "
						"are not correctly set (cols= "<<desc_msg.cols<<" rows= "<<desc_msg.rows<<", type= "<<desc_msg.type<<" )! We assume that the data "
						"is compressed (cols= "<<(int)desc_msg.descdata.size()<< ", rows=1, type= "<<CV_8UC1<<"(CV_8UC1))." << endl;

			}
			data = cv::Mat(1, desc_msg.descdata.size(), CV_8UC1, (void*)desc_msg.descdata.data()).clone();
		}
	}
	return data;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& img_msg, const orb_slam3_ros_wrapper::KeyPoints& kpts_msg, const orb_slam3_ros_wrapper::Descriptor& descs_msg);
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        //cv_ptr = cv_bridge::toCvShare(msg);
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::KeyPoint> opencv_keypoints = GetKpts(kpts_msg);

    cv::Mat opencv_descriptors = GetDesc(descs_msg);

    //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    //SLAM.TrackMonocular(im,opencv_keypoints, opencv_descriptors, tframe);
    mpSLAM->TrackMonocular(cv_ptr->image,opencv_keypoints, opencv_descriptors,cv_ptr->header.stamp.toSec());
}


