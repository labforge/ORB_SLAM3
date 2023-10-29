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

//#include"../../../include/System.h"

#include "common.h"
#include <orb_slam3_ros_wrapper/KeyPoint.h>
#include <orb_slam3_ros_wrapper/KeyPoints.h>
#include <orb_slam3_ros_wrapper/GlobalDescriptor.h>
#include <orb_slam3_ros_wrapper/Descriptor.h>
#include <zlib.h>


using namespace std;
double ttrack = 0;
double ttrack_tot = 0;
int nImages = 0;
std::vector<float> vTimesTrack;
cv::Ptr<cv::CLAHE> clahe;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& img_msg_l, const orb_slam3_ros_wrapper::KeyPointsConstPtr& kpts_msg_l, const orb_slam3_ros_wrapper::DescriptorConstPtr& descs_msg_l,
                   const sensor_msgs::ImageConstPtr& img_msg_r, const orb_slam3_ros_wrapper::KeyPointsConstPtr& kpts_msg_r, const orb_slam3_ros_wrapper::DescriptorConstPtr& descs_msg_r);

    std::vector<cv::KeyPoint> GetKpts(const orb_slam3_ros_wrapper::KeyPoints &kpts_msg);
    cv::KeyPoint keypointFromROS(const orb_slam3_ros_wrapper::KeyPoint & msg);
    cv::Mat GetDesc(const orb_slam3_ros_wrapper::Descriptor &desc_msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Acc");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);
    
    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    //node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    //node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> image_sub_l(node_handler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::KeyPoints> keypoints_sub_l(node_handler, "/camera/left/keypoints", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::Descriptor> descriptors_sub_l(node_handler, "/camera/left/descriptors", 1);

    message_filters::Subscriber<sensor_msgs::Image> image_sub_r(node_handler, "/camera/right/image_raw", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::KeyPoints> keypoints_sub_r(node_handler, "/camera/right/keypoints", 1);
    message_filters::Subscriber<orb_slam3_ros_wrapper::Descriptor> descriptors_sub_r(node_handler, "/camera/right/descriptors", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, orb_slam3_ros_wrapper::KeyPoints, orb_slam3_ros_wrapper::Descriptor,
                                                            sensor_msgs::Image, orb_slam3_ros_wrapper::KeyPoints, orb_slam3_ros_wrapper::Descriptor> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), image_sub_l,keypoints_sub_l, descriptors_sub_l,
                                                               image_sub_r,keypoints_sub_r, descriptors_sub_r);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2,_3,_4,_5,_6));

    setup_ros_publishers(node_handler, image_transport, sensor_type);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();
    
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    std::ofstream file("ttrack.csv");
    for (const auto& value : vTimesTrack) {
        file << value << std::endl;
    }
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    float mean, median;
    mean = totaltime/nImages;
    median = vTimesTrack[nImages/2];
    cout << "median tracking time: " << median << endl;
    cout << "mean tracking time: " << mean << endl;
    //cout <<"proccIm: "<< proccIm << endl;
    //cout <<"nImages[0]: "<< nImages[0] << endl;
    //while(1);
    file << "median tracking time: " << median << std::endl;
    file << "mean tracking time: " << mean << std::endl;
    file.close();

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

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& img_msg_l, const orb_slam3_ros_wrapper::KeyPointsConstPtr& kpts_msg_l, const orb_slam3_ros_wrapper::DescriptorConstPtr& descs_msg_l,
                             const sensor_msgs::ImageConstPtr& img_msg_r, const orb_slam3_ros_wrapper::KeyPointsConstPtr& kpts_msg_r, const orb_slam3_ros_wrapper::DescriptorConstPtr& descs_msg_r)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr_l;
    try
    {
        cv_ptr_l = cv_bridge::toCvShare(img_msg_l, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::KeyPoint> opencv_keypoints_l = GetKpts(*kpts_msg_l);

    cv::Mat opencv_descriptors_l = GetDesc(*descs_msg_l);

    cv_bridge::CvImageConstPtr cv_ptr_r;
    try
    {
        cv_ptr_r = cv_bridge::toCvShare(img_msg_r, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::KeyPoint> opencv_keypoints_r = GetKpts(*kpts_msg_r);

    cv::Mat opencv_descriptors_r = GetDesc(*descs_msg_r);

    nImages++;
    clahe->apply(cv_ptr_l->image,cv_ptr_l->image);
    clahe->apply(cv_ptr_r->image,cv_ptr_r->image);
    //std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    mpSLAM->TrackStereo(cv_ptr_l->image,opencv_keypoints_l, opencv_descriptors_l,
                           cv_ptr_r->image,opencv_keypoints_r, opencv_descriptors_r,
                           cv_ptr_l->header.stamp.toSec());
    //mpSLAM->TrackStereo(imLeft,kptsLeft,descsLeft,imRight,kptsRight,descsRight,tImLeft);

    //std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    ttrack_tot += ttrack;
    //if(ttrack > 0.05)
        //cerr << "ttrack: " << ttrack << std::endl;
    vTimesTrack.push_back(ttrack);
}


