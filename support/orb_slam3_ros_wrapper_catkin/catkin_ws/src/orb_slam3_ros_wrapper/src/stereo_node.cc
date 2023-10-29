/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
*
*/

#include "common.h"

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

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
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

    //message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    //setup_ros_publishers(node_handler, image_transport, sensor_type);

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

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    nImages++;
    clahe->apply(cv_ptrLeft->image,cv_ptrLeft->image);
    clahe->apply(cv_ptrRight->image,cv_ptrRight->image);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Main algorithm runs here
    Sophus::SE3f Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    //cerr << "ttrack: " << ttrack << std::endl;
    vTimesTrack.push_back(ttrack);
/*
    Sophus::SE3f Twc = Tcw.inverse();

    ros::Time msg_time = cv_ptrLeft->header.stamp;

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
*/

}
