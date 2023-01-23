/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo_inertial.cc
*
*/

#include "common.h"
#include <orb_slam3_ros_wrapper/KeyPoint.h>
#include <orb_slam3_ros_wrapper/KeyPoints.h>
#include <orb_slam3_ros_wrapper/GlobalDescriptor.h>
#include <orb_slam3_ros_wrapper/Descriptor.h>
#include <zlib.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM): mpSLAM(pSLAM){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    void GrabKptsLeft(const orb_slam3_ros_wrapper::KeyPoints& msg);
    void GrabDescsLeft(const orb_slam3_ros_wrapper::Descriptor& msg);
    void GrabKptsRight(const orb_slam3_ros_wrapper::KeyPoints& msg);
    void GrabDescsRight(const orb_slam3_ros_wrapper::Descriptor& msg);
    
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    std::vector<cv::KeyPoint> GetKpts(const orb_slam3_ros_wrapper::KeyPoints &kpts_msg);
    cv::KeyPoint keypointFromROS(const orb_slam3_ros_wrapper::KeyPoint & msg);
    cv::Mat GetDesc(const orb_slam3_ros_wrapper::Descriptor &desc_msg);
    
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    queue<orb_slam3_ros_wrapper::KeyPoints> kptsLeftBuf, kptsRightBuf;
    queue<orb_slam3_ros_wrapper::Descriptor> descsLeftBuf, descsRightBuf;

    std::mutex mBufMutexLeft,mBufMutexRight,mBufMutexLeftKpts,mBufMutexRightKpts,mBufMutexLeftDescs,mBufMutexRightDescs;
    
    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

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

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb(&SLAM);

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = node_handler.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);
    ros::Subscriber sub_kpts_left = node_handler.subscribe("/camera/left/keypoints", 100, &ImageGrabber::GrabKptsLeft, &igb);
    ros::Subscriber sub_descs_left = node_handler.subscribe("/camera/left/descriptors", 100, &ImageGrabber::GrabDescsLeft, &igb);
    ros::Subscriber sub_kpts_right = node_handler.subscribe("/camera/right/keypoints", 100, &ImageGrabber::GrabKptsRight, &igb);
    ros::Subscriber sub_descs_right = node_handler.subscribe("/camera/right/descriptors", 100, &ImageGrabber::GrabDescsRight, &igb);

    setup_ros_publishers(node_handler, image_transport, sensor_type);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

void ImageGrabber::GrabKptsLeft(const orb_slam3_ros_wrapper::KeyPoints &kpts_msg)
{
    mBufMutexLeftKpts.lock();
    if (!kptsLeftBuf.empty())
        kptsLeftBuf.pop();
    kptsLeftBuf.push(kpts_msg);
    mBufMutexLeftKpts.unlock();
}

void ImageGrabber::GrabKptsRight(const orb_slam3_ros_wrapper::KeyPoints &kpts_msg)
{
    mBufMutexRightKpts.lock();
    if (!kptsRightBuf.empty())
        kptsRightBuf.pop();
    kptsRightBuf.push(kpts_msg);
    mBufMutexRightKpts.unlock();
}

void ImageGrabber::GrabDescsLeft(const orb_slam3_ros_wrapper::Descriptor &desc_msg)
{
    mBufMutexLeftDescs.lock();
    if (!descsLeftBuf.empty())
        descsLeftBuf.pop();
    descsLeftBuf.push(desc_msg);
    mBufMutexLeftDescs.unlock();
}

void ImageGrabber::GrabDescsRight(const orb_slam3_ros_wrapper::Descriptor &desc_msg)
{
    mBufMutexRightDescs.lock();
    if (!descsRightBuf.empty())
        descsRightBuf.pop();
    descsRightBuf.push(desc_msg);
    mBufMutexRightDescs.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

cv::KeyPoint ImageGrabber::keypointFromROS(const orb_slam3_ros_wrapper::KeyPoint & msg)
{
	return cv::KeyPoint(msg.pt.x, msg.pt.y, msg.size, msg.angle, msg.response, msg.octave, msg.class_id);
}

// std::vector<cv::KeyPoint> keypointsFromROS(const std::vector<rtabmap_ros::KeyPoint> & msg) - used this code in GetKpts function below

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

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        std::vector<cv::KeyPoint> kptsLeft, kptsRight;
        cv::Mat descsLeft, descsRight;
        double tImLeft = 0, tImRight = 0, tKptsLeft = 0, tDescsLeft = 0, tKptsRight = 0, tDescsRight = 0;

        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!kptsLeftBuf.empty()&&!descsLeftBuf.empty()&&!kptsRightBuf.empty()&&!descsRightBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();
            tKptsLeft = kptsLeftBuf.front().header.stamp.toSec();
            tKptsRight = kptsRightBuf.front().header.stamp.toSec();
            tDescsLeft = descsLeftBuf.front().header.stamp.toSec();
            tDescsRight = descsRightBuf.front().header.stamp.toSec();

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            ros::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();
            
            this->mBufMutexLeftKpts.lock();
            kptsLeft = GetKpts(kptsLeftBuf.front());
            kptsLeftBuf.pop();
            this->mBufMutexLeftKpts.unlock();

            this->mBufMutexRightKpts.lock();
            kptsRight = GetKpts(kptsRightBuf.front());
            kptsRightBuf.pop();
            this->mBufMutexRightKpts.unlock();

            this->mBufMutexLeftDescs.lock();
            descsLeft = GetDesc(descsLeftBuf.front());
            descsLeftBuf.pop();
            this->mBufMutexLeftDescs.unlock();

            this->mBufMutexRightDescs.lock();
            descsRight = GetDesc(descsRightBuf.front());
            descsRightBuf.pop();
            this->mBufMutexRightDescs.unlock();

            // ORB-SLAM3 runs in TrackStereo()
            //Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft);
            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,kptsLeft,descsLeft,imRight,kptsRight,descsRight,tImLeft);
            Sophus::SE3f Twc = Tcw.inverse();
            
            publish_ros_camera_pose(Twc, msg_time);
            publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
            publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
            
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}