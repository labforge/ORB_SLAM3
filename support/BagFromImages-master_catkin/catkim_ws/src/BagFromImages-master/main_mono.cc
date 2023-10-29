#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "Thirdparty/DLib/FileFunctions.h"
#include <string>
#include <BagFromImages/KeyPoint.h>
#include <BagFromImages/KeyPoints.h>
#include <BagFromImages/GlobalDescriptor.h>
#include <BagFromImages/Descriptor.h>
#include <zlib.h>

using namespace std;


std::vector<unsigned char> compressData(const cv::Mat & data)
{
	std::vector<unsigned char> bytes;
	if(!data.empty())
	{
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
		uLong destLen = compressBound(sourceLen);
		bytes.resize(destLen);
		int errCode = compress(
						(Bytef *)bytes.data(),
						&destLen,
						(const Bytef *)data.data,
						sourceLen);

		bytes.resize(destLen+3*sizeof(int));
		*((int*)&bytes[destLen]) = data.rows;
		*((int*)&bytes[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes[destLen+2*sizeof(int)]) = data.type();

		if(errCode == Z_MEM_ERROR)
		{
			cout << "Z_MEM_ERROR : Insufficient memory." << endl;
		}
		else if(errCode == Z_BUF_ERROR)
		{
			cout << "Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data." << endl;
		}
	}
	return bytes;
}


void keypointToROS(const cv::KeyPoint & kpt, BagFromImages::KeyPoint & msg)
{
	msg.angle = kpt.angle;
	msg.class_id = kpt.class_id;
	msg.octave = kpt.octave;
	msg.pt.x = kpt.pt.x;
	msg.pt.y = kpt.pt.y;
	msg.response = kpt.response;
	msg.size = kpt.size;
}
/*
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, std::vector<BagFromImages::KeyPoint> & msg)
{
	msg.resize(kpts.size());
	for(unsigned int i=0; i<msg.size(); ++i)
	{
		keypointToROS(kpts[i], msg[i]);
	}
}
*/
void keypointsToROS(const std::vector<cv::KeyPoint> & kpts, BagFromImages::KeyPoints & msg)
{
	msg.keypoints.resize(kpts.size());
	for(unsigned int i=0; i<msg.keypoints.size(); ++i)
	{
		keypointToROS(kpts[i], msg.keypoints[i]);
	}
}

/*
void globalDescriptorToROS(const cv::mat & desc, BagFromImages::GlobalDescriptor & msg)
{
	//msg.type = desc.type();
	//msg.info = rtabmap::compressData(desc.info());
	msg.data = rtabmap::compressData(desc.data());
}

void globalDescriptorsToROS(const std::vector<cv::mat> & desc, std::vector<BagFromImages::GlobalDescriptor> & msg)
{
	msg.clear();
	if(!desc.empty())
	{
		msg.resize(desc.size());
		for(unsigned int i=0; i<msg.size(); ++i)
		{
			globalDescriptorToROS(desc[i], msg[i]);
		}
	}
}
*/

void descriptorToROS(const cv::Mat & data, BagFromImages::Descriptor & dataMsg, bool compress)
{
    //cout << "descriptorToROS" << endl;
	if(!data.empty())
	{
		if(compress)
		{
			//dataMsg.data = BagFromImages::compressData(data);
            dataMsg.descdata = compressData(data);
			dataMsg.rows = 1;
			dataMsg.cols = dataMsg.descdata.size();
			dataMsg.type = CV_8UC1;
		}
		else
		{
			dataMsg.descdata.resize(data.step[0] * data.rows); // use step for non-contiguous matrices
			memcpy(dataMsg.descdata.data(), data.data, dataMsg.descdata.size());
			dataMsg.rows = data.rows;
			dataMsg.cols = data.cols;
			dataMsg.type = data.type();
		}
	}
}

// TODO void descriptorsToROS(const std::vector<cv::Mat> & data, std::vector<BagFromImages::UserData> & dataMsg)
void descriptorsToROS(const std::vector<cv::Mat> & data, std::vector<BagFromImages::Descriptor> & dataMsg)
{
    //cout << "descriptorsToROS" << endl;
    dataMsg.resize(data.size());
	for(unsigned int i=0; i<dataMsg.size(); ++i)
	{
		descriptorToROS(data[i], dataMsg[i], false);
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");
    //cout << "argc: "<< argc;
    if(argc!=6)
    {
        cerr << "Usage: rosrun BagFromImages main_mono <path to image directory> <image extension .ext> <frequency> <Left keypts & Descriptors> <path to output bag>" << endl;
        return 0;
    }

    std::string left_dir = argv[1]; 
    //std::string right_dir = argv[2]; 
    std::string file_ext = argv[2];
    std::string freq_arg = argv[3];
    std::string left_kptsndescs_dir = argv[4]; 
    //std::string right_kptsndescs_dir = argv[6];
    std::string out_bag = argv[5];
    
    ros::start();

    // Vector of paths to image
    vector<string> left_filenames = DUtils::FileFunctions::Dir(left_dir.c_str(), file_ext.c_str(), true);
    //vector<string> right_filenames = DUtils::FileFunctions::Dir(right_dir.c_str(), file_ext.c_str(), true);

    cout << "Left Images: " << left_filenames.size() << endl;
    //cout << "Right Images: " << right_filenames.size() << endl;

    // Frequency
    double freq = stof(freq_arg);

    // Output bag
    rosbag::Bag bag_out(out_bag,rosbag::bagmode::Write);
    bag_out.setChunkThreshold(52428800);

    ros::Time t ;//= ros::Time::now();
    vector<vector<string>> content;
    vector<string> row;
    string line, word;
    std::vector<ros::Time> rosTimes;
    bool isFirstLine = true;

    cout << left_dir+"/../data.csv" << endl;
    fstream file (left_dir+"/../data.csv", ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            if (isFirstLine)
            {
                isFirstLine = false;
                continue;  // Skip reading the first line
            }
            row.clear();
             
            stringstream str(line);
             
            while(getline(str, word, ','))
                row.push_back(word);
            
            content.push_back(row);
            
        }
    }
    else
    {
        cout<<"Could not open the file\n";
    }
    //content.erase(content.begin());
    cout << "size: "<< content.size() << endl;

    /* Loop to print full content */
    /*
    cout << "content: "<< endl;
    for (const auto& subVec : content) {
        for (const auto& str : subVec) {
            std::cout << str << " ";
        }
        std::cout << std::endl;
    }
    */
    /* Loop to print content[0] */
    /*for (const auto& subVec : content) 
    {
        if (!subVec.empty()) 
        {
            cout << subVec[0] << endl;
        }
    }
    */

    uint32_t secs,nsecs; 
    for (const auto& subVec : content) 
    {
        //for (const auto& timestamp : subVec) 
        {
            uint64_t time = std::stoull(subVec[0]);
            secs = static_cast<uint32_t>(static_cast<uint64_t>(time) / static_cast<uint64_t>(1000000000));
            nsecs = static_cast<uint32_t>(static_cast<uint64_t>(time) % static_cast<uint64_t>(1000000000));
            //cout <<"timestamp: "<<subVec[0]<< " time read: "<<time<<" secs.nsecs: "<<secs<<"."<<nsecs<<endl;
            ros::Time rosTime(secs, nsecs);
            rosTimes.push_back(rosTime);
        }
    }
 
    //exit(0);
    const float T=1.0f/freq;
    ros::Duration d(T);

    cv::Mat im;
    cv_bridge::CvImage cvImage;

    vector<cv::KeyPoint> imLeftKeypts;//imRightKeypts;
    cv::Mat imLeftDescs;//imRightDescs;

    cv::FileStorage fskeyptsLeft(left_kptsndescs_dir+"/keypointsLeft_r.yml", cv::FileStorage::READ);
    cv::FileStorage fsdescsLeft(left_kptsndescs_dir+"/descriptorsLeft_r.yml", cv::FileStorage::READ);
    //cv::FileStorage fskeyptsRight(right_kptsndescs_dir+"/keypointsRight_r.yml", cv::FileStorage::READ);
    //cv::FileStorage fsdescsRight(right_kptsndescs_dir+"/descriptorsRight_r.yml", cv::FileStorage::READ);
    //cout << left_kptsndescs_dir+"/keypointsLeft_r.yml" << endl;
    //cout << fskeyptsLeft.isOpened() << endl;
    cv::FileNode kptLeftFileNode, descLeftFileNode;// kptRightFileNode, descRightFileNode;

    //std::vector<BagFromImages::KeyPoint> left_kpts_to_bag;
    //std::vector<BagFromImages::KeyPoint> right_kpts_to_bag;
    BagFromImages::KeyPoints left_kpts_to_bag;
    //BagFromImages::KeyPoints right_kpts_to_bag;
    // std::vector<BagFromImages::Descriptor> left_descs_to_bag;
    // std::vector<BagFromImages::Descriptor> right_descs_to_bag;
    BagFromImages::Descriptor left_descs_to_bag;
    //BagFromImages::Descriptor right_descs_to_bag;

    char *ptr,*ptr1; // declare a ptr pointer
    string str1;
    std::string filename;

    for(size_t i=0;i<left_filenames.size();i++)
    {
        if(!ros::ok())
            break;
    
        cout << "Processing: "<< left_filenames[i] << endl;

        //cout << "rosTimes[i]: "<< rosTimes[i] << endl;
        t = rosTimes[i];
        
        im = cv::imread(left_filenames[i],cv::IMREAD_UNCHANGED);
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::MONO16;
        cvImage.header.stamp = t;
        cvImage.header.frame_id = "0000" ; 

        bag_out.write("/cam0/image_raw",ros::Time(t),cvImage.toImageMsg());

        ptr = strtok((char*)(left_filenames[i]).c_str(), "/"); // use strtok() function to separate string using comma (,) delimiter.  
        while (ptr != NULL)  
        {  
            ptr1=ptr;
            ptr = strtok (NULL, "/");  
        }
        str1.assign(ptr1);
        filename.assign(str1.substr (0,str1.find(".png")));
        //cout << filename << endl;

        kptLeftFileNode = fskeyptsLeft["img"+filename];
        descLeftFileNode = fsdescsLeft["img"+filename];

        read(kptLeftFileNode, imLeftKeypts);

        read(descLeftFileNode, imLeftDescs);
        keypointsToROS(imLeftKeypts, left_kpts_to_bag);
        left_kpts_to_bag.header.stamp = t;
        left_kpts_to_bag.header.frame_id = "0000" ; 
        bag_out.write("/cam0/keypoints",ros::Time(t),left_kpts_to_bag);

        descriptorToROS(imLeftDescs, left_descs_to_bag, false); 

        left_descs_to_bag.header.stamp = t;
        left_descs_to_bag.header.frame_id = "0000" ;
        bag_out.write("/cam0/descriptors",ros::Time(t),left_descs_to_bag);
        #if 0
        cout << "Processing: "<< right_filenames[i] << endl;

        im = cv::imread(right_filenames[i],cv::IMREAD_UNCHANGED);
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::MONO16;
        cvImage.header.stamp = t;
        cvImage.header.frame_id = "0000"; 
        bag_out.write("/cam1/image_raw",ros::Time(t),cvImage.toImageMsg());
        

        ptr = strtok((char*)(right_filenames[i]).c_str(), "/"); // use strtok() function to separate string using comma (,) delimiter.  
        while (ptr != NULL)  
        {  
            ptr1=ptr;
            ptr = strtok (NULL, "/");  
        }
        str1.assign(ptr1);
        filename.assign(str1.substr (0,str1.find(".png")));
        //cout << filename << endl;

        kptRightFileNode = fskeyptsRight["img"+filename];
        descRightFileNode = fsdescsRight["img"+filename];
        read(kptRightFileNode, imRightKeypts);
        read(descRightFileNode, imRightDescs);
        keypointsToROS(imRightKeypts, right_kpts_to_bag);
        right_kpts_to_bag.header.stamp = t;
        right_kpts_to_bag.header.frame_id = "0000" ;

        bag_out.write("/cam1/keypoints",ros::Time(t),right_kpts_to_bag);

        descriptorToROS(imRightDescs, right_descs_to_bag, false); 
        right_descs_to_bag.header.stamp = t;
        right_descs_to_bag.header.frame_id = "0000" ;
        bag_out.write("/cam1/descriptors",ros::Time(t),right_descs_to_bag);
        #endif
        //t+=d;
        cout << i+1 << " / " << left_filenames.size() << endl;

        //exit(0);
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
