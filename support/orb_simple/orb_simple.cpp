#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions
#include "Thirdparty/DLib/FileFunctions.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::AKAZE> akaze;
	Mat descriptors_orb;
	//Ptr<ORB> orb = ORB::create();
        //orb = ORB::create(2000, 1.2,8,19,0,4,ORB::FAST_SCORE, 31, 5);
	orb = ORB::create(2000, 1.2,8,31,0,2,ORB::FAST_SCORE, 20, 2);
        akaze = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 256, 3, 0.0001f, 8, 8, KAZE::DIFF_PM_G2 );
        //akaze = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 256, 3, 0.001f, 8, 8, KAZE::DIFF_PM_G2 );
	
	std::vector< KeyPoint > keypoints_orb;

    	std::string img_dir = argv[1];
    	std::string file_ext = argv[2];
	cout << img_dir << endl;
	cout << file_ext << endl;
	
	// Vector of paths to image
    	vector<string> img_filenames = DUtils::FileFunctions::Dir(img_dir.c_str(), file_ext.c_str(), true);

    	cout << "Images: " << img_filenames.size() << endl;

	for(size_t i=0;i<img_filenames.size();i++)
	{
		cout << "Processing: "<< img_filenames[i] << endl;
		Mat img = imread(img_filenames[i],IMREAD_GRAYSCALE);
		orb->detectAndCompute(img,noArray(), keypoints_orb, descriptors_orb,false);
		//akaze->detectAndCompute(img,noArray(), keypoints_orb, descriptors_orb,false);
		cv::FileStorage fskpts("keypoints.yml", cv::FileStorage::APPEND);
        	cv::FileStorage fsdescs("descriptors.yml", cv::FileStorage::APPEND);
        	write( fskpts , "img"+to_string(i+1), keypoints_orb );
        	write( fsdescs , "img"+to_string(i+1), descriptors_orb );
        	fskpts.release();
        	fsdescs.release();
	}

    return 0;
}
