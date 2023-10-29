#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

int main()
{
	int desc_size = 0;
	//Mat img = imread("simple.jpeg",0);
	Mat img = imread("1520530308199447626.png",0);

        //Mat descriptors_orb;
        Ptr<ORB> orb = ORB::create();
        std::vector< KeyPoint > keypoints_orb;
        orb->detect(img, keypoints_orb, noArray());
            //cout << keypoints_orb.size() << endl;
        for (KeyPoint i: keypoints_orb)
            //cout << i.pt << ' ';
            {
                //string str(i);
                //cout << i.pt << endl;
            }
            //cout << i << endl;
        Mat descriptors_orb = cv::Mat(keypoints_orb.size(), 32, CV_8U);
        orb->compute(img, keypoints_orb, descriptors_orb);
            //cout << descriptors_orb.size() << ' ';
            //cout << descriptors_orb << ' ';

        //FileStorage fskeypts("keypoints.yml", FileStorage::APPEND);
        FileStorage fskeypts("keypoints.yml", FileStorage::WRITE);
        FileStorage fsdescs("descriptors.yml", FileStorage::WRITE);
        //write( fskeypts , "simple", keypoints_orb );
	//string x = std::to_string(99);
        //write( fskeypts , std::to_string(99), keypoints_orb );
        write( fskeypts , "img"+to_string(99), keypoints_orb );
        write( fsdescs , "img99", descriptors_orb );
        fskeypts.release();
        fsdescs.release();
	/*
	img = imread("simple2.jpeg",0);
        orb->detect(img, keypoints_orb, noArray());
            cout << keypoints_orb.size() << endl;
        for (KeyPoint i: keypoints_orb)
            //cout << i.pt << ' ';
            {
                //string str(i);
                //cout << i.pt << endl;
            }
            //cout << i << endl;
        orb->compute(img, keypoints_orb, descriptors_orb);
            //cout << descriptors_orb.size() << ' ';
            //cout << descriptors_orb << ' ';

        //write( fs , "simple2", keypoints_orb );
        //fs.release();
	*/
	/*
	vector<KeyPoint> mykpts2;
	Mat mydescs2;
  	FileStorage fs2keypts("keypoints.yml", FileStorage::READ);
  	FileStorage fs2descs("descriptors.yml", FileStorage::READ);
  	FileNode kptFileNode = fs2keypts["simple"];
  	FileNode descFileNode = fs2descs["simple"];
  	read( kptFileNode, mykpts2 );
  	read( descFileNode, mydescs2 );
	

  	fs2keypts.release();
  	fs2descs.release();



        FileStorage fs3keypts("keypoints2.yml", FileStorage::WRITE);
        FileStorage fs3descs("descriptors2.yml", FileStorage::WRITE);
        write( fs3keypts , "simple", mykpts2 );
        write( fs3descs , "simple", mydescs2 );
        fs3keypts.release();
        fs3descs.release();
	*/

    	return 0;
}
