#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions

using namespace std;
using namespace cv;

const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 100; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames

namespace example {
class Tracker
{
public:
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
        detector(_detector),
        matcher(_matcher)
    {}

    void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
    Mat process(const Mat frame, Stats& stats);
    Ptr<Feature2D> getDetector() {
        return detector;
    }
protected:
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat first_frame, first_desc;
    vector<KeyPoint> first_kp;
    vector<Point2f> object_bb;
};

void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
{
    cv::Point *ptMask = new cv::Point[bb.size()];
    const Point* ptContain = { &ptMask[0] };
    int iSize = static_cast<int>(bb.size());
    for (size_t i=0; i<bb.size(); i++) {
        ptMask[i].x = static_cast<int>(bb[i].x);
        ptMask[i].y = static_cast<int>(bb[i].y);
    }
    first_frame = frame.clone();
    cv::Mat matMask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::fillPoly(matMask, &ptContain, &iSize, 1, cv::Scalar::all(255));
    detector->detectAndCompute(first_frame, matMask, first_kp, first_desc);
    stats.keypoints = (int)first_kp.size();
    drawBoundingBox(first_frame, bb);
    putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
    object_bb = bb;
    delete ptMask;
}

Mat Tracker::process(const Mat frame, Stats& stats)
{
    vector<KeyPoint> kp;
    Mat desc;
    detector->detectAndCompute(frame, noArray(), kp, desc);
    stats.keypoints = (int)kp.size();

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }
    stats.matches = (int)matched1.size();

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    if(matched1.size() >= 4) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }

    if(matched1.size() < 4 || homography.empty()) {
        Mat res;
        hconcat(first_frame, frame, res);
        stats.inliers = 0;
        stats.ratio = 0;
        return res;
    }
    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    stats.inliers = (int)inliers1.size();
    stats.ratio = stats.inliers * 1.0 / stats.matches;

    vector<Point2f> new_bb;
    perspectiveTransform(object_bb, new_bb, homography);
    Mat frame_with_bb = frame.clone();
    if(stats.inliers >= bb_min_inliers) {
        drawBoundingBox(frame_with_bb, new_bb);
    }
    Mat res;
    drawMatches(first_frame, inliers1, frame_with_bb, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    return res;
}
}

int main()
{
	int desc_size = 0;
	Mat img = imread("simple.jpeg",0);

	//Mat descriptors_orb;
	Ptr<ORB> orb = ORB::create();
	std::vector< KeyPoint > keypoints_orb;
	orb->detect(img, keypoints_orb, noArray());
	//    cout << keypoints.size() << endl;
	for (KeyPoint i: keypoints_orb)
	    //cout << i.pt << ' ';
	    cout << i.pt << endl;
	Mat descriptors_orb = cv::Mat(keypoints_orb.size(), 32, CV_8U);
	orb->compute(img, keypoints_orb, descriptors_orb);
	    cout << descriptors_orb.size() << ' ';
	    cout << descriptors_orb << ' ';
	orb->detectAndCompute(img,noArray(), keypoints_orb, descriptors_orb,true);
	    cout << descriptors_orb.size() << ' ';
	    cout << descriptors_orb << ' ';

	vector<KeyPoint>& keypoints = keypoints_orb;
	for (int i=0; i< keypoints.size();i++) 
		keypoints[i].class_id=0;
	//for (KeyPoint i: keypoints) 
	//	i.class_id=0;

	//Mat descriptors_akaze;
	//desc_size = orb->getDescriptorSize();
	//cout << "Desc size: "<< desc_size <<endl;
	Ptr<AKAZE> akaze = AKAZE::create();
	akaze->setDescriptorSize(256);
	//std::vector< KeyPoint > keypoints_akaze;
	//akaze->detect(img, keypoints_akaze, noArray());
	//    cout << keypoints.size() << endl;
	//for (KeyPoint i: keypoints_akaze)
	    //cout << i.pt << ' ';
	    //cout << i.pt << endl;
	//akaze->compute(img, keypoints_akaze, descriptors_akaze);
	Mat descriptors_akaze = cv::Mat(keypoints.size(), 32, CV_8U);
	cout<<"------------"<<endl;
	akaze->compute(img, keypoints, descriptors_akaze);
	    cout << descriptors_akaze.size() << ' ';
	    cout << descriptors_akaze << ' ';

	akaze->detectAndCompute(img, noArray(),keypoints, descriptors_akaze,true);
	    cout << descriptors_akaze.size() << ' ';
	    cout << descriptors_akaze << ' ';

	

    return 0;
}
