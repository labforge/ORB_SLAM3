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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;
using namespace cv;

int main()
{
    vector<KeyPoint> mykptsLeft,mykptsRight;
    Mat mydescsLeft,mydescsRight;
    int imgcount = 0;
    vector<vector<string>> content;
    vector<string> row;
    string line, word;
    int i,j;
     
    fstream file ("data.csv", ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            row.clear();
             
            stringstream str(line);
             
            while(getline(str, word, ','))
                row.push_back(word);

            content.push_back(row);
        }
    }
    else
        cout<<"Could not open the file\n";

    cout << "size: "<< content.size() << endl;
    /* 
    for(i=0;i<content.size();i++)
    {
        //for(j=0;j<content[i].size();j++)
        for(j=0;j<content[i].size();j++)
        {
            cout<<content[i][j]<<" ";
        }
        cout<<"\n";
    }
    cout << "i:" << i << "j:" << j <<endl;
    */
    for(i=0;i<content.size();i++)
    {
        //cout<<"img"+content[i][0]<<endl;
    }
#if 1
    FileStorage fskeyptsLeft("keypointsLeft.yml", FileStorage::READ);
    FileStorage fsdescsLeft("descriptorsLeft.yml", FileStorage::READ);
    FileStorage fskeyptsRight("keypointsRight.yml", FileStorage::READ);
    FileStorage fsdescsRight("descriptorsRight.yml", FileStorage::READ);

    FileStorage fskeyptsLeft_r("keypointsLeft_r.yml", FileStorage::WRITE);
    FileStorage fsdescsLeft_r("descriptorsLeft_r.yml", FileStorage::WRITE);
    FileStorage fskeyptsRight_r("keypointsRight_r.yml", FileStorage::WRITE);
    FileStorage fsdescsRight_r("descriptorsRight_r.yml", FileStorage::WRITE);

    FileNode kptLeftFileNode, descLeftFileNode, kptRightFileNode, descRightFileNode;

    //for(imgcount=1;imgcount<=2821;imgcount++)
    for(imgcount=1;imgcount<=content.size();imgcount++)
    {
        //imgcount = 1;
        kptLeftFileNode = fskeyptsLeft["img"+std::to_string(imgcount)];
        descLeftFileNode = fsdescsLeft["img"+std::to_string(imgcount)];
        kptRightFileNode = fskeyptsRight["img"+std::to_string(imgcount)];
        descRightFileNode = fsdescsRight["img"+std::to_string(imgcount)];

        read(kptLeftFileNode, mykptsLeft);
        read(descLeftFileNode, mydescsLeft);
        read(kptRightFileNode, mykptsRight);
        read(descRightFileNode, mydescsRight);

        //cout<< mykptsLeft << endl;
       
        //cout  << "img" + content[imgcount-1][0] << endl; 
        write(fskeyptsLeft_r , "img"+ content[imgcount-1][0], mykptsLeft);
        write(fsdescsLeft_r ,"img"+ content[imgcount-1][0], mydescsLeft);
        write(fskeyptsRight_r ,"img"+ content[imgcount-1][0], mykptsRight);
        write(fsdescsRight_r ,"img"+ content[imgcount-1][0], mydescsRight);
    }

    fskeyptsLeft.release();
    fsdescsLeft.release();
    fskeyptsRight.release();
    fsdescsRight.release();

    fskeyptsLeft_r.release();
    fsdescsLeft_r.release();
    fskeyptsRight_r.release();
    fsdescsRight_r.release();
#endif
#if 0
    vector<cv::KeyPoint> imLeftKeypts,imRightKeypts;
    cv::Mat imLeftDescs,imRightDescs;

    cv::FileStorage fskeyptsLeft("keypointsLeft_r.yml", cv::FileStorage::READ);
    cv::FileStorage fsdescsLeft("descriptorsLeft_r.yml", cv::FileStorage::READ);
    cv::FileStorage fskeyptsRight("keypointsRight_r.yml", cv::FileStorage::READ);
    cv::FileStorage fsdescsRight("descriptorsRight_r.yml", cv::FileStorage::READ);
    
    cv::FileNode kptLeftFileNode, descLeftFileNode, kptRightFileNode, descRightFileNode;
  
 
    //for(imgcount=1;imgcount<=2821;imgcount++)
    imgcount = 1;
    {

        kptLeftFileNode = fskeyptsLeft["img"+ content[imgcount-1][0]];
        descLeftFileNode = fsdescsLeft["img"+ content[imgcount-1][0]];
        kptRightFileNode = fskeyptsRight["img"+ content[imgcount-1][0]];
        descRightFileNode = fsdescsRight["img"+ content[imgcount-1][0]];

        read(kptLeftFileNode, imLeftKeypts);
        read(descLeftFileNode, imLeftDescs);
        read(kptRightFileNode, imRightKeypts);
        read(descRightFileNode, imRightDescs);

	for (cv::KeyPoint i: imLeftKeypts)
	{
                cout << i.angle << endl;
                cout << i.response << endl;
                cout << i.octave << endl;
                cout << i.class_id << endl;
		cout << "***********"<<endl;
	}
    }

#endif

    return 0;
}
