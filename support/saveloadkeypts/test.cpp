#include <thread>
#include <opencv2/opencv.hpp>
int monoLeft, monoRight;
using namespace std;

class Frame
{
public:
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
    {
        vector<int> vLapping = {x0,x1};
        if(flag==0)
        {
            monoLeft += 1;
        }
        else
        {
            monoRight += 1;
        }
    }
};

class test
{

public:
int main()
{
    cv::Mat imLeft;
    std::thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
    return 0;
}
};

