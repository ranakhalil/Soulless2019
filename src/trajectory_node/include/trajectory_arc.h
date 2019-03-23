#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
#include <math.h>

using namespace std;
using namespace cv;

class TrajectoryArc {
    public:
        TrajectoryArc();
    private:
        float steering_theta = 0.0;
        float horizon = 0.0;
        float height = 0.0;
        float width = 0.0;
        bool _params_initialized = false;
        float alpha = 0.2;
        ros::NodeHandle nodeHandle_;
        ros::Publisher  steeringPublisher_;
        cv_bridge::CvImagePtr cv_ptr;
        
        /* Functions */
	    int is_red_pixel(cv::Mat image, int x, int y);
	    int is_green_pixel(cv::Mat image, int x, int y);
	    vector<float> softmax(vector<float> x);
        int center_trajectories(cv::Mat image, int r, bool visualize);
	    int right_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize);
	    int left_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize);
        void callback(const sensor_msgs::ImageConstPtr& msg);
        int dot(vector<float> v_a, vector<float> v_b);
};
