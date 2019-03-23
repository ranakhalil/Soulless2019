#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <vector>
using namespace std;

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
	    int is_red_pixel(vector<vector<vector<float>>> image, int x, int y);
	    bool is_green_pixel(vector<vector<vector<float>>> image, int x, int y);
	    vector<float> softmax(vector<float> x);
        int center_trajectories(vector<vector<vector<float>>> image, int r, bool visualize=true)
	    int right_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true);
	    int left_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true);
        void callback(const sensor_msgs::ImageConstPtr& msg);
        int dot(vector<float> v_a, vector<float> v_b);
};

