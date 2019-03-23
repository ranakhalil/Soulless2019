#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <vector>
using namespace std;

vector<int> TRAJECTOR_PIXELS = {10711, 12315, 13894, 9520, 13894, 12315, 10711}
vector<float> STEERING_RATIOS = {-1.0, -0.6, -0.3, 0, 0.3, 0.6, 1.0}
vector<int> R = {100, 150, 200}

class RosNode:
    public:
        RosNode();
    private:
        float steering_theta = 0.0
        float horizon = 0.0
        float height = 0.0
        float width = 0.0
        bool _params_initialized = false
        float alpha = 0.2
        cv_bridge::CvImagePtr cv_ptr;
    def __init__(self):
        rospy.init_node('trajectory_arc_node', anonymous= True )
        self.visualize = rospy.get_param('~visualize')
        self.time_since_last_update = rospy.get_rostime().secs
        self.bridge = CvBridge()
        self.steeringTwistPub = rospy.Publisher('/steering', Float64, queue_size=1)
        self.imageTrajectoryPub = rospy.Publisher('/trajectory_image', Image, queue_size=1)
        rospy.Subscriber('/segmented_image', Image, self.callback, queue_size=1, buff_size=2**24)
        rospy.spin()

	int is_red_pixel(vector<vector<vector<float>>> image, int x, int y);
	bool is_green_pixel(vector<vector<vector<float>>> image, int x, int y);
	vector<float> softmax(vector<float> x);
    int center_trajectories(vector<vector<vector<float>>> image, int r, bool visualize=true)

	int right_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true);

	int left_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true);

    void callback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::vector<float> results = std::vector<int>(7);
        results[0] = (float)this->left_trajectories(image, R[0], 10, 10, False);
        results[1] = (float)this->left_trajectories(image, R[1], 10, 10, False);
        results[2] = (float)this->left_trajectories(image, R[2], 10, 10, False);
        results[3] = (float)this->center_trajectories(image, 20, False);
        results[4] = (float)this->right_trajectories(image, R[2], 10, 10, False);
        results[5] = (float)this->right_trajectories(image, R[1], 10, 10, False);
        results[6] = (float)this->right_trajectories(image, R[0], 10, 10, False);
        for(int i=0; i<7; i++) {
            results[i] /= (float)TRAJECTOR_PIXELS[i];
        }
        
    }


    def callback(self, data):
        start = time.time()
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        shape = image.shape
        if not self._params_initialized:
            self.height = shape[0]
            self.width = shape[1] - 1
            self.horizon = int(self.height * 0.4)
            self._params_initialized = True
            rospy.loginfo("Image Params has been initialized")
        
        results = np.array( results, dtype = float ) / np.array( TRAJECTOR_PIXELS, dtype = float )
        results = self.softmax(results)

        results = np.dot(results, STEERING_RATIOS)

        self.steering_theta = self.alpha * self.steering_theta + (1 - self.alpha) * results

        # msg = Twist()
        # msg.linear.x = 0.0
        # msg.angular.x = self.steering_theta
        msg = Float64()
        msg.data = 450*self.steering_theta
        self.steeringTwistPub.publish(msg)
        
        if self.visualize:
            image_cpy = image.copy()
            self.left_trajectories(image_cpy, R[0], 10, 10)
            self.left_trajectories(image_cpy, R[1], 10, 10)
            self.left_trajectories(image_cpy, R[2], 10, 10)
            self.center_trajectories(image_cpy, 20)
            self.right_trajectories(image_cpy, R[2], 10, 10)
            self.right_trajectories(image_cpy, R[1], 10, 10)
            self.right_trajectories(image_cpy, R[0], 10, 10)
            self.imageTrajectoryPub.publish(self.bridge.cv2_to_imgmsg(image_cpy, "bgr8"))
        rospy.logerr(time.time() - start)

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
