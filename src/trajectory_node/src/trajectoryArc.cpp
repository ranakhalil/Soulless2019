#include "trajectoryArc.h"

// TRAJECTOR_PIXELS = [10711, 12315, 13894, 9520, 13894, 12315, 10711]
// STEERING_RATIOS = [-1.0, -0.6, -0.3, 0, 0.3, 0.6, 1.0]
// R = [100, 150, 200]

TrajectoryArc::TrajectoryArc() {
        // rospy.init_node('trajectory_arc_node', anonymous= True )
        // self.visualize = rospy.get_param('~visualize')
        // self.time_since_last_update = rospy.get_rostime().secs
        // self.bridge = CvBridge()
        // self.steeringTwistPub = rospy.Publisher('/steering', Float64, queue_size=1)
        // self.imageTrajectoryPub = rospy.Publisher('/trajectory_image', Image, queue_size=1)
        // rospy.Subscriber('/segmented_image', Image, self.callback, queue_size=1, buff_size=2**24)
        // rospy.spin()
}

int is_red_pixel(vector<vector<vector<float>>> image, int x, int y)
{
	if(image[y][x][0] == 0 and image[y][x][1] == 0 and image[y][x][2] == 255)
		return 1;
	return 0;
}

int is_green_pixel(vector<vector<vector<float>>> image, int x, int y)
{
	return (image[y][x][0] == 0 and image[y][x][1] == 255 and image[y][x][2] == 0);
}

vector<float> softmax(vector<float> x)
{
    vector<float> softmax_x;
    x_sum = 0;
	for(float value : x)
	{
		x_sum += exp(value)
	}
    for(float value : x)
	{
		softmax_x.push_back(exp(value)/x_sum);
	}
    return softmax_x;
}

int center_trajectories(vector<vector<vector<float>>> image, int r, bool visualize=true)
{

    int red_pixel_count = 0;
    int height = image.size();
    int width = image[0].size();
    int horizion = height*.4;
  
    for(int y = height-50; y > horizion; y--)
    {
        xL = int((ceil( (float)width / 2 ) -r))
        xR = int((ceil( (float)width / 2 ) +r))
        for(int x = xL; x < xR; x++)
        {
            red_pixel_count += is_red_pixel(image,x,y);
            if(visualize and is_red_pixel(image,x,y)==1)
            {
            	image[y][x][0] = 255;
            	image[y][x][1] = 255;
            	image[y][x][2] = 255;
            }
            else if(is_green_pixel(image,x,y))
            	return red_pixel_count;
        }
    }
    return red_pixel_count;
}
