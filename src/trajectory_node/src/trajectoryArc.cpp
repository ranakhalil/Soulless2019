#include "trajectoryArc.h"

vector<int> TRAJECTOR_PIXELS = {10711, 12315, 13894, 9520, 13894, 12315, 10711}
vector<float> STEERING_RATIOS = {-1.0, -0.6, -0.3, 0, 0.3, 0.6, 1.0}
vector<int> R = {100, 150, 200}

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

int TrajectoryArc::is_red_pixel(vector<vector<vector<float>>> image, int x, int y)
{
	if(image[y][x][0] == 0 && image[y][x][1] == 0 && image[y][x][2] == 255)
		return 1;
	return 0;
}

int TrajectoryArc::is_green_pixel(vector<vector<vector<float>>> image, int x, int y)
{
	return (image[y][x][0] == 0 && image[y][x][1] == 255 && image[y][x][2] == 0);
}

int TrajectoryArc::dot(vector<float> v_a, vector<float> v_b) 
{
    float product = 0.0;
    for (int i = 0; i < v_a.size() -1; i++)
    {
        product = product + (v_a[i] * v_b[i]);
    }
    return product;
}

vector<float> TrajectoryArc::softmax(vector<float> x)
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

int TrajectoryArc::center_trajectories(vector<vector<vector<float>>> image, int r, bool visualize=true)
{

    int red_pixel_count = 0;
    int height = image.size();
    int width = image[0].size();
    int horizon = height * 0.4;

    for(int y = height - 50; y > horizon; y--)
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
            {
                return red_pixel_count;
            }
        }
    }
    
    return red_pixel_count;
}


int TrajectoryArc::right_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true)
{

	int red_pixel_count = 0;
	int height = image.size();
	int width = image[0].size();
	int horizion = height * 0.4;
	
	for(int y = height-50; y > horizion; y--)
	{
	    xL = width;
	    xR = width;
	    if ( R + r ) * ( R + r )-( y - height ) * ( y - height) >= 0 :
            xL = int((ceil( (float)width / 2. )+(R-r))-math.sqrt(( R + r ) * ( R + r ) - ( y-height )*(y - height)))
        if (R-r) * (R-r)-( y - height ) * ( y - height ) >= 0:
            xR = int((ceil( width /2. )+(R+r))-math.sqrt((R-r)*(R-r)-( y-height )*( y-height )))
        xL = max( min( xL, width ), 0 )
        xR = max( min( xR, width ), 0 )
        x_count = 0
	    for(int x = xL; x < xR; x++)
	    {
	        red_pixel_count += is_red_pixel(image,x,y);
	        if(visualize and is_red_pixel(image,x,y)==1)
	        {
	        	image[y][x][0] = 255;
	        	image[y][x][1] = 255;
	        	image[y][x][2] = 255;
	        }
	        else if(is_green_pixel(image,x,y)) {
                if (x_count < LTolerance) return red_pixel_count;
                else break;
            }
	    }
	}
	return red_pixel_count;
}

int TrajectoryArc::left_trajectories(vector<vector<vector<float>>> image, int R, int r, int LTolerance, bool visualize=true)
{

	int red_pixel_count = 0;
	int height = image.size();
	int width = image[0].size();
	int horizion = height * 0.4;
	
	for(int y = height-50; y > horizion; y--)
	{
	    int xL = 0;
	    int xR = 0;
	    if ( R - r ) * ( R - r )-( y - height ) * ( y - height) >= 0 :
            xL = int((ceil( (float)width / 2.0 )-(R+r))-math.sqrt(( R - r ) * ( R - r ) - ( y-height )*(y - height)))
        if (R+r) * (R+r)-( y - height ) * ( y - height ) >= 0:
            xR = int((ceil( self.width /2. )+(R-r))-math.sqrt((R+r)*(R+r)-( y-height )*( y-height )))
        xL = max( min( xL, width ), 0 )
        xR = max( min( xR, width ), 0 )
        x_count = 0
	    for(int x = xR-1; x < xL; x--)
	    {
	        red_pixel_count += is_red_pixel(image,x,y);
	        if(visualize & is_red_pixel(image,x,y)==1)
	        {
	        	image[y][x][0] = 255;
	        	image[y][x][1] = 255;
	        	image[y][x][2] = 255;
	        }
	        else if(is_green_pixel(image,x,y)) {
                if (x_count < LTolerance) return red_pixel_count;
                else break;
            }
	    }
	}
	return red_pixel_count;
}

void TrajectoryArc::callback(const sensor_msgs::ImageConstPtr& msg) {
        vector<float> results = vector<int>(7);
        float steeringDotProduct = 0.0;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        results[0] = (float)this->left_trajectories(cv_ptr->image, R[0], 10, 10, False);
        results[1] = (float)this->left_trajectories(cv_ptr->image, R[1], 10, 10, False);
        results[2] = (float)this->left_trajectories(cv_ptr->image, R[2], 10, 10, False);
        results[3] = (float)this->center_trajectories(cv_ptr->image, 20, False);
        results[4] = (float)this->right_trajectories(cv_ptr->image, R[2], 10, 10, False);
        results[5] = (float)this->right_trajectories(cv_ptr->image, R[1], 10, 10, False);
        results[6] = (float)this->right_trajectories(cv_ptr->image, R[0], 10, 10, False);
        for(int i=0; i < 7; i++) {
            results[i] /= (float)TRAJECTOR_PIXELS[i];
        }
        
        results = this->softmax(results);
        steeringDotProduct = this->dot(results, STEERING_RATIOS);

        steeringDotProduct = this->dot(results, STEERING_RATIOS);
        this->steering = this->alpha * this->steering + (1 - this->alpha) * steeringDotProduct;




    }
