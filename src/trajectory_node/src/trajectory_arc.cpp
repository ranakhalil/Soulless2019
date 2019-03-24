#include "trajectory_arc.h"

vector<int> TRAJECTOR_PIXELS = {10711, 12315, 13894, 9520, 13894, 12315, 10711};
vector<float> STEERING_RATIOS = {-1.0, -0.6, -0.3, 0, 0.3, 0.6, 1.0};
vector<int> R = {100, 150, 200};

TrajectoryArc::TrajectoryArc()
{
    this->steeringPublisher_ = this->nodeHandle_.advertise<std_msgs::Float64>("steering", 1);
    this->steeringTrajectoryPublisher_ = this->nodeHandle_.advertise<sensor_msgs::Image>("trajectory_image", 1);
    this->segmentedImage_ = this->nodeHandle_.subscribe("segmented_image", 1, &TrajectoryArc::callback, this);
}

int TrajectoryArc::is_red_pixel(cv::Mat image, int x, int y)
{
	if(image.at<Vec3b>(y, x)[0] == 0 && image.at<Vec3b>(y, x)[1] == 0 && image.at<Vec3b>(y, x)[2] == 255)
		return 1;
	return 0;
}

int TrajectoryArc::is_green_pixel(cv::Mat image, int x, int y)
{
	return (image.at<Vec3b>(y, x)[0] == 0 && image.at<Vec3b>(y, x)[1] == 255 && image.at<Vec3b>(y, x)[2] == 0);
}

float TrajectoryArc::dot(vector<float> v_a, vector<float> v_b) 
{
    float product = 0.0;
    for (int i = 0; i < v_a.size(); i++)
    {
        product = product + (v_a[i] * v_b[i]);
    }
    return product;
}

vector<float> TrajectoryArc::softmax(vector<float> x)
{
    vector<float> softmax_x;
    float x_sum = 0.0;

	for(float value : x)
	{
		x_sum += exp(value);
	}
    for(float value : x)
	{
		softmax_x.push_back(exp(value)/x_sum);
	}
    return softmax_x;
}

int TrajectoryArc::center_trajectories(cv::Mat image, int r, bool visualize=false)
{
    int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
    int horizon = height * 0.4;

    for(int y = height - 50; y > horizon; y--)
    {
        int xL = (int) (ceil( (float)width / 2.0 ) - r);
        int xR = (int) (ceil( (float)width / 2.0 ) + r);

        for(int x = xL; x < xR; x++)
        {
            red_pixel_count += + is_red_pixel(image,x,y);
            if(visualize && is_red_pixel(image,x,y) == 1)
            {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
            }
            else if(is_green_pixel(image, x, y))
            {
                return red_pixel_count;
            }
        }
    }
    
    return red_pixel_count;
}


int TrajectoryArc::right_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize=false)
{

	int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
    Mat cloned_image;
    int horizon = height * 0.4;

	for(int y = height-50; y > horizon; y--)
	{
	    int xL = width;
	    int xR = width;
	    if( ( R + r ) * ( R + r )-( y - height ) * ( y - height) >= 0 )
        {
             xL = (int) ((ceil( (float)width / 2.0  )+(R-r)) - sqrt(( R + r ) * ( R + r ) - ( y-height ) * (y - height)));
        }
           
        if ( (R-r) * (R-r)-( y - height ) * ( y - height ) >= 0)
        {
            xR = (int) ((ceil( width /2. )+(R+r)) - sqrt((R-r)*(R-r)-( y-height )*( y-height )));
        }
            
        xL = max( min( xL, width ), 0 );
        xR = max( min( xR, width ), 0 );
        int x_count = 0;    
	    for(int x = xL; x < xR; x++)
	    {
	        red_pixel_count += is_red_pixel(image,x,y);
	        if(visualize && is_red_pixel(image,x,y))
	        {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
	        }
	        else if(is_green_pixel(image,x,y)) 
            {
                if (x_count < LTolerance) 
                {
                    return red_pixel_count;
                }
                else {
                    x_count++;
                    break;
                }
            }
	    }
	}

	return red_pixel_count;
}

int TrajectoryArc::left_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize=true)
{
	int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
    int horizon = height * 0.4;
    
	for(int y = height-50; y > horizon; y--)
	{
	    int xL = 0;
	    int xR = 0;
	    if (( R - r ) * ( R - r )-( y - height ) * ( y - height) >= 0)
        {
            xL = (int) ((ceil( (float) width / 2.0 )-(R+r)) + sqrt(( R - r ) * ( R - r ) - ( y-height ) * (y - height)));
        }
        if ( (R+r) * (R+r)-( y - height ) * ( y - height ) >= 0 )
        {
             xR = (int) ((ceil( (float) width / 2.0 )-(R-r) ) + sqrt((R+r)*(R+r)-( y-height ) * ( y- height )));
        }
        xL = max( min( xL, width ), 0 );

        xR = max( min( xR, width ), 0 );
        if(xR == xL) continue;
        int x_count = 0;
	    for(int x = xR; x > xL; x--)
	    {
	        red_pixel_count += is_red_pixel(image,x,y);
	        if(visualize && is_red_pixel(image,x,y))
	        {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
	        }
	        else if(is_green_pixel(image,x,y)) 
            {
                if (x_count < LTolerance) 
                {
                    return red_pixel_count;
                }
                else {
                    x_count++;
                    break;
                }
            }
	    }
	}
    return red_pixel_count;
}

void TrajectoryArc::callback(const sensor_msgs::ImageConstPtr& msg) {
        vector<float> results(7);
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (visualize_)
        {
            cloned_image_ = cv_ptr->image.clone();
            results[0] = (float)this->left_trajectories(cv_ptr->image, R[0], 10, 10, true);
            results[1] = (float)this->left_trajectories(cv_ptr->image, R[1], 10, 10, true);
            results[2] = (float)this->left_trajectories(cv_ptr->image, R[2], 10, 10, true);
            results[3] = (float)this->center_trajectories(cv_ptr->image, 20, true);
            results[4] = (float)this->right_trajectories(cv_ptr->image, R[2], 10, 10, true);
            results[5] = (float)this->right_trajectories(cv_ptr->image, R[1], 10, 10, true);
            results[6] = (float)this->right_trajectories(cv_ptr->image, R[0], 10, 10, true);
        }
        else 
        {
            results[0] = (float)this->left_trajectories(cv_ptr->image, R[0], 10, 10, false);
            results[1] = (float)this->left_trajectories(cv_ptr->image, R[1], 10, 10, false);
            results[2] = (float)this->left_trajectories(cv_ptr->image, R[2], 10, 10, false);
            results[3] = (float)this->center_trajectories(cv_ptr->image, 20, false);
            results[4] = (float)this->right_trajectories(cv_ptr->image, R[2], 10, 10, false);
            results[5] = (float)this->right_trajectories(cv_ptr->image, R[1], 10, 10, false);
            results[6] = (float)this->right_trajectories(cv_ptr->image, R[0], 10, 10, false);
        }
        
        for(int i=0; i < 7; i++) ROS_ERROR("results[%d] %.3f", i, results[i]);
        for(int i=0; i < 7; i++) {
            results[i] /= (float)TRAJECTOR_PIXELS[i];
        }
        
        results = this->softmax(results);
        float steeringDotProduct = this->dot(results, STEERING_RATIOS);
        this->steering_theta = this->alpha * this->steering_theta + (1 - this->alpha) * steeringDotProduct;
        std_msgs::Float64 steering_msg;
        steering_msg.data = max( min( (float)(450 * this->steering_theta), (float) 450.0), (float)-450.0);
        this->steeringPublisher_.publish(steering_msg);
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
            std_msgs::Header(), 
            sensor_msgs::image_encodings::BGR8, 
            this->cloned_image_
        ).toImageMsg();

        this->steeringTrajectoryPublisher_.publish(img_msg);
    }

    int main(int argc, char* argv[])
    {
        ros::init(argc, argv, "trajectory_node");
	    TrajectoryArc node = TrajectoryArc();
        ros::spin();
	    return 0;
    }
