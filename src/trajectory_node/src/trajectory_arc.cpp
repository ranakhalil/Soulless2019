#include "trajectory_arc.h"
#include <vector>

// vector<int> TRAJECTOR_PIXELS = {10711, 12315, 13894, 9520, 13894, 12315, 10711};
vector<int> TRAJECTOR_PIXELS = {0, 0, 0, 0, 0, 0, 0};
vector<float> STEERING_RATIOS = {-.7, -.9, -1.2, 0, 1.2, .9, .7};
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

int TrajectoryArc::is_cone_or_lane(cv::Mat image, int x, int y)
{
	return (image.at<Vec3b>(y, x)[0] == 0 && (image.at<Vec3b>(y, x)[1] == 255 ^ image.at<Vec3b>(y, x)[2] == 255));
}

void TrajectoryArc::EraseCone(cv::Mat& image, int y, int x)
{
	if((x > 0 && x < image.size().width) && (y > 0 && y < image.size().height))
	{

		// if pixel is blue(cone)
		if(image.at<Vec3b>(y, x)[0] == 0 && image.at<Vec3b>(y, x)[1] == 0 && image.at<Vec3b>(y, x)[2] == 255)
		{
			image.at<Vec3b>(y, x)[2] = 0;
			vector<int> delta = {-1,0,1};
			for(int dy : delta)
				for(int dx : delta)
					EraseCone(image,y+dy,x+dx);
		}
	}

}

std::vector<Pos> TrajectoryArc::cone_pos(cv::Mat image)
{
	cv::Mat cone_image = image.clone();
	int height = image.size().height;
    int width = image.size().width;

    std::vector<Pos> cones;

    for(int y = height; y < horizon; y--)
    {
    	for(int x = 0; x < width; x++)
    	{
    		// if pixel is blue(part of cone)
			if(image.at<Vec3b>(y, x)[0] == 0 && image.at<Vec3b>(y, x)[1] == 0 && image.at<Vec3b>(y, x)[2] == 255)
			{
				Pos pos(y,x);
				cones.push_back(pos);
				// erase adjacent blue pixel(erase that cone)
				EraseCone(cone_image,y,x);
			}
    	}
    }
    return cones;
}

void TrajectoryArc::cone_lines(cv::Mat image)
{
	std::vector<Pos> cones = cone_pos(image);
	int cone0 = 0;
	for(int cone1 = 1; cone1 < cones.size(); cone1++)
	{
		line(image,Point(cones[cone0].x,cones[cone0].y),Point(cones[cone1].x,cones[cone1].y),Scalar(0,255,0),8);
		cone0 = cone1;
	}

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
            red_pixel_count += is_red_pixel(image,x,y);
            if(visualize && is_red_pixel(image,x,y) == 1)
            {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
            }
            else if(is_cone_or_lane(image, x, y))
            {
                return red_pixel_count;
            }
        }
    }
    
    return red_pixel_count;
}

int TrajectoryArc::center_trajectories_count(cv::Mat image, int r)
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

        for(int x = xL; x < xR; x++) red_pixel_count += 1;
    }
    
    return red_pixel_count;
}

int TrajectoryArc::right_trajectories_count(cv::Mat image, int R, int r)
{

	int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
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
	    for(int x = xL; x < xR; x++)  red_pixel_count += 1;
	}

	return red_pixel_count;
}


int TrajectoryArc::right_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize=false)
{

	int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
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
	    for(int x = xL; x < xR; x++) {
	        x_count += is_red_pixel(image,x,y);
	        if(visualize && is_red_pixel(image,x,y))
	        {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
	        }
	        else if(is_cone_or_lane(image,x,y)) {
                if (x_count < LTolerance) {
	                red_pixel_count += x_count;
                    return red_pixel_count;
                } else {
                    break;
                }
            }
            
	    }
	    red_pixel_count += x_count;
	}

	return red_pixel_count;
}

int TrajectoryArc::left_trajectories(cv::Mat image, int R, int r, int LTolerance, bool visualize=true) {
    int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
    int horizon = height * 0.4;
    
    for(int y = height-50; y > horizon; y--) {
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
	    for(int x = xR; x > xL; x--) {
	        x_count += is_red_pixel(image,x,y);
	        if(visualize && is_red_pixel(image,x,y))
	        {
            	this->cloned_image_.at<Vec3b>(y, x)[0] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[1] = 255;
            	this->cloned_image_.at<Vec3b>(y, x)[2] = 255;
	        }
	        else if(is_cone_or_lane(image,x,y)) {
                if (x_count < LTolerance) {
	                red_pixel_count += x_count;
                    return red_pixel_count;
                } else {
                    break;
                }
            }
            
	    }
	    red_pixel_count += x_count;
	}
    return red_pixel_count;
}

int TrajectoryArc::left_trajectories_count(cv::Mat image, int R, int r) {
    int red_pixel_count = 0;
    cv::Size size = image.size();
    int height = size.height;
    int width = size.width;
    int horizon = height * 0.4;
    
    for(int y = height-50; y > horizon; y--) {
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
	    for(int x = xR; x > xL; x--) red_pixel_count += 1;
	    red_pixel_count += x_count;
	}
    return red_pixel_count;
}

void TrajectoryArc::callback(const sensor_msgs::ImageConstPtr& msg) {
        vector<float> results(7);
        std::vector<Pos> current_cones;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat image = cv_ptr->image;
        if (TRAJECTOR_PIXELS[0] == 0) {
            TRAJECTOR_PIXELS[0] = (float)this->left_trajectories_count(image, R[0], 10);
            TRAJECTOR_PIXELS[1] = (float)this->left_trajectories_count(image, R[1], 10);
            TRAJECTOR_PIXELS[2] = (float)this->left_trajectories_count(image, R[2], 10);
            TRAJECTOR_PIXELS[3] = (float)this->center_trajectories_count(image, 20);
            TRAJECTOR_PIXELS[4] = (float)this->right_trajectories_count(image, R[2], 10);
            TRAJECTOR_PIXELS[5] = (float)this->right_trajectories_count(image, R[1], 10);
            TRAJECTOR_PIXELS[6] = (float)this->right_trajectories_count(image, R[0], 10);
        }

        if (visualize_)
        {
            this->cloned_image_ = image.clone();
            results[0] = (float)this->left_trajectories(image, R[0], 10, 10, true);
            results[1] = (float)this->left_trajectories(image, R[1], 10, 10, true);
            results[2] = (float)this->left_trajectories(image, R[2], 10, 10, true);
            results[3] = (float)this->center_trajectories(image, 20, true);
            results[4] = (float)this->right_trajectories(image, R[2], 10, 10, true);
            results[5] = (float)this->right_trajectories(image, R[1], 10, 10, true);
            results[6] = (float)this->right_trajectories(image, R[0], 10, 10, true);
            current_cones = this->cone_pos(image);
        }
        else 
        {
            results[0] = (float)this->left_trajectories(image, R[0], 10, 10, false);
            results[1] = (float)this->left_trajectories(image, R[1], 10, 10, false);
            results[2] = (float)this->left_trajectories(image, R[2], 10, 10, false);
            results[3] = (float)this->center_trajectories(image, 20, false);
            results[4] = (float)this->right_trajectories(image, R[2], 10, 10, false);
            results[5] = (float)this->right_trajectories(image, R[1], 10, 10, false);
            results[6] = (float)this->right_trajectories(image, R[0], 10, 10, false);
            current_cones = this->cone_pos(image);
        }

        ROS_INFO("Cones Found : %d ", current_cones.size());
        
        for(int i=0; i < 7; i++) ROS_ERROR("results[%d] %.3f", i, results[i]);
        for(int i=0; i < 7; i++) {
            results[i] /= (float)TRAJECTOR_PIXELS[i];
        }
        
        results = this->softmax(results);
        float steeringDotProduct = this->dot(results, STEERING_RATIOS);
        this->steering_theta = this->alpha * this->steering_theta + (1 - this->alpha) * steeringDotProduct;
        std_msgs::Float64 steering_msg;
        steering_msg.data = max( min( (float)(450 * this->steering_theta), (float) 450.0), (float)-450.0);
        
        this->steering_theta *= 3;
        cv::Size size = this->cloned_image_.size();
        int height = size.height;
        int width = size.width;
        arrowedLine(this->cloned_image_, Point(width/2, height), Point(width/2 + 100*sin(3.14159/9*this->steering_theta), height - 100*cos(3.14159/9*this->steering_theta)), Scalar(0,0,0), 20);
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
