#define DEPTH_WINDOW_NAME "DEPTH"
#if 0
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#else
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
#endif
#include "libsynexens3/libsynexens3.h"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/msg/int32.hpp"

using namespace sensor_msgs;
using namespace image_transport;
using namespace std;

class PublisherNode : public rclcpp::Node
{
public:



explicit PublisherNode(const rclcpp::NodeOptions & options)
  : Node("cs20", options) 
{
    config();
   //  image_transport::ImageTransport it;
    
    //pub = it.advertise("camera/image", 1);
    
    //pub = image_transport::create_publisher(PublisherNode, "camera/image");

    timer_ = create_wall_timer(
    20, std::bind(&PublisherNode::timer_callback, this));
}

void config(){

    sy3::sy3_error e;
	printf("version:%s \n", sy3::sy3_get_version(e));
	sy3::context *ctx = sy3::sy3_create_context(e);
	sy3::device *dev = ctx->query_device(e);
	if (e != sy3::sy3_error::SUCCESS)
	{
		printf("error:%s \n", sy3::sy3_error_to_string(e));
		
	}
	pline = sy3::sy3_create_pipeline(ctx, e);
	sy3::config *cfg = sy3_create_config(e);

	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, e);

	pline->start(cfg, e);
	bool quit = false;
	
	int switch_flag = 1;
	g_is_start = true;
   

}

void timer_callback()
{
    
	sy3::sy3_error e;
	sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
	sy3::depth_frame *depth_frame = frameset->get_depth_frame();
		
	if (depth_frame == nullptr){
		//	printf("depth_frame:empty \n");
	}
	else{
		show_depth_frame(depth_frame, DEPTH_WINDOW_NAME);		
	}

	delete frameset;
	nIndex++;

}

void show_depth_frame(sy3::depth_frame *frame, const char *name)
{
	if (frame)
	{
		g_frame_count++;

        cv::Mat depth_frame_buffer_mat(frame->get_height(),
         frame->get_width(), CV_16UC1, frame->get_data());

        sensor_msgs::ImagePtr& depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();
        
        std::string frame = "tf2" + "_frame";
        rclcpp::Time time = rclcpp::Clock().now();

		depth_image->header.stamp = time;
        depth_image->header.frame_id = frame;
        
        camera_info.header.frame_id = frame;
        camera_info.header.stamp = time;
        sy3::sy3_intrinsics intrinsics = frame->get_profile()->get_intrinsics();

        camera_info.width = intrinsics.width;
        camera_info.height = intrinsics.height;

        camera_info.distortion_model = "plumb_bob";

          // The distortion parameters, size depending on the distortion model.
        // For "plumb_bob", the 5 parameters are: (k1, k2, k3, k4, k5).
        camera_info.r = {intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
                        intrinsics.coeffs[3], intrinsics.coeffs[4]};
        // clang-format off
        // Intrinsic camera matrix for the raw (distorted) images.
        //     [fx  0 cx]
        // K = [ 0 fy cy]
        //     [ 0  0  1]
        // Projects 3D points in the camera coordinate frame to 2D pixel
        // coordinates using the focal lengths (fx, fy) and principal point
        // (cx, cy).
        camera_info.k = {intrinsics.fx,  0.0f,            intrinsics.ppx,
                        0.0f,           intrinsics.fy,   intrinsics.ppy,
                        0.0f,           0.0,                       1.0f};
        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        // By convention, this matrix specifies the intrinsic (camera) matrix
        //  of the processed (rectified) image. That is, the left 3x3 portion
        //  is the normal camera intrinsic matrix for the rectified image.
        // It projects 3D points in the camera coordinate frame to 2D pixel
        //  coordinates using the focal lengths (fx', fy') and principal point
        //  (cx', cy') - these may differ from the values in K.
        // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
        //  also have R = the identity and P[1:3,1:3] = K.
        camera_info.p = {intrinsics.fx,   0.0f,            intrinsics.ppx,   0.0f,
                        0.0f,            intrinsics.fy,   intrinsics.ppy,   0.0f,
                        0.0f,            0.0,                       1.0f,   0.0f};
        // Rectification matrix (stereo cameras only)
        // A rotation matrix aligning the camera coordinate system to the ideal
        // stereo image plane so that epipolar lines in both stereo images are
        // parallel.
        camera_info.r = {1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f};
        // clang-format on

        /*camera_info_.d = intrinsics.d;
        camera_info_.k = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 1,
            static_cast<float>(camera_info_.height / 2), 0, 0, 1};
        camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        camera_info_.p = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 0, 1,
            static_cast<float>(camera_info_.height / 2), 0, 0, 0, 1, 0};*/

        pub.publish(depth_image);
        publisher_info_.publish(camera_info);
/*
		uint8_t *depth_color = frame->apply_colormap();
		cv::Mat yuvImg(frame->get_height(), frame->get_width(), CV_8UC3, depth_color);

		std::string msg = std::to_string(frame->get_width()) + "x" + std::to_string(frame->get_height()) + " fps:" + std::to_string(g_fps);
		int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 1;
		int thickness = 2;
		int baseline;
		cv::Size text_size = cv::getTextSize(msg, font_face, font_scale, thickness, &baseline);

		cv::Point origin;
		origin.x = yuvImg.cols / 2 - text_size.width / 2;
		origin.y = 0 + text_size.height;
		cv::putText(yuvImg, msg, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 2, 0);

		cv::namedWindow("MAP_COLOR", cv::WINDOW_NORMAL);
		cv::imshow("MAP_COLOR", yuvImg);

		sy3::sy3_intrinsics intrinsics = frame->get_profile()->get_intrinsics();
		// printf("intrinsics: %d x %d \n", intrinsics.width, intrinsics.height);*/
	}
}

private:
    //ImagePtr depth_raw_frame(new Image);
    
    image_transport::Publisher pub;
    sensor_msgs::msg::CameraInfo camera_info;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_info_;
    

    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Int32 message_;
    volatile bool g_is_start = false;
    volatile int g_fps = 0;
    double  g_last_time = 0;
    volatile int g_frame_count = 0;
    //std::thread fpsThread;
    sy3::pipeline *pline;
    int nIndex = 0;


};


int main(int argc, char * argv[]) {

    //sudo apt install ros-foxy-image-transport-plugins

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node_pub = std::make_shared<PublisherNode>(options);
    //auto node_sub = std::make_shared<SubscriberNode>();
    //rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::ExecutorArgs(), 4);
    executor.add_node(node_pub);
    //executor.add_node(node_sub);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}






















/*


void cs20node::publish_loop(){



	sy3::sy3_error e;
	sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
	sy3::depth_frame *depth_frame = frameset->get_depth_frame();
		
	if (depth_frame == nullptr)
		{
			//	printf("depth_frame:empty \n");
		}
	else
		{
			
			show_depth_frame(depth_frame, DEPTH_WINDOW_NAME);
			
	}

	delete frameset;
	nIndex++;
		/*
		switch (cv::waitKey(30))
		{

		case 'a':
		{

			pline->stop(e);
			cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, 640, 480, e);
			//cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_IR, 640, 480, e);
			pline->start(cfg, e);
			switch_flag = !switch_flag;
			nIndex = 1;
			printf("%s  %d   switch 640x480\n", __FILE__, __LINE__);
		}
		break;

		case 'b':
		{
			pline->stop(e);
			cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, 320, 240, e);
			//cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_IR, 320, 240, e);
			pline->start(cfg, e);
			switch_flag = !switch_flag;
			nIndex = 0;
			printf("%s  %d   switch 320x240\n", __FILE__, __LINE__);
		}
		break;

		case 'c':
		{

			uint16_t value;
			dev->get_sensor(e)->get_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, value, e);
			printf("SY3_OPTION_DEPTH_IMAGE_FILTER:%d \n", value);
		}
		break;

		case 'd':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, filter_value, e);
			filter_value = !filter_value;
		}
		break;

		case '0':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_TOF_IMAGE_FLIP, 0, e);
		}
		break;

		case '1':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, 888, e);
		}
		break;

		case '5':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, 666, e);
		}
		break;

		case '6':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, 777, e);
		}
		break;

		case '2':
		{
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE_RANGE, 20333, 300, e);
		}
		break;

		case '3':
		{
			uint16_t min = 0;
			dev->get_sensor(e)->get_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, min, e);
			printf("min %d ", min);
		}
		break;

		case 'e': {
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, 14800, e);
		} break;

		case 'f': {
			dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_DISTANCE_RANGE, 2477,22, e);
		} break;

		case 'g': {

			uint16_t min = 0;uint16_t max = 0;
			dev->get_sensor(e)->get_option(sy3::sy3_option::SY3_OPTION_DEFAULT_DISTANCE_RANGE, min, max, e);
			printf("min %d max %d \n", min, max);

		} break;

		case 'q': {

			g_is_start = false;
			pline->stop(e);
			delete ctx;
			cv::destroyAllWindows();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			quit = true;
			break;

		}break;

		default:

			break;
		}

		


}


void calculate_framerate()
{

	while (g_is_start)
	{
		double cur_time = cv::getTickCount() / cv::getTickFrequency() * 1000;

		if (cur_time - g_last_time > 1000)
		{
			//printf("===============> cur_time:%lf \n", cur_time);
			g_fps = g_frame_count;
			g_frame_count = 0;
			g_last_time = cur_time;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void print_device_info(sy3::device *dev)
{
	sy3::sy3_error e;
	printf("\nUsing device 0, an %s\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_NAME, e));
	printf("    Serial number: %s\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_SERIAL_NUMBER, e));
	printf("    Firmware version: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_FIRMWARE_VERSION, e));
}

void print_support_format(sy3::device *dev, sy3::sy3_error &e)
{

	std::vector<sy3::sy3_stream> support_stream = dev->get_support_stream(e);
	for (int i = 0; i < support_stream.size(); i++)
	{
		printf("support stream:%s \n", sy3_stream_to_string(support_stream[i]));
		std::vector<sy3::sy3_format> support_format = dev->get_support_format(support_stream[i], e);
		for (int j = 0; j < support_format.size(); j++)
		{
			printf("\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
		}
	}
}
*/