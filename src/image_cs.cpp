
//#if 0
//#define DEPTH_WIDTH 640
//#define DEPTH_HEIGHT 480
//#else
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
//#endif
#include "libsynexens3/libsynexens3.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"

#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <unistd.h>
#include <thread>

///////////////////////////////////////////////////////////////
std::shared_ptr<rclcpp::Node> node = nullptr;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> publisher_info_;
image_transport::Publisher pub;
image_transport::Publisher pub_color;
sensor_msgs::msg::CameraInfo camera_info;

////////////////////////////////////////////////////////////
volatile bool g_is_start = false;
volatile int g_fps = 0;
double  g_last_time = 0;
volatile int g_frame_count = 0;
int nIndex = 0;
volatile bool config1 = false;
volatile bool config2 = false;

std::thread fpsThread;
sy3::pipeline *pline;
sy3::sy3_intrinsics intrinsics;
sy3::device *dev;

uint16_t filter_value = 0;

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

void config(){
    sy3::sy3_error e;
	printf("version:%s \n", sy3::sy3_get_version(e));
	sy3::context *ctx = sy3::sy3_create_context(e);
	dev = ctx->query_device(e);
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

    image_transport::ImageTransport it(node);
    
    pub = it.advertise("camera/image", 100);

    pub_color = it.advertise("camera2/image2", 1);
    
    publisher_info_ = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/info", 100);

    fpsThread = std::thread(calculate_framerate);

}



void show_depth_frame(sy3::depth_frame *frame)
{
	if (frame)
	{
		g_frame_count++;

        cv::Mat depth_frame_buffer_mat(frame->get_height(),
         frame->get_width(), CV_16UC1, frame->get_data());

        sensor_msgs::msg::Image::SharedPtr depth_image;
        //sensor_msgs::ImagePtr& depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();
        depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();

        std::string frame_tf = "tf2_frame";
        rclcpp::Time time = rclcpp::Clock().now();

		depth_image->header.stamp = time;
        depth_image->header.frame_id = frame_tf;
        
        camera_info.header.frame_id = frame_tf;
        camera_info.header.stamp = time;
        //sy3::sy3_intrinsics intrinsics = frame->get_profile()->get_intrinsics();

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
        publisher_info_->publish(camera_info);

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

        sensor_msgs::msg::Image::SharedPtr depth_img_color;
        //sensor_msgs::ImagePtr& depth_image = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();
        depth_img_color = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8UC3, yuvImg).toImageMsg();
        depth_img_color->header.stamp = time;
        depth_img_color->header.frame_id = frame_tf;

        pub_color.publish(depth_img_color);
		
        //cv::namedWindow("MAP_COLOR", cv::WINDOW_NORMAL);
		//cv::imshow("MAP_COLOR", yuvImg);

		//sy3::sy3_intrinsics intrinsics = frame->get_profile()->get_intrinsics();
		// printf("intrinsics: %d x %d \n", intrinsics.width, intrinsics.height);
	}
}




void timerCallback()
{
    sy3::sy3_error e;
	sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
	sy3::depth_frame *depth_frame = frameset->get_depth_frame();
		
	if (depth_frame == nullptr){
		//	printf("depth_frame:empty \n");
	}
	else{

        if(g_frame_count % 450 == 0 && config1 == false) {
            dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, filter_value, e);
            filter_value = !filter_value;
            uint16_t value;
            dev->get_sensor(e)->get_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, value, e);
            //printf("%d SY3_OPTION_DEPTH_IMAGE_FILTER:%d \n", g_frame_count, value);
            config1 = true;
        }

        if(g_frame_count % 550 == 0 && config2 == false) {
            dev->get_sensor(e)->set_option(sy3::sy3_option::SY3_OPTION_DISTANCE_RANGE, 2500,20, e);

            uint16_t min = 0;uint16_t max = 0;
			dev->get_sensor(e)->get_option(sy3::sy3_option::SY3_OPTION_DISTANCE_RANGE, min, max, e);
			printf("min %d max %d \n", min, max);
            config2 = true;
        }


        intrinsics = depth_frame->get_profile()->get_intrinsics();
		show_depth_frame(depth_frame);		

	}
    

	delete frameset;
	nIndex++;
}




int main(int argc, char * argv[]) {

    //sudo apt install ros-foxy-image-transport-plugins

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    node = std::make_shared<rclcpp::Node>("cs_20");

    auto timer = node->create_wall_timer(
    std::chrono::milliseconds(20), timerCallback);

    rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::ExecutorArgs(), 4);
    executor.add_node(node);

    config();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
