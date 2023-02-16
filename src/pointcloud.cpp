#include "libsynexens3/libsynexens3.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#define no_init_all_deprecated


#define RAW_WIDTH 1280
#define RAW_HEIGHT 960

#if 1
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#else
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
#endif

#define RGB_WIDTH 1920
#define RGB_HEIGHT 1080

#define RGBD_WINDOW_NAME "RGBD"
#define RGB_WINDOW_NAME "RGB"
#define TOF_WINDOW_NAME "TOF"



void Render(sy3::depth_frame *piexls_depth, sy3::frame *piexls_rgb, int width, int height)
{
	std::cout << width << std::endl;
	std::cout << height << std::endl;


	cv::Mat gray16(piexls_depth->get_height(), piexls_depth->get_width(), CV_16UC1, piexls_depth->get_data());
	cv::Mat tmp;
	cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
	cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
	cv::convertScaleAbs(tmp, gray8);
	cv::Mat yuvImg(piexls_rgb->get_height(), piexls_rgb->get_width(), CV_8UC3, piexls_rgb->get_data());
	//RGBD
	cv::Mat rgbd_img(height, width, CV_8UC3, cv::Scalar(0));// rgb_img;
	for (int row = 0; row < yuvImg.rows; row++) {
		for (int col = 0; col < yuvImg.cols; col++)
		{
			if (gray16.ptr<uint16_t>(row)[col] > 10)
			{
				rgbd_img.ptr<uchar>(row)[col * 3] = yuvImg.ptr<uchar>(row)[col * 3];
				rgbd_img.ptr<uchar>(row)[col * 3 + 1] = yuvImg.ptr<uchar>(row)[col * 3 + 1];
				rgbd_img.ptr<uchar>(row)[col * 3 + 2] = yuvImg.ptr<uchar>(row)[col * 3 + 2];
			}
		}
	}
	cv::namedWindow(RGBD_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(RGBD_WINDOW_NAME, RGB_WIDTH, RGB_HEIGHT);
	cv::imshow(RGBD_WINDOW_NAME, rgbd_img);

}






void show_depth_frame(sy3::depth_frame *frame, const char *name)
{
	if (frame)
	{

		cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);

	}
}

void show_rgb_rgb_frame(sy3::frame *frame, const char *name)
{
	if (frame)
	{

		cv::Mat yuvImg(frame->get_height(), frame->get_width(), CV_8UC3, frame->get_data());
		//cv::Mat rgbImg(frame->get_height(), frame->get_width(), CV_8UC3);
		//	cv::cvtColor(yuvImg, rgbImg, cv::ColorConversionCodes::COLOR_BGR2BGR);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, yuvImg);
		const sy3::stream_profile *rgb_profile = frame->get_profile();
		sy3::sy3_intrinsics rgb_inteinics = rgb_profile->get_intrinsics();

	}
}

void show_align_rgbd(sy3::depth_frame *depth, sy3::rgb_frame *rgb, sy3::process_engine *engine)
{
	sy3::sy3_error e;
	if (depth && rgb)
	{
		sy3::sy3_intrinsics intrinsics_tof = depth->get_profile()->get_intrinsics();
		printf("depth intrinsics: %d x %d  %f %f\n", intrinsics_tof.width, intrinsics_tof.height, intrinsics_tof.fx, intrinsics_tof.fy);
		sy3::sy3_intrinsics intrinsics_rgb = rgb->get_profile()->get_intrinsics();
		printf("rgb intrinsics: %d x %d  %f %f\n", intrinsics_rgb.width, intrinsics_rgb.height, intrinsics_rgb.fx, intrinsics_rgb.fy);

		show_depth_frame(depth, "depth");

		if (depth->get_width() == 640) {
			sy3::frameset *set = engine->align_to_rgb(depth, rgb, e);
			show_depth_frame(set->get_depth_frame(), "algin_depth");
			show_rgb_rgb_frame(set->get_rgb_frame(), "algin_rgb");
			int height = set->get_depth_frame()->get_height();
			int width = set->get_depth_frame()->get_width();
			std::cout << height << width << "asdsd" << std::endl;
			//Render(set->get_depth_frame(), set->get_rgb_frame(), width, height);
			delete set;

		}

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

int main(int argc, char **argv)
{
	sy3::sy3_error e;
	printf("version:%s \n", sy3::sy3_get_version(e));
	sy3::context *ctx = sy3::sy3_create_context(e);
	sy3::device *dev = ctx->query_device(e);

	if (e != sy3::sy3_error::SUCCESS) {
		printf("error:%d  %s \n", e, sy3::sy3_error_to_string(e));
		return 0;
	}

	print_support_format(dev, e);
	print_device_info(dev);

	sy3::pipeline *pline = sy3::sy3_create_pipeline(ctx, e);

	sy3::config *cfg = sy3_create_config(e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, e);

	pline->start(cfg, e);

	uint16_t exposure = 0;
	bool quit = false;

	while (true)
	{




		sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
		sy3::depth_frame *depth_frame = frameset->get_depth_frame();
		sy3::rgb_frame *rgb_frame = frameset->get_rgb_frame();
		//show_align_rgbd(depth_frame, rgb_frame, pline->get_process_engin(e));


		if (depth_frame && rgb_frame)
		{
			/*sy3::sy3_intrinsics intrinsics_tof = depth_frame->get_profile()->get_intrinsics();
			printf("depth intrinsics: %d x %d  %f %f\n", intrinsics_tof.width, intrinsics_tof.height, intrinsics_tof.fx, intrinsics_tof.fy);
			sy3::sy3_intrinsics intrinsics_rgb = rgb_frame->get_profile()->get_intrinsics();
			printf("rgb intrinsics: %d x %d  %f %f\n", intrinsics_rgb.width, intrinsics_rgb.height, intrinsics_rgb.fx, intrinsics_rgb.fy);

			show_depth_frame(depth_frame, "depth");*/

			if (depth_frame->get_width() == 640) {
				sy3::frameset *set = pline->get_process_engin(e)->align_to_rgb(depth_frame, rgb_frame, e);
				//show_depth_frame(set->get_depth_frame(), "algin_depth");
				//show_rgb_rgb_frame(set->get_rgb_frame(), "algin_rgb");
				int height = set->get_depth_frame()->get_height();
				int width = set->get_depth_frame()->get_width();
				std::cout << "height : " << height << "   width : " << width << std::endl;
				sy3::sy3_error e1;
				sy3::depth_frame *depth_frames = set->get_depth_frame();
				sy3::points* points = pline->get_process_engin(e1)->comptute_points(depth_frames, e1);
				//float* data = points->get_points();
				delete points;
				delete set;

			}

		}


		delete frameset;

		cv::waitKey(1);

	}
	system("pause");

	return 0;
}
