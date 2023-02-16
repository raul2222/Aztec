#include "libsynexens3/libsynexens3.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define RGB_WIDTH 1920
#define RGB_HEIGHT 1080
#define RGB_WINDOW_NAME "RGB"


void show_rgb_nv12_frame(sy3::frame *frame, const char *name)
{
	if (frame)
	{

		cv::Mat yuvImg(frame->get_height() * 3 / 2, frame->get_width(), CV_8UC1, frame->get_data());
		cv::Mat rgbImg(frame->get_height(), frame->get_width(), CV_8UC3);
		cv::cvtColor(yuvImg, rgbImg, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, rgbImg);
		const sy3::stream_profile *rgb_profile = frame->get_profile();
		sy3::sy3_intrinsics rgb_inteinics = rgb_profile->get_intrinsics();
		//	print_intri(rgb_inteinics);
	}
}

int main(int argc, char **argv)
{
	sy3::sy3_error e;
	printf("version:%s \n", sy3::sy3_get_version(e));
	sy3::context *ctx = sy3::sy3_create_context(e);
	sy3::device *dev = ctx->query_device(e);
	if (e != sy3::sy3_error::SUCCESS) {
		printf("error:%s \n", sy3::sy3_error_to_string(e));
		return 0;
	}

	sy3::pipeline *pline = sy3::sy3_create_pipeline(ctx, e);

	sy3::config *cfg = sy3_create_config(e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, e);

	pline->start(cfg, e);

	bool quit = false;
	while (!quit)
	{

		sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
		sy3::rgb_frame *rgb_frame = frameset->get_rgb_frame();
		show_rgb_nv12_frame(rgb_frame, RGB_WINDOW_NAME);
		delete frameset;
		cv::waitKey(1);
	}
	system("pause");

	return 0;
}
