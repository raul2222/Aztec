#include "libsynexens3/libsynexens3.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

static cv::Point cur_mouse_point = {-1, -1};
bool  dump_raw = false;
int  dump_index = 0;



void on_mouse(int event, int x, int y, int flags, void* ustc) {


	if (event == cv::EVENT_LBUTTONDOWN || (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)))
	{

		printf("x:%d y:%d\n", x, y);
		cur_mouse_point = cv::Point(x, y);
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
		cur_mouse_point = cv::Point(-1, -1);
	}
}

void DumpRaw(uint8_t* depth_data, cv::Mat& rbg_mat, cv::Mat& rbgd_mat, int w, int h) {

	std::string pre = std::to_string(dump_index++) + "-" + std::to_string(w) + "x" + std::to_string(h) + "-" + std::to_string(time(0));
	std::string depth_name = "dump\\" + pre + "_depth.raw";
	std::string rgb_name = "dump\\" + pre + "_rgb.bmp";
	std::string rgbd_name = "dump\\" + pre + "_rgbd_result.bmp";

	FILE* fp = fopen(depth_name.c_str(), "wb+");

	if (fp != NULL)
	{
		fwrite(depth_data, sizeof(uint16_t), w * h, fp);
		fflush(fp);
		fclose(fp);
	}
	cv::imwrite(rgb_name, rbg_mat);
	cv::imwrite(rgbd_name, rbgd_mat);
	printf("save depth:%s   rgb:%s \n", depth_name.c_str(), rgb_name.c_str());

}

void Render(uint8_t* piexls_depth, uint8_t* piexls_rgb, int width, int height)
{
	cv::Mat gray16_img(height, width, CV_16UC1, piexls_depth);
	cv::Mat normalize_img;
	cv::Mat depth_color_img;
	cv::Mat gray8_img = cv::Mat::zeros(gray16_img.size(), CV_8U);
	cv::normalize(gray16_img, normalize_img, 0, 255, cv::NORM_MINMAX);
	cv::convertScaleAbs(normalize_img, gray8_img);
	cv::cvtColor(gray8_img, depth_color_img, cv::COLOR_GRAY2BGR);

	cv::Mat rgb_img(height, width, CV_8UC3, piexls_rgb);
	//cv::cvtColor(rgb_img, rgb_img, cv::COLOR_BGR2RGB);

	int  width_bar = 40;
	cv::Mat bar_img = cv::Mat(height, width_bar, CV_8UC3, cv::Scalar(0x00, 0x00, 0xcc));
	cv::cvtColor(bar_img, bar_img, cv::COLOR_RGB2BGR);

	cv::Mat render_img;
	cv::hconcat(rgb_img, bar_img, render_img);
	cv::hconcat(render_img, depth_color_img, render_img);

	cv::Mat help_bar_img = cv::Mat(height, width_bar, CV_8UC3, cv::Scalar(0x00, 0xff, 0x00));
	cv::cvtColor(help_bar_img, bar_img, cv::COLOR_RGB2BGR);


	cv::hconcat(render_img, bar_img, render_img);

	//RGBD�ϳ�ͼ
	cv::Mat rgbd_img(height, width, CV_8UC3, cv::Scalar(0));// rgb_img;
	for (int row = 0; row < rgb_img.rows; row++)
		for (int col = 0; col < rgb_img.cols; col++)
		{
			if (gray16_img.ptr<uint16_t>(row)[col] > 10)
			{
				rgbd_img.ptr<uchar>(row)[col * 3] = rgb_img.ptr<uchar>(row)[col * 3];
				rgbd_img.ptr<uchar>(row)[col * 3 + 1] = rgb_img.ptr<uchar>(row)[col * 3 + 1];
				rgbd_img.ptr<uchar>(row)[col * 3 + 2] = rgb_img.ptr<uchar>(row)[col * 3 + 2];
			}
		}
	cv::hconcat(render_img, rgbd_img, render_img);

	cv::namedWindow(RGBD_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::setMouseCallback(RGBD_WINDOW_NAME, on_mouse, 0);
	int x = width * 2 + width_bar * 2 + width / 2;
	int y = height / 2;
	int local_x = x - width * 2 - width_bar * 2;
	int local_y = y;
	char temp[48];
	sprintf(temp, " (x:%d,y:%d,d:%d)", local_x, local_y, ((uint16_t*)piexls_depth)[local_y * width + local_x]);
	cv::putText(render_img, temp, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
	circle(render_img, cv::Point(x, y), 10, cv::Scalar(255, 255, 0), 0);

	x = width + width_bar + width / 2;
	y = height / 2; char temp1[48];
	sprintf(temp1, " (x:%d,y:%d,d:%d)", local_x, local_y, ((uint16_t*)piexls_depth)[local_y * width + local_x]);
	cv::putText(render_img, temp1, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
	circle(render_img, cv::Point(x, y), 10, cv::Scalar(255, 0, 0), 0);

	x = width / 2;
	y = height / 2;
	char temp2[48];
	sprintf(temp2, " (x:%d,y:%d,d:%d)", local_x, local_y, ((uint16_t*)piexls_depth)[local_y * width + local_x]);
	cv::putText(render_img, temp2, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
	circle(render_img, cv::Point(x, y), 10, cv::Scalar(255, 255, 0), 0);

	//sprintf(temp, " d:%d", ((uint16_t*)piexls_depth)[local_y *width + local_x]);

	if (cur_mouse_point.x > 0 && cur_mouse_point.y > 0 && cur_mouse_point.y < height) {

		if (cur_mouse_point.x < width) {
			char temp[48];
			sprintf(temp, " (x:%d,y:%d,d:%d)", cur_mouse_point.x, cur_mouse_point.y, ((uint16_t*)piexls_depth)[cur_mouse_point.y * width + cur_mouse_point.x]);
			cv::putText(render_img, temp, cv::Point(cur_mouse_point.x, cur_mouse_point.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
			circle(render_img, cv::Point(cur_mouse_point.x, cur_mouse_point.y), 10, cv::Scalar(255, 255, 0), 0);

			sprintf(temp, " d:%d", ((uint16_t*)piexls_depth)[cur_mouse_point.y * width + cur_mouse_point.x]);
			cv::putText(render_img, temp, cv::Point(cur_mouse_point.x + width + width_bar, cur_mouse_point.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
			circle(render_img, cv::Point(cur_mouse_point.x + width + width_bar, cur_mouse_point.y), 10, cv::Scalar(255, 255, 0), 0);
		}
		else if (cur_mouse_point.x < width * 2 + width_bar && cur_mouse_point.x>width + width_bar) {
			int local_x = cur_mouse_point.x - width - width_bar;
			int local_y = cur_mouse_point.y;
			char temp[32];
			circle(render_img, cv::Point(cur_mouse_point.x, cur_mouse_point.y), 10, cv::Scalar(255, 233, 0), 0);
			//sprintf(temp, " d:%d", ((uint16_t*)piexls_depth)[cur_mouse_point.y *width + cur_mouse_point.x -width - width_bar]);
			sprintf(temp, " (x:%d,y:%d,d:%d)", local_x, local_y, ((uint16_t*)piexls_depth)[local_y * width + local_x]);
			cv::putText(render_img, temp, cv::Point(cur_mouse_point.x, cur_mouse_point.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
		}
		else if (cur_mouse_point.x < width * 3 + width_bar * 2 && cur_mouse_point.x>width * 2 + width_bar * 2) {

			int local_x = cur_mouse_point.x - width * 2 - width_bar * 2;
			int local_y = cur_mouse_point.y;
			char temp[48];
			sprintf(temp, " (x:%d,y:%d,d:%d)", local_x, local_y, ((uint16_t*)piexls_depth)[local_y * width + local_x]);
			cv::putText(render_img, temp, cv::Point(cur_mouse_point.x, cur_mouse_point.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2, 8);
			circle(render_img, cv::Point(cur_mouse_point.x, cur_mouse_point.y), 10, cv::Scalar(255, 255, 0), 0);

			sprintf(temp, " d:%d", ((uint16_t*)piexls_depth)[local_y * width + local_x]);
			//cv::putText(render_img, temp, cv::Point(cur_mouse_point.x, cur_mouse_point.y), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 0, 255), 2, 8);
		}

	}
	if (dump_raw) {
		DumpRaw(piexls_depth, rgb_img, rgbd_img, width, height);
		dump_raw = false;
	}

	//cv::namedWindow("result", cv::WINDOW_NORMAL);
	//cv::resizeWindow("result", cv::Size(rgbd_img.cols/2, rgbd_img.rows / 2));
	//cv::imshow("result", rgbd_img);
	//if (dump_raw) {
	//	cv::imwrite(std::to_string(dump_index-1) + "-result"+  ".png", rgbd_img);
	//	//dump_raw = false;
	//}

	cv::resizeWindow(RGBD_WINDOW_NAME, cv::Size(render_img.cols / 2, render_img.rows / 2));
	cv::imshow(RGBD_WINDOW_NAME, render_img);

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

		sy3::frameset *set = engine->align_to_rgb(depth, rgb, e);
		/*show_depth_frame(set->get_depth_frame(), "algin_depth");
		show_rgb_rgb_frame(set->get_rgb_frame(), "algin_rgb");*/
		/*cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);*/
		Render((uint8_t*)set->get_depth_frame()->get_data(), (uint8_t*)set->get_rgb_frame()->get_data(), set->get_rgb_frame()->get_width(),
			set->get_rgb_frame()->get_height());

		delete set;
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
	while (!quit)
	{

		sy3::frameset *frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
		sy3::depth_frame *depth_frame = frameset->get_depth_frame();
		sy3::rgb_frame *rgb_frame = frameset->get_rgb_frame();

		show_align_rgbd(depth_frame, rgb_frame, pline->get_process_engin(e));

		switch (cv::waitKey(1)) {

		case 'd':
			dump_raw = true;
			break;
		case 's':
			dump_raw = false;
			break;
		}
		

		delete frameset;
		cv::waitKey(1);
	}
	system("pause");

	return 0;
}
