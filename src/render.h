#ifndef LIBSYNEXENS3_EXAMPLES_DEPTH_H
#define LIBSYNEXENS3_EXAMPLES_DEPTH_H

#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"libsynexens3/libsynexens3.h"

#define IR_WINDOW_NAME "IR"
#define COLOR_MAP_WINDOW_NAME "COLOR_MAP"
#define RGBD_WINDOW_NAME "RGBD"

using namespace std;

void print_intri(const sy3::sy3_intrinsics& intrinsics)
{
	sy3::sy3_error e;
	stringstream ss;
	ss << left << setw(14) << "  Width: " << "\t" << intrinsics.width << endl <<
		left << setw(14) << "  Height: " << "\t" << intrinsics.height << endl <<
		left << setw(14) << "  PPX: " << "\t" << setprecision(15) << intrinsics.ppx << endl <<
		left << setw(14) << "  PPY: " << "\t" << setprecision(15) << intrinsics.ppy << endl <<
		left << setw(14) << "  Fx: " << "\t" << setprecision(15) << intrinsics.fx << endl <<
		left << setw(14) << "  Fy: " << "\t" << setprecision(15) << intrinsics.fy << endl <<
		//  left << setw(14) << "  Distortion: " << "\t" << rs2_distortion_to_string(intrinsics.model) << endl <<
		left << setw(14) << "  Coeffs: ";

	for (auto i = 0u; i < sizeof(intrinsics.coeffs) / sizeof(intrinsics.coeffs[0]); ++i)
		ss << "\t" << setprecision(15) << intrinsics.coeffs[i] << "  ";

	float fov[2];
	sy3::sy3_fov(fov,&intrinsics,e);
	ss << endl << left << setw(14) << "  FOV (deg): " << "\t" << setprecision(4) << fov[0] << " x " << fov[1];

	cout << ss.str() << endl << endl;
}


void show_ir_frame(sy3::frame *frame, const char* name)
{
	if (frame) {
		cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);
		const sy3::stream_profile* ir_profile = frame->get_profile();
		sy3::sy3_intrinsics ir_inteinics = ir_profile->get_intrinsics();
		//	print_intri(ir_inteinics);
	}
}


#endif