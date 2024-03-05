#include <cmath>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#define SHOW_FRAMES 1
#define FRAMES_PER_SECOND 24
#define CROP_NUM 300

namespace objects {
	// 1 pixel = 1 cm
	class Crop {
		public:
		double x, y, r;
	};


	class Robot {
		public:
			double length, width; // cm, cm
			double x, y, angle; // cm, cm, degrees

			Robot(double x, double y, double angle, double l, double w)
			{
				this->x = x;
				this->y = y;
				this->angle = angle;
				length = l;
				width = w;
			};

			void move(double speed, double angle) // centimeters per second, degrees
			{
				this->angle += angle * (speed / FRAMES_PER_SECOND);
				x += (speed / FRAMES_PER_SECOND) * std::cos(this->angle * 3.1415/180);
				y += (speed / FRAMES_PER_SECOND) * std::sin(this->angle * 3.1415/180);
			}

			std::vector<Crop> detect(Crop crops[]) {
				std::vector<Crop> detected;
				double center_x = (int) this->x + (length + 20) * std::cos((this->angle) * 3.1415/180);
				double center_y = (int) this->y + (length + 20) * std::sin((this->angle) * 3.1415/180);

				for (int i = 0; i < 300; i++)
				{
					if (std::sqrt((crops[i].x - center_x)*(crops[i].x - center_x) + (crops[i].y - center_y)*(crops[i].y - center_y)) < 70)
					{
						detected.push_back(crops[i]);
					}
				}

				return detected;
			}
	};
}


namespace draw {
	using namespace objects;
	void draw_robot(cv::Mat* img, Robot* r)
	{
		cv::circle(*img, cv::Point((int) r->x + (r->length + 20) * std::cos((r->angle) * 3.1415/180), (int) r->y + (r->length + 20) * std::sin((r->angle) * 3.1415/180)), 70, cv::Scalar(20, 20, 20), cv::FILLED, cv::LINE_8);
		
		double x1 = r->x + (r->width / 2) * std::cos((r->angle+90) * 3.1415/180) + (r->length / 2) * std::cos((r->angle) * 3.1415/180);	
		double y1 = r->y + (r->width / 2) * std::sin((r->angle+90) * 3.1415/180) + (r->length / 2) * std::sin((r->angle) * 3.1415/180);

		double x2 = r->x + (r->width / 2) * std::cos((r->angle-90) * 3.1415/180) + (r->length / 2) * std::cos((r->angle) * 3.1415/180);	
		double y2 = r->y + (r->width / 2) * std::sin((r->angle-90) * 3.1415/180) + (r->length / 2) * std::sin((r->angle) * 3.1415/180);

		double x3 = r->x + (r->width / 2) * std::cos((r->angle+90) * 3.1415/180) - (r->length / 2) * std::cos((r->angle) * 3.1415/180);	
		double y3 = r->y + (r->width / 2) * std::sin((r->angle+90) * 3.1415/180) - (r->length / 2) * std::sin((r->angle) * 3.1415/180);

		double x4 = r->x + (r->width / 2) * std::cos((r->angle-90) * 3.1415/180) - (r->length / 2) * std::cos((r->angle) * 3.1415/180);	
		double y4 = r->y + (r->width / 2) * std::sin((r->angle-90) * 3.1415/180) - (r->length / 2) * std::sin((r->angle) * 3.1415/180);

		cv::line(*img,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(255,0,0),2);
		cv::line(*img,cv::Point(x2,y2),cv::Point(x4,y4),cv::Scalar(255,0,0),2);
		cv::line(*img,cv::Point(x3,y3),cv::Point(x4,y4),cv::Scalar(255,0,0),2);
		cv::line(*img,cv::Point(x3,y3),cv::Point(x1,y1),cv::Scalar(255,0,0),2);
		cv::circle(*img, cv::Point((int) r->x + (r->length / 2) * std::cos((r->angle) * 3.1415/180), (int) r->y + (r->length / 2) * std::sin((r->angle) * 3.1415/180)), 5, cv::Scalar( 0, 0, 255), cv::FILLED, cv::LINE_8);
	}

	void draw_crops(cv::Mat* img, Crop crops[])
	{
		Crop c;
		for (int i = 0; i < CROP_NUM; i++) 
		{
			c = crops[i];
			cv::circle(*img,
      	cv::Point((int) c.x, (int) c.y),
      	c.r,
      	cv::Scalar( 0, 255, 0),
      	cv::FILLED,
      	cv::LINE_8 );	
		}
	}

	void draw_detected(cv::Mat* img, std::vector<Crop> detected)
	{
		for (Crop c : detected) 
		{
			cv::circle(*img,
      	cv::Point((int) c.x, (int) c.y),
      	c.r,
      	cv::Scalar( 0, 0, 255),
      	cv::FILLED,
      	cv::LINE_8 );	
		}


	}
}

namespace connect {

}


int main(int argc, char* argv[])
{
	// setup
	std::cout << "NaviSim: START SETUP" << std::endl;

	int frame_number = 0;
	objects::Robot robot = objects::Robot(925, 600, -90, 82.0, 60.0);

	objects::Crop crops[CROP_NUM];

	int cx = 1000;
	int cy = 950;

	for (int i = 0; i < 12; i++) // 12 rows
	{
		for (int j = 0; j < 25; j++) {
			crops[i*25+j].x = cx + 40 * cos((cy - 600) * (3.1415 / 350));
			crops[i*25+j].y = cy;
			crops[i*25+j].r = 2;
			cy -= 30;
		}
		cx -= 75;
		cy = 950;
	}

	// simulation loop
	while (1)
	{
		frame_number++;
		//robot.move(100, -0.2);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000 / FRAMES_PER_SECOND));

		std::vector<objects::Crop> detected = robot.detect(crops);

		// show frame 
		#if SHOW_FRAMES

		std::cout << "NaviSim: FRAME " << frame_number << std::endl;

	
	
	  cv::Mat img = cv::Mat::zeros(1200, 1200, CV_8UC3);
		draw::draw_robot(&img, &robot);
		draw::draw_crops(&img, crops);
		draw::draw_detected(&img, detected);

		cv::imshow("NaviSim", img);
		cv::waitKey(1);

		#endif

	}
}
