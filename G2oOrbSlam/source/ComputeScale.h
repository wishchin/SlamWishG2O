#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "ShowTrajectory.h"

#include "Tracking.h"
#include "Map.h"

bool vfcompare(float A, float B);

namespace ORB_SLAM
{
	class AbsScale
	{
	public:
		cv::Mat Img_l;
		cv::Mat Img_r;

		ORB_SLAM::FramePublisher* sFramePub;
		ORB_SLAM::Map* sWorld;

		vector<ORB_SLAM::MapPoint*> MatchedMapPoints;
		vector<bool> Outliers;
		vector<cv::KeyPoint> CurrentKeys;
		ORB_SLAM::KeyFrame* CurrentKeyFrame;

		int FramesNum;
		float MapScale;

		boost::mutex mMutexScale0;
		boost::mutex mMutexScale1;
		
		//AbsScale();
		AbsScale(ORB_SLAM::FramePublisher* FramePub, ORB_SLAM::Map* World);

		void Run(ORB_SLAM::FramePublisher* FramePub, ORB_SLAM::Map* World);
		//void Run();
		float Computor(cv::Mat Img_l, cv::Mat Img_r);
	};
}