#include "stdafx.h"

#include "ComputeScale.h"

bool vfcompare(float A, float B)
{
	return A < B;
}

namespace ORB_SLAM
{
	AbsScale::AbsScale(ORB_SLAM::FramePublisher* FramePub, 
		ORB_SLAM::Map* World):sFramePub(FramePub),sWorld(World)
	//AbsScale::AbsScale()
	{
		//Img_l = FramePub->CurrentLeftImage;
		//Img_r = FramePub->CurrentRightImage;

		//MatchedMapPoints = FramePub->GetMatchedMapPoints();
		//Outliers = FramePub->GetOutliers();
		//CurrentKeys = FramePub->GetCurrentKeys();

		//CurrentKeyFrame = World->GetAllKeyFrames()[World->GetAllKeyFrames().size()-1];

		//FramesNum = World->KeyFramesInMap();
		MapScale = 1;
	}

	void AbsScale::Run(ORB_SLAM::FramePublisher* FramePub, ORB_SLAM::Map* World)
	//void AbsScale::Run()
	{
		while (1)
		{
			FramesNum = sWorld->KeyFramesInMap();
			if (FramesNum == 10)
			{
				//boost::mutex::scoped_lock lock(mMutexScale0);
				cout << endl << "FrameNum = " << FramesNum << endl;

				Img_l = sFramePub->CurrentLeftImage.clone();
				Img_r = sFramePub->CurrentRightImage.clone();

				//Img_l = cv::imread("left_0.bmp");
				//Img_r = cv::imread("right_0.bmp");

				imshow("l312", Img_l);
				imshow("r312", Img_r);
				cvWaitKey(10);

				MatchedMapPoints = sFramePub->GetMatchedMapPoints();
				cout << MatchedMapPoints.size() << endl;

				Outliers = sFramePub->GetOutliers();
				cout << Outliers.size() << endl;

				CurrentKeys = sFramePub->GetCurrentKeys();
				cout << CurrentKeys.size() << endl;

				CurrentKeyFrame = sWorld->GetAllKeyFrames()[World->GetAllKeyFrames().size()-1];

				MapScale = Computor(Img_l, Img_r);
				if (MapScale != 1)
				{
					break;
				}
			}
		}
	}

	float AbsScale::Computor(cv::Mat Img_l, cv::Mat Img_r)
	{
		//boost::mutex::scoped_lock lock(mMutexScale1);
		int h = 480, w = 640;
		//左摄像头的参数(320 * 240)
#pragma region Para_l

		double *mi_l;
		double *md_l;
		mi_l = new double[3*3];
		md_l = new double[5];

		CvMat intrinsic_matrix_l,distortion_coeffs_l;
		//摄像机内参数
		cvInitMatHeader(&intrinsic_matrix_l,3,3,CV_64FC1,mi_l);
		//镜头畸变参数
		cvInitMatHeader(&distortion_coeffs_l,1,5,CV_64FC1,md_l);

		//320*240 120度广角,参数由matlab获得
		double fc1_l,fc2_l,cc1_l,cc2_l,kc1_l,kc2_l,kc3_l,kc4_l,kc5_l;
		fc1_l = 438.309392545601;
		fc2_l = 437.413208466851;
		cc1_l = 328.251931559548;
		cc2_l = 251.422879088361;
		kc1_l = -0.391334768135577;
		kc2_l = 0.154196647710438;
		kc3_l = 0.000393196119271317;
		kc4_l = -0.00374817457772282;
		kc5_l = 0;

		cvmSet(&intrinsic_matrix_l, 0, 0, fc1_l);
		cvmSet(&intrinsic_matrix_l, 0, 1, 0);
		cvmSet(&intrinsic_matrix_l, 0, 2, cc1_l);
		cvmSet(&intrinsic_matrix_l, 1, 0, 0);
		cvmSet(&intrinsic_matrix_l, 1, 1, fc2_l);
		cvmSet(&intrinsic_matrix_l, 1, 2, cc2_l);
		cvmSet(&intrinsic_matrix_l, 2, 0, 0);
		cvmSet(&intrinsic_matrix_l, 2, 1, 0);
		cvmSet(&intrinsic_matrix_l, 2, 2, 1);

		cvmSet(&distortion_coeffs_l, 0, 0, kc1_l);
		cvmSet(&distortion_coeffs_l, 0, 1, kc2_l);
		cvmSet(&distortion_coeffs_l, 0, 2, kc3_l);
		cvmSet(&distortion_coeffs_l, 0, 3, kc4_l);
		cvmSet(&distortion_coeffs_l, 0, 4, kc5_l);

#pragma endregion 

		//右摄像头的参数(320 * 240)
#pragma region Para_r

		double *mi_r;
		double *md_r;
		mi_r = new double[3*3];
		md_r = new double[5];

		CvMat intrinsic_matrix_r,distortion_coeffs_r;
		//摄像机内参数
		cvInitMatHeader(&intrinsic_matrix_r,3,3,CV_64FC1,mi_r);
		//镜头畸变参数
		cvInitMatHeader(&distortion_coeffs_r,1,5,CV_64FC1,md_r);

		//320*240 120度广角,参数由matlab获得
		double fc1_r,fc2_r,cc1_r,cc2_r,kc1_r,kc2_r,kc3_r,kc4_r,kc5_r;
		fc1_r = 438.342648119741;
		fc2_r = 438.045639432845;
		cc1_r = 338.911308383114;
		cc2_r = 255.121745143177;
		kc1_r = -0.355615818033982;
		kc2_r = 0.114288892204538;
		kc3_r = -0.00143781468460296;
		kc4_r = 0.00396006864145378;
		kc5_r = 0;

		cvmSet(&intrinsic_matrix_r, 0, 0, fc1_r);
		cvmSet(&intrinsic_matrix_r, 0, 1, 0);
		cvmSet(&intrinsic_matrix_r, 0, 2, cc1_r);
		cvmSet(&intrinsic_matrix_r, 1, 0, 0);
		cvmSet(&intrinsic_matrix_r, 1, 1, fc2_r);
		cvmSet(&intrinsic_matrix_r, 1, 2, cc2_r);
		cvmSet(&intrinsic_matrix_r, 2, 0, 0);
		cvmSet(&intrinsic_matrix_r, 2, 1, 0);
		cvmSet(&intrinsic_matrix_r, 2, 2, 1);

		cvmSet(&distortion_coeffs_r, 0, 0, kc1_r);
		cvmSet(&distortion_coeffs_r, 0, 1, kc2_r);
		cvmSet(&distortion_coeffs_r, 0, 2, kc3_r);
		cvmSet(&distortion_coeffs_r, 0, 3, kc4_r);
		cvmSet(&distortion_coeffs_r, 0, 4, kc5_r);

#pragma endregion 

		//双目系统参数
#pragma region Para_r2l(320*240)
		double *R_r2l;
		double *T_r2l;
		R_r2l = new double[3*3];
		T_r2l = new double[3];

		CvMat rotation_matrix, translation_matrix;
		//摄像机内参数
		cvInitMatHeader(&rotation_matrix,3,3,CV_64FC1,R_r2l);
		//镜头畸变参数
		cvInitMatHeader(&translation_matrix,3,1,CV_64FC1,T_r2l);

		cvmSet(&rotation_matrix, 0, 0, 0.999217415262760);
		cvmSet(&rotation_matrix, 1, 0, -0.0165015892574143);
		cvmSet(&rotation_matrix, 2, 0, -0.0359479427448721);
		cvmSet(&rotation_matrix, 0, 1, 0.0163939580147868);
		cvmSet(&rotation_matrix, 1, 1, 0.999860207734650);
		cvmSet(&rotation_matrix, 2, 1, -0.00328681143861797);
		cvmSet(&rotation_matrix, 0, 2, 0.0359971551128478);
		cvmSet(&rotation_matrix, 1, 2, 0.00269491016607455);
		cvmSet(&rotation_matrix, 2, 2, 0.999348258758166);

		cvmSet(&translation_matrix, 0, 0, -157.251570632330);
		cvmSet(&translation_matrix, 1, 0, 2.36085446195827);
		cvmSet(&translation_matrix, 2, 0, 5.58686929485279);

#pragma endregion

		//双目校正
#pragma region Undistort
		CvMat* mx1 = cvCreateMat( h, w, CV_32F );
		CvMat* my1 = cvCreateMat( h, w, CV_32F );
		CvMat* mx2 = cvCreateMat( h, w, CV_32F );
		CvMat* my2 = cvCreateMat( h, w, CV_32F );
		double R1[3][3], R2[3][3], P1[3][4], P2[3][4], Q[4][4];
		CvMat t_R1 = cvMat(3, 3, CV_64F, R1);
		CvMat t_R2 = cvMat(3, 3, CV_64F, R2);
		CvMat t_Q = cvMat(4, 4, CV_64F, Q );
		CvRect roi1, roi2;

		CvMat t_P1 = cvMat(3, 4, CV_64F, P1);
		CvMat t_P2 = cvMat(3, 4, CV_64F, P2);
		cvStereoRectify( &intrinsic_matrix_l, &intrinsic_matrix_r,
			&distortion_coeffs_l, &distortion_coeffs_r, cvSize(w,h),
			&rotation_matrix, &translation_matrix, 
			&t_R1, &t_R2, &t_P1, &t_P2, &t_Q,
			CV_CALIB_ZERO_DISPARITY,
			0,cvSize(w,h), &roi1, &roi2); 
		// 计算mx1,my1,mx2,my2用于cvRemap()
		cvInitUndistortRectifyMap(&intrinsic_matrix_l,
			&distortion_coeffs_l,&t_R1,&t_P1, mx1, my1);
		cvInitUndistortRectifyMap(&intrinsic_matrix_r,
			&distortion_coeffs_r,&t_R2,&t_P2, mx2, my2);
#pragma endregion

		//视差图计算元
#pragma region StereoBM
		//cv::StereoBM* bm = new cv::StereoBM;
		//int numberOfDisparities = 0;
		//numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((w/8) + 15) & -16;  

		//bm->state->roi1 = roi1;  
		//bm->state->roi2 = roi2;  
		//bm->state->preFilterCap = 31;

		//int SADWindowSize = 0;
		//bm->state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;  
		//bm->state->minDisparity = 0;  
		//bm->state->numberOfDisparities = numberOfDisparities;  
		//bm->state->textureThreshold = 10;  
		//bm->state->uniquenessRatio = 15;  
		//bm->state->speckleWindowSize = 100;  
		//bm->state->speckleRange = 32;  
		//bm->state->disp12MaxDiff = 1;
		cv::StereoSGBM* sgbm = new cv::StereoSGBM;  
		sgbm->preFilterCap = 63;  
		int SADWindowSize=11;   
		int cn = 1;  
		sgbm->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;  
		sgbm->P1 = 4*cn*sgbm->SADWindowSize*sgbm->SADWindowSize;  
		sgbm->P2 = 32*cn*sgbm->SADWindowSize*sgbm->SADWindowSize;  
		sgbm->minDisparity = 0;  
		sgbm->numberOfDisparities = 32;  
		sgbm->uniquenessRatio = 10;  
		sgbm->speckleWindowSize = 100;  
		sgbm->speckleRange = 32;  
		sgbm->disp12MaxDiff = 1;  

#pragma endregion

		cv::Mat Img_l0 = Img_l.clone();
		cv::Mat Img_r0 = Img_r.clone();
		//cv::Mat Gray_l, Gray_r;
		//cv::Mat Image_l, Image_r;
		//cv::Mat disp;
		cv::Mat Gray_l(480, 640, CV_32FC1, cv::Scalar(0));
		cv::Mat Gray_r(480, 640, CV_32FC1, cv::Scalar(0));
		cv::Mat Image_l(480, 640, CV_32FC1, cv::Scalar(0));
		cv::Mat Image_r(480, 640, CV_32FC1, cv::Scalar(0));
		cv::Mat disp(480, 640, CV_32FC1, cv::Scalar(0));

		imshow("source_l", Img_l0);
		imshow("source_r", Img_r0);
		
		//转换为灰度图
		cv::cvtColor(Img_l0, Gray_l, CV_RGB2GRAY);
		cv::cvtColor(Img_r0, Gray_r, CV_RGB2GRAY);

		cv::Mat _mx1(mx1);
		cv::Mat _my1(my1);
		cv::Mat _mx2(mx2);
		cv::Mat _my2(my2);

		//矫正畸变(opencv)
		cv::remap(Gray_l,Image_l,_mx1,_my1,CV_INTER_LINEAR);
		cv::remap(Gray_r,Image_r,_mx2,_my2,CV_INTER_LINEAR);

		imshow("l", Image_l);
		imshow("r", Image_r);
		cvWaitKey(10);
		
		cout << "BM start" << endl;
		
		sgbm->operator()(Image_l, Image_r, disp);
		//bm->operator()(Image_l, Image_r, disp);
		cout << "BM end" << endl;

		imshow("disp", disp);
		sgbm->~StereoSGBM();

		//基于视差图计算绝对坐标
		cv::Mat PointCoordinateValue;
		cv::Mat ReprojectMatrix(&t_Q);
		cv::reprojectImageTo3D(disp, PointCoordinateValue, ReprojectMatrix, true, CV_32F);
		PointCoordinateValue *= 16;

		//光心相对坐标
		cv::Mat CurrentCameraCenter = CurrentKeyFrame->GetCameraCenter().clone();

		float CCCX = CurrentCameraCenter.at<float>(0);
		float CCCY = CurrentCameraCenter.at<float>(1);
		float CCCZ = CurrentCameraCenter.at<float>(2);

		//取出当前帧特征点与相对坐标
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr SaveCurrentDatas(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointXYZRGB SaveCurrentData;
		for(unsigned int i = 0; i < MatchedMapPoints.size();i++)
		{
			if(MatchedMapPoints[i] || Outliers[i])
			{
				if(!Outliers[i])
				{
					SaveCurrentData.x = MatchedMapPoints[i]->GetWorldPos().at<float>(0);
					SaveCurrentData.y = MatchedMapPoints[i]->GetWorldPos().at<float>(1);
					SaveCurrentData.z = MatchedMapPoints[i]->GetWorldPos().at<float>(2);
					SaveCurrentData.r = CurrentKeys[i].pt.x;
					SaveCurrentData.g = CurrentKeys[i].pt.y;

					SaveCurrentDatas->push_back(SaveCurrentData);
				}
			}
		}


		//取出坐标
		pcl::PointCloud<pcl::PointXYZ>::Ptr AbsoluteCoordinate(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr RelativeCoordinate(new pcl::PointCloud<pcl::PointXYZ>);
		vector<float> ScaleSets;
		for (int i = 0; i < SaveCurrentDatas->size(); i++)
		{
			int x = (int)SaveCurrentDatas->points[i].r;
			int y = (int)SaveCurrentDatas->points[i].g;

			cv::Mat src(1,1,CV_32FC2,cv::Scalar(1,1));
			cv::Mat dst(1,1,CV_32FC2,cv::Scalar(1,1));

			src.at<cv::Vec2f>(0,0).val[0] = x;
			src.at<cv::Vec2f>(0,0).val[1] = y;

			//cout << "原始点位置：" << x << "," << y << endl;

			cv::Mat i_matrix(&intrinsic_matrix_l);
			cv::Mat d_coeffs(&distortion_coeffs_l);
			cv::Mat T_R1(&t_R1);
			cv::Mat T_P1(&t_P1);

			cv::undistortPoints(src, dst, i_matrix, d_coeffs, T_R1, T_P1);

			x = dst.at<cv::Vec2f>(0,0).val[0];
			y = dst.at<cv::Vec2f>(0,0).val[1];
			//cout << "校正后点位置：" << x << "," << y << endl;

			imwrite("undi_source.bmp", Img_l);
			imwrite("undi.bmp", Image_l);

			if (x <= 5 || x >= 635 || y <= 5 || y >= 475)
			{
				continue;
			}

			float a, b, c;
			bool FitValue = false;
			for (int i = 0; i < 3; i++)
			{
				for (int j = x - i; j <= x + i; j++)
				{
					for (int k = y - i; k <= y + i; k++)
					{
						c = PointCoordinateValue.at<cv::Vec3f>(j,k).val[2];
						if (c < 100 || c > 8000)
						{
							continue;
						}
						a = PointCoordinateValue.at<cv::Vec3f>(j,k).val[0];
						b = PointCoordinateValue.at<cv::Vec3f>(j,k).val[1];
						c = PointCoordinateValue.at<cv::Vec3f>(j,k).val[2];
						FitValue = true;
						break;
					}
					if (FitValue)
					{
						break;
					} 
				}
				if (FitValue)
				{
					break;
				} 
			}

			//cout << "a = " << a << endl
			//	<< "b = " << b << endl
			//	<< "c = " << c << endl;

			if (a > -4000 && a < 4000
				&& b > -4000 && b < 4000
				&& c > 100 && c < 8000)
			{
				//cout << "S = " << SaveCurrentDatas->points[i].x << endl;
				//cout << "a = " << a << endl;
				//cout << "CCCX = " << CCCX << endl;
				ScaleSets.push_back((sqrt(pow(a,2) + pow(b,2) + pow(c,2)))
					/(sqrt(pow(SaveCurrentDatas->points[i].x - CCCX, 2)
					+ pow(SaveCurrentDatas->points[i].y - CCCY, 2) + pow(SaveCurrentDatas->points[i].z - CCCZ, 2))));
			}
		}

		//cout << "scalesets.size() = " << ScaleSets.size() << endl;

		for (int i = 0; i < ScaleSets.size(); i++)
		{
			//cout << ScaleSets[i] << endl;
		}
		cout << "end" << endl;

		std::sort(ScaleSets.begin(), ScaleSets.end(), vfcompare);
		MapScale = ScaleSets[ScaleSets.size()/2];
		cvWaitKey(10);
		return MapScale;
	}
}