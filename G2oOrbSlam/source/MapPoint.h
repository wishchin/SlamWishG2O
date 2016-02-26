/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <iostream>  
#include<array>

#include<boost/thread.hpp>
#include<opencv2/core/core.hpp>

#include"KeyFrame.h"
#include"Map.h"


namespace ORB_SLAM
{


	class ImageFeature;
	class KeyFrame;
	class Map;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);

    void IncreaseVisible();
    void IncreaseFound();
    float GetFoundRatio();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     boost::mutex mMutexPos;
     boost::mutex mMutexFeatures;
};


namespace visualization_msgs
{
	class ImageFeature;
	class KeyFrame;
	class Map;

	class Header{
		public:
		std::string  frame_id;
	};

	typedef struct Orientation{
		float Alpha;
		float  Beta;
		float Theta;
		float     w;
		Orientation()
		{
			Alpha=0;
			Beta =0;
			Theta=0;
			w  =1.0;
		};
	};

	typedef struct Pose {

		Orientation orientation;
	};

	typedef struct Marker {
		
	public:
		int ARROW;
		int CUBE;
		int SPHERE;
		int CYLINDER;
		int LINE_STRIP;
		int LINE_LIST;
		int CUBE_LIST;
		int SPHERE_LIST;
		int POINTS;
		int TEXT_VIEW_FACING;
		int MESH_RESOURCE;
		int TRIANGLE_LIST;

		int ADD;
		int MODIFY;
		//int DELETE=2;
		int DELETEALL;

	public:
		Marker()
		{
			ARROW=0;
			CUBE=1;
			SPHERE=2;
			CYLINDER=3;
			LINE_STRIP=4;
			LINE_LIST=5;
			CUBE_LIST=6;
			SPHERE_LIST=7;
			POINTS=8;
			TEXT_VIEW_FACING=9;
			MESH_RESOURCE=10;
			TRIANGLE_LIST=11;

			ADD=0;
			MODIFY=0;
			// DELETE=2;
			DELETEALL=3;
		};

		~Marker(){};

	protected:

	private:
		//snip....

	public:
		Header header; // header for time/frame information
		//float header; 
		std::string  ns; // Namespace to place this object in... used in conjunction with id to create a unique name for the object
		int     id ;  // object ID useful in conjunction with the namespace for manipulating and deleting the object later
		int type   ;   // Type of object
		int action ;  // 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
		//geometry_msgs/Pose      pose; // Pose of the object
		Pose pose;// std::array<float,3>      pose; //std::vector<float>     pose; // 
		//geometry_msgs/Vector3   scale; // Scale of the object 1,1,1 means default (usually 1 meter square)
		pcl::PointXYZRGB    scale;
		//std_msgs/ColorRGBA      color;  // Color [0.0-1.0]
		pcl::PointXYZRGBA      color;
		//duration             lifetime; // How long the object should last before being automatically deleted.  0 means forever
		int                  lifetime;
		bool             frame_locked;  // If this marker should be frame-locked, i.e. retransformed into its frame every timestep

		//Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
		//geometry_msgs/Point[] points;
		pcl::PointCloud<pcl::PointXYZRGB> points;

		//Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
		//number of colors must either be 0 or equal to the number of points
		//NOTE: alpha is not yet used
		//std_msgs/ColorRGBA[] colors;
		pcl::PointXYZRGB   colors;

		// NOTE: only used for text markers
		std::string text;

		// NOTE: only used for MESH_RESOURCE markers
		std::string        mesh_resource;
		bool mesh_use_embedded_materials;
	};
}

} //namespace ORB_SLAM

#endif // MAPPOINT_H
