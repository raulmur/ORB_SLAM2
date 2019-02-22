/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace ORB_SLAM2 {

struct CameraParameters {
	int width;
	int height;

	float fx;
	float fy;
	float cx;
	float cy;

	float k1;
	float k2;
	float p1;
	float p2;
	float k3;

	float bf;
	float fps;

	int RGB;
};

struct ORBextractorParameters {
	int nFeatures;
	float scaleFactor;
	int nLevels;
	int iniThFAST;
	int minThFAST;
};

struct TrackingParameters {
	CameraParameters Camera;
	ORBextractorParameters ORBextractor;
	float ThDepth;
	float DepthMapFactor;
};

struct ViewerParameters {
	float KeyFrameSize;
	float KeyFrameLineWidth;
	float GraphLineWidth;
	float PointSize;
	float CameraSize;
	float CameraLineWidth;

	float ViewpointX;
	float ViewpointY;
	float ViewpointZ;
	float ViewpointF; // focal length
};

struct Parameters : public TrackingParameters {
	ViewerParameters Viewer;
};

}

#endif // PARAMETERS_H
