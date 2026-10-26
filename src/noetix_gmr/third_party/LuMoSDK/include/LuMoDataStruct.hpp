/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: LuMoDataStruct.hpp
* DESCRIPTION	: Mocap struct.
*
* VERDION	: 1.0.0
* DATE		: 2021/03/01
* AUTHOR	: jzy
*
***********************************************************************/
#pragma once

#include <map>
#include <string>
#include <vector>

#define LUSTERNET_SPACE_BEGIN namespace lusternet {
#define LUSTERNET_SPACE_END }
#define LUSTERNET_SPACE using namespace lusternet;

LUSTERNET_SPACE_BEGIN


/**
 * @brief  散点结构体，保存实时运动状态下的散点位置和姿态
 */
struct LST_MarkerINFO
{
	/**
	 * @brief Marker ID
	 */
	uint32_t MarkerID = 0;
	/**
	 * @brief Marker name
	 */
	std::string MarkerName = "";
	/**
	 * @brief 位置：X
	 */
	float X = 0.0;
	/**
	 * @brief 位置：Y
	 */
	float Y = 0.0;
	/**
	 * @brief 位置：Z
	 */
	float Z = 0.0;

};

/**
 * @brief  刚体结构体，保存实时运动状态下的刚体位置和姿态
 */
typedef struct tagStructRigidData
{
	uint32_t RigidID = 0;
	std::string RigidName = "";
	/**
	 * @brief 位置：X
	 */
	float X = 0.0;
	/**
	 * @brief 位置：Y
	 */
	float Y = 0.0;
	/**
	 * @brief 位置：Z
	 */
	float Z = 0.0;
	/**
	 * @brief 角度四元数：qx
	 */
	float qx = 0.0;
	/**
	 * @brief 角度四元数：qy
	 */
	float qy = 0.0;
	/**
	 * @brief 角度四元数：qz
	 */
	float qz = 0.0;
	/**
	 * @brief 角度四元数：qw
	 */
	float qw = 1.0;
	/**
	 * @brief 刚体追踪状态：true-成功；false-失败
	 */
	bool IsTrack = false;
	/**
	 * @brief 刚体追踪质量：1-好；2-中；3-差；
	 */
	int QualityGrade = 0;

	/**
	 * @brief 刚体速度；
	 */
	float fSpeed = 0.0;
	float fXSpeed = 0.0;
	float fYSpeed = 0.0;
	float fZSpeed = 0.0;
	/**
	 * @brief 刚体加速度；
	 */
	float fAcceleratedSpeed = 0.0;
	float fXAcceleratedSpeed = 0.0;
	float fYAcceleratedSpeed = 0.0;
	float fZAcceleratedSpeed = 0.0;
	/**
	 * @brief 刚体欧拉角；
	 */
	float fXEulerAngle = 0.0;
	float fYEulerAngle = 0.0;
	float fZEulerAngle = 0.0;
	/**
	 * @brief 刚体角速度；
	 */
	float fXPalstance = 0.0;
	float fYPalstance = 0.0;
	float fZPalstance = 0.0;
	/**
	 * @brief 刚体角加速度；
	 */
	float AccfXPalstance = 0.0;
	float AccfYPalstance = 0.0;
	float AccfZPalstance = 0.0;

}LST_RIGID_DATA;

/**
 * @brief  人体骨骼（节点）结构体，保存实时运动状态下的骨骼位置和姿态
 */
typedef struct tagStructJointData
{
	// Information
	int iJointID;
	std::string sJointName;
	// Pose data
	float X;
	float Y;
	float Z;
	float qx;
	float qy;
	float qz;
	float qw;
	float fConfidence;
	float fAngleX;
	float fAngleY;
	float fAngleZ;

	tagStructJointData() : iJointID(0),
		X(0.0f), Y(0.0f), Z(0.0f),
		qx(0.0f), qy(0.0f), qz(0.0f), qw(1.0f),fConfidence(0.0),
		fAngleX(0.0), fAngleY(0.0), fAngleZ(0.0)
	{

	}
}LST_SKELETON_JOINTDATA;

/**
 * @brief  人体结构体，保存实时运动状态下的人体的位置和姿态
 */
typedef struct tagStructSkeletonData
{
	uint32_t BodyID;
	std::string BodyName;
	
	std::vector<LST_SKELETON_JOINTDATA> vecJointNodes;

	// state
	bool IsTrack;
	//控制机器人数据转换后格式
	std::string sRobotName = "";  //机器人类型名称
	std::map<std::string, float> mapMortorAngle; //机器人电机运动对应的角度

	tagStructSkeletonData()
	{
		BodyID = 0;
		BodyName = "";
		vecJointNodes.clear();
		IsTrack = false;
	}
}LST_BODY_DATA;

/**
 * @brief  点集结构体
 */
typedef struct tagStructMarkerSetData 
{
	std::string sName = ""; //点集名称
	std::vector<LST_MarkerINFO> vmarkers; //点集内点

}LST_MARKER_SET;

/**
 * @brief  测力台数据结构体
 */
typedef struct tagStructForcePlateData
{
	float Fx = 0.f;
	float Fy = 0.f;
	float Fz = 0.f;
	float Mx = 0.f;
	float My = 0.f;
	float Mz = 0.f;
	float Lx = 0.f;
	float Lz = 0.f;

}LST_FORCEPLATE_DATA;

/**
 * @brief  时码数据结构体
 */
typedef struct tagComTimeCode
{
	unsigned long    mHours = 0;
	unsigned long    mMinutes = 0;
	unsigned long    mSeconds = 0;
	unsigned long    mFrames = 0;
	unsigned long    mSubFrame = 0;

}LST_TIMECODE_DATA;

/**
 * @brief  肌电数据结构体
 */
typedef struct tagStructElectromyographyData
{
	std::map<std::string, double> EmgData;
	tagStructElectromyographyData()
	{
		EmgData.clear();
	}
}LST_ELECTROMYOGRAPHY_DATA;

typedef struct tagStructCustomSkeletonData
{
	uint32_t Id;
	std::string sName;
	uint32_t type;
	std::vector<LST_SKELETON_JOINTDATA> vJointData;
	tagStructCustomSkeletonData()
	{
		
	}
}LST_CUSTOM_SKELETON;

typedef struct tagObjectInfo
{
	float X = 0.0;
	float Y = 0.0;

}LST_OBJECT_DATA;

/**
 * @brief  动作捕捉数据结构体，保存实时运动状态下的所有资产的位置和姿态
 */
struct LusterMocapData
{
	// Frame ID
	uint32_t FrameID;

	// Timestamp, no use
	unsigned long long TimeStamp;

	unsigned long long uCameraSyncTime;

	unsigned long long uBroadcastTime;

	// Marker Data
	std::vector<LST_MarkerINFO> Frame3DMarker;

	// Rigid body Data
	std::vector<LST_RIGID_DATA> FrameRigidBody;

	// Skeleton Data
	std::vector<LST_BODY_DATA> FrameBodysPose;

	// MarkerSet
	std::vector<LST_MARKER_SET> FrameMarkerSet;

	// TimeCode Data
	LST_TIMECODE_DATA TimeCode;
	
	// electromyography Data
	LST_ELECTROMYOGRAPHY_DATA ElectromyographyData;
	
	std::vector<LST_CUSTOM_SKELETON> FrameCustomSkeleton;

	// New ForcePlate
	std::map<int, LST_FORCEPLATE_DATA> FrameNewForcePlate;
	
	//cameraObject
	std::map<std::string, std::vector<LST_OBJECT_DATA>> FrameCameraObjects2D;
	LusterMocapData()
	{
		FrameID = 0;
		TimeStamp = 0;
		uCameraSyncTime = 0;
		uBroadcastTime = 0;
		Frame3DMarker.clear();
		FrameRigidBody.clear();
		FrameBodysPose.clear();
		FrameMarkerSet.clear();
		ElectromyographyData.EmgData.clear();
		FrameCustomSkeleton.clear();
		FrameNewForcePlate.clear();
		FrameCameraObjects2D.clear();
	}
	LusterMocapData(const std::string& data) { Parse(data); }
	
	void Parse(const std::string& data);
	// Test-首钢内容定制化传输
	void Parse_Rigid(const std::string& data);
};
typedef struct tagStructCamera
{
	/**
	 * @brief 相机Ip
	 */
	std::string Ip;
	/**
	 * @brief 相机序列号
	 */
	std::string Serial;
	/**
	 * @brief 相机型号
	 */
	std::string Model;
	/**
	 * @brief 相机曝光时间
	 */
	int Exposure = 0;
	/**
	 * @brief 相机增益
	 */
	int Gain = 0;
	/**
	 * @brief 相机帧率
	 */
	int FrameRate = 0;
	/**
	 * @brief 相机MTU值
	 */
	float MTU = 0;
	/**
	 * @brief 相机ID
	 */
	int Id = 0;
}LST_CAMERA_DATA;

struct LusterCameraList
{
	std::vector<LST_CAMERA_DATA> vCameraList;
	LusterCameraList()
	{
		vCameraList.clear();
	}
	LusterCameraList(const std::string& data) { Parse_CameraList(data); }

	void Parse_CameraList(const std::string& data);
};
LUSTERNET_SPACE_END
