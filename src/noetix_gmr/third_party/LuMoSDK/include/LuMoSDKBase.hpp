/***********************************************************************
*
* Copyright (c) 2021-2022 Luster LightTech(Beijing) Co.Ltd.
* All Rights Reserved.
*
*
* FILE NAME		: LusterSDKBase.hpp
* DESCRIPTION	: SDK API.
*
* VERDION	: 1.0.0
* DATE		: 2022/04/10
* AUTHOR	: jzy
* 
***********************************************************************/
#pragma once


#include <cstddef>
#include <memory>
#include "LuMoDataStruct.hpp"

#define LUSTERNET_SPACE_BEGIN namespace lusternet {
#define LUSTERNET_SPACE_END }
#define LUSTERNET_SPACE using namespace lusternet;

LUSTERNET_SPACE_BEGIN

class CReceiveBase
{
public:
	// 0.析构
	virtual ~CReceiveBase() {}

	// 1.连接服务端
	virtual void Connect(const std::string& ip) = 0;

	// 2.连接初始化
	virtual void Init() = 0;

	// 3.获取相机列表
	virtual void GetCameraList(LusterCameraList& cameraList) = 0;

	// 4.与服务端断开连接
	virtual void Disconnect(const std::string& ip) = 0;

	// 5.连接状态
	virtual bool IsConnected() = 0;

	// 6.动捕数据接收：ptrData-散点，刚体，人体，帧ID，时间戳
	//   RecState-数据接收状态，0-阻塞状态，1-非阻塞状态
	virtual void ReceiveData(LusterMocapData& ptrData, int RecState = 0) = 0;

	// 7.关闭连接服务
	virtual void Close() = 0;

	
};

// SDK变量构造方法，可直接调用
__attribute__((visibility("default"))) std::shared_ptr<CReceiveBase> getFZReceive();

LUSTERNET_SPACE_END
