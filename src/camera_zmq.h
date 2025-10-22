#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <opencv2/core.hpp>

#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>

#include <zmq.h>

// Lightweight JSON (header-only)
#include <nlohmann/json.hpp>

class CameraZmq : public rtabmap::Camera
{
public:
	CameraZmq(const std::string & endpoint, const std::string & topic = "camera.rgbd");
	~CameraZmq() override;

protected:
	bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "camera") override;
	rtabmap::SensorData captureImage(rtabmap::CameraInfo * info = nullptr) override;

public:
	// Retrieve a small JPEG thumbnail of the latest color frame (for kf_packet)
	bool getLatestColorThumb(std::vector<unsigned char> & jpgBytes, int maxWidth = 320);
	// Retrieve the latest known intrinsics
	bool getLatestIntrinsics(rtabmap::CameraModel & modelOut) const;

private:
	bool recvMessageParts(std::vector<std::vector<unsigned char>> & parts);
	static bool decodeRgbJpeg(const std::vector<unsigned char> & buf, cv::Mat & rgbBgrOut);
	static bool decodeDepthPng16(const std::vector<unsigned char> & buf, cv::Mat & depthU16Out);

private:
	std::string endpoint_;
	std::string topic_;
	void * ctx_ {nullptr};
	void * sub_ {nullptr};
	bool initialized_ {false};

	// Last-known camera model and color frame for auxiliary publishing
	mutable std::mutex lastMutex_;
	rtabmap::CameraModel lastModel_;
	cv::Mat lastColorBgr_;
	double lastStampSec_ {0.0};
	// Depth units (meters per uint16 unit) from the publisher metadata
	double depthUnitsM_ {0.001};
};


