#pragma once

#include <string>
#include <vector>
#include <mutex>

#include <opencv2/core.hpp>

#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/IMU.h>

#include <zmq.h>

// Lightweight JSON (header-only)
#include <nlohmann/json.hpp>

class CameraZmq : public rtabmap::Camera
{
public:
	CameraZmq(const std::string & endpoint, const std::string & topic = "camera.rgbd");
	~CameraZmq() override;

	// Public wrapper for initialization
	bool initialize() { return init(); }

protected:
	bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "camera") override;
	rtabmap::SensorData captureImage(rtabmap::SensorCaptureInfo * info = nullptr) override;
	std::string getSerial() const override { return "zmq_camera"; }
	bool isCalibrated() const override { return true; }

public:
	// Retrieve a small JPEG thumbnail of the latest color frame (for kf_packet)
	bool getLatestColorThumb(std::vector<unsigned char> & jpgBytes, int maxWidth = 320);
	// Retrieve the latest known intrinsics
	bool getLatestIntrinsics(rtabmap::CameraModel & modelOut) const;
	// Retrieve latest timestamp and depth units
	bool getLatestMeta(uint64_t & tsNsOut, double & depthUnitsMOut) const;

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
	// Last source timestamp in ns
	uint64_t lastTsNs_ {0};
	// IMU data
	cv::Vec3d lastAccel_ {0, 0, 0};  // accelerometer [ax, ay, az] m/s^2
	cv::Vec3d lastGyro_ {0, 0, 0};   // gyroscope [gx, gy, gz] rad/s
	bool hasImu_ {false};
};


