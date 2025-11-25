#include "camera_zmq.h"

#include <chrono>
#include <stdexcept>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UConversion.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

CameraZmq::CameraZmq(const std::string & endpoint, const std::string & topic)
	: endpoint_(endpoint), topic_(topic)
{
}

CameraZmq::~CameraZmq()
{
	if (sub_) {
		zmq_close(sub_);
		sub_ = nullptr;
	}
	if (ctx_) {
		zmq_ctx_term(ctx_);
		ctx_ = nullptr;
	}
}

bool CameraZmq::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	(void)calibrationFolder;
	(void)cameraName;
	std::cout << "[camera_zmq] initializing, endpoint=" << endpoint_ << " topic=" << topic_ << std::endl;
	ctx_ = zmq_ctx_new();
	if (!ctx_) {
		std::cerr << "[camera_zmq] failed to create ZMQ context" << std::endl;
		return false;
	}
	sub_ = zmq_socket(ctx_, ZMQ_SUB);
	if (!sub_) {
		std::cerr << "[camera_zmq] failed to create ZMQ socket" << std::endl;
		return false;
	}
	if (zmq_connect(sub_, endpoint_.c_str()) != 0) {
		std::cerr << "[camera_zmq] failed to connect to " << endpoint_ << std::endl;
		return false;
	}
	if (zmq_setsockopt(sub_, ZMQ_SUBSCRIBE, topic_.data(), (int)topic_.size()) != 0) {
		std::cerr << "[camera_zmq] failed to subscribe to topic " << topic_ << std::endl;
		return false;
	}
	// Use short timeout and retry loop instead of blocking forever
	int timeoutMs = 1000;
	zmq_setsockopt(sub_, ZMQ_RCVTIMEO, &timeoutMs, sizeof(timeoutMs));
	initialized_ = true;
	std::cout << "[camera_zmq] initialized successfully" << std::endl;
	return true;
}

static bool recv_one(void * sock, std::vector<unsigned char> & out, bool & more)
{
	more = false;
	int64_t more_i = 0;
	size_t more_sz = sizeof(more_i);
	zmq_msg_t msg;
	zmq_msg_init(&msg);
	int rc = zmq_msg_recv(&msg, sock, 0);
	if (rc == -1) {
		zmq_msg_close(&msg);
		return false;
	}
	unsigned char * data = static_cast<unsigned char*>(zmq_msg_data(&msg));
	size_t size = zmq_msg_size(&msg);
	out.assign(data, data + size);
	zmq_msg_close(&msg);
	zmq_getsockopt(sock, ZMQ_RCVMORE, &more_i, &more_sz);
	more = more_i != 0;
	return true;
}

bool CameraZmq::recvMessageParts(std::vector<std::vector<unsigned char>> & parts)
{
	parts.clear();
	bool more = false;
	std::vector<unsigned char> buf;
	if (!recv_one(sub_, buf, more)) return false;
	parts.emplace_back(std::move(buf));
	while (more) {
		std::vector<unsigned char> part;
		if (!recv_one(sub_, part, more)) return false;
		parts.emplace_back(std::move(part));
	}
	return true;
}

bool CameraZmq::decodeRgbJpeg(const std::vector<unsigned char> & buf, cv::Mat & rgbBgrOut)
{
	cv::Mat encoded(1, (int)buf.size(), CV_8UC1, const_cast<unsigned char*>(buf.data()));
	rgbBgrOut = cv::imdecode(encoded, cv::IMREAD_COLOR);
	return !rgbBgrOut.empty();
}

bool CameraZmq::decodeDepthPng16(const std::vector<unsigned char> & buf, cv::Mat & depthU16Out)
{
	cv::Mat encoded(1, (int)buf.size(), CV_8UC1, const_cast<unsigned char*>(buf.data()));
	depthU16Out = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
	return !depthU16Out.empty() && depthU16Out.type() == CV_16UC1;
}

rtabmap::SensorData CameraZmq::captureImage(rtabmap::SensorCaptureInfo * info)
{
	(void)info;
	if (!initialized_) {
		std::cerr << "[camera_zmq] not initialized" << std::endl;
		return rtabmap::SensorData();
	}

	// Keep trying until we get data - don't return empty on timeout
	std::vector<std::vector<unsigned char>> parts;
	static int retryCount = 0;
	while (!recvMessageParts(parts)) {
		if (retryCount % 5 == 0) {  // Log every 5 seconds
			std::cout << "[camera_zmq] waiting for message (is publisher running?)..." << std::endl;
		}
		retryCount++;
		// Continue retrying instead of returning empty
	}
	retryCount = 0;
	// std::cout << "[camera_zmq] received message with " << parts.size() << " parts" << std::endl;
	if (parts.size() < 4) {
		return rtabmap::SensorData();
	}
	std::string topic(reinterpret_cast<const char*>(parts[0].data()), parts[0].size());
	if (topic != topic_) {
		return rtabmap::SensorData();
	}
	json meta;
	try {
		meta = json::parse(parts[1]);
	} catch (...) {
		return rtabmap::SensorData();
	}
	const uint64_t ts_ns = meta.value("ts_ns", (uint64_t)0);
	const double stampMs = ts_ns > 0 ? (double)ts_ns * 1e-6 : 0.0; // ms
	const auto intr = meta["intrinsics"];
	const float fx = intr.value("fx", 0.0f);
	const float fy = intr.value("fy", 0.0f);
	const float cx = intr.value("cx", 0.0f);
	const float cy = intr.value("cy", 0.0f);
	const int width = intr.value("width", 0);
	const int height = intr.value("height", 0);
	depthUnitsM_ = meta.value("depth_units_m", 0.001);

	cv::Mat colorBgr;
	if (!decodeRgbJpeg(parts[2], colorBgr)) {
		return rtabmap::SensorData();
	}
	cv::Mat depthU16;
	if (!decodeDepthPng16(parts[3], depthU16)) {
		return rtabmap::SensorData();
	}

	cv::Mat depth32f;
	depthU16.convertTo(depth32f, CV_32FC1, depthUnitsM_);

	// Debug: Check depth data validity
	static int debugCount = 0;
	if (debugCount % 30 == 0) {
		double minVal, maxVal;
		cv::minMaxLoc(depthU16, &minVal, &maxVal);
		int validPixels = cv::countNonZero(depthU16);
		std::cout << "[camera_zmq] Depth check: min=" << minVal << " max=" << maxVal
		          << " validPixels=" << validPixels << "/" << (depthU16.rows * depthU16.cols)
		          << " (" << (100.0 * validPixels / (depthU16.rows * depthU16.cols)) << "%)" << std::endl;
	}
	debugCount++;

	rtabmap::CameraModel model("rs", fx, fy, cx, cy, rtabmap::Transform::getIdentity(), 0.0, cv::Size(width, height));

	{
		std::lock_guard<std::mutex> lock(lastMutex_);
		lastModel_ = model;
		lastColorBgr_ = colorBgr.clone();
		lastStampSec_ = stampMs * 1e-3;
		lastTsNs_ = ts_ns;
	}

	static int frameCount = 0;
	if (frameCount < 5 || frameCount % 30 == 0) {
		std::cout << "[camera_zmq] Producing SensorData frame #" << frameCount
		          << " (" << colorBgr.cols << "x" << colorBgr.rows << ")" << std::endl;
	}
	frameCount++;

	return rtabmap::SensorData(colorBgr, depth32f, model, 0, stampMs);
}

bool CameraZmq::getLatestColorThumb(std::vector<unsigned char> & jpgBytes, int maxWidth)
{
	cv::Mat img;
	{
		std::lock_guard<std::mutex> lock(lastMutex_);
		if (lastColorBgr_.empty()) return false;
		img = lastColorBgr_.clone();
	}
	int w = img.cols;
	int h = img.rows;
	if (w > maxWidth && w > 0) {
		int newH = (int)((double)h * (double)maxWidth / (double)w);
		cv::resize(img, img, cv::Size(maxWidth, newH));
	}
	std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
	return cv::imencode(".jpg", img, jpgBytes, params);
}

bool CameraZmq::getLatestIntrinsics(rtabmap::CameraModel & modelOut) const
{
	std::lock_guard<std::mutex> lock(lastMutex_);
	if (lastModel_.fx() <= 0.0f || lastModel_.fy() <= 0.0f) return false;
	modelOut = lastModel_;
	return true;
}

bool CameraZmq::getLatestMeta(uint64_t & tsNsOut, double & depthUnitsMOut) const
{
	std::lock_guard<std::mutex> lock(lastMutex_);
	if (lastTsNs_ == 0) return false;
	tsNsOut = lastTsNs_;
	depthUnitsMOut = depthUnitsM_;
	return true;
}


