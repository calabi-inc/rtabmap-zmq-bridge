#include "camera_zmq.h"

#include <cstdlib>
#include <string>
#include <unordered_map>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rtabmap/core/SensorCaptureThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UConversion.h>

#include <zmq.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

static std::atomic<bool> g_running(true);

void signalHandler(int signum) {
	std::cout << "[bridge] caught signal " << signum << ", shutting down..." << std::endl;
	g_running = false;
}

static inline void transformToXYZRPY(const rtabmap::Transform & T, double & x, double & y, double & z, double & roll, double & pitch, double & yaw)
{
	x = T.x();
	y = T.y();
	z = T.z();
	float r, p, yw;
	T.getEulerAngles(r, p, yw);
	roll = (double)r;
	pitch = (double)p;
	yaw = (double)yw;
}

static inline void transformToTQ(const rtabmap::Transform & T, double t[3], double q[4])
{
	t[0] = T.x(); t[1] = T.y(); t[2] = T.z();
	Eigen::Quaternionf quat = T.getQuaternionf();
	q[0] = (double)quat.x();
	q[1] = (double)quat.y();
	q[2] = (double)quat.z();
	q[3] = (double)quat.w();
}

class BridgePublisher : public UEventsHandler
{
public:
	BridgePublisher(const std::string & pubEndpoint, CameraZmq * cam, rtabmap::Rtabmap * core, bool publishFullRGBD = false)
		: cam_(cam), rtabmap_(core), publishFullRGBD_(publishFullRGBD)
	{
		std::cout << "[bridge] initializing publisher at " << pubEndpoint << std::endl;
		ctx_ = zmq_ctx_new();
		pub_ = zmq_socket(ctx_, ZMQ_PUB);
		zmq_bind(pub_, pubEndpoint.c_str());
		std::cout << "[bridge] publisher bound (full_rgbd=" << (publishFullRGBD_ ? "true" : "false") << ")" << std::endl;
	}
	~BridgePublisher() override
	{
		if (pub_) { zmq_close(pub_); pub_ = nullptr; }
		if (ctx_) { zmq_ctx_term(ctx_); ctx_ = nullptr; }
	}

protected:
	bool handleEvent(UEvent * event) override
	{
		if (event->getClassName().compare("OdometryEvent") == 0) {
			const rtabmap::OdometryEvent * e = (const rtabmap::OdometryEvent *)event;
			if (e->pose().isNull()) {
				std::cout << "[odometry] WARNING: null pose!" << std::endl;
				return false;
			}
			publishTrackingPose(e->pose(), e->info().stamp);
			return true;
		}
		else if (event->getClassName().compare("RtabmapEvent") == 0) {
			const rtabmap::RtabmapEvent * e = (const rtabmap::RtabmapEvent *)event;
			const rtabmap::Statistics & s = e->getStats();

			static int rtabmapCount = 0;
			static int lastPublishedRefId = 0;
			static rtabmap::Transform lastPublishedPose;
			int currentRefId = s.refImageId();
			int wmSize = s.poses().size();

			// Get current pose
			rtabmap::Transform currentPose;
			auto it = s.poses().find(currentRefId);
			if (it != s.poses().end()) {
				currentPose = it->second;
			}

			// Check if this is a REAL new keyframe worth publishing:
			// 1. refId changed (new node created)
			// 2. Node is actually in the poses map (wasn't immediately deleted)
			// 3. Sufficient displacement from last PUBLISHED keyframe
			bool isNewNode = (currentRefId > 0 && currentRefId != lastPublishedRefId);
			bool isInGraph = !currentPose.isNull();

			// Calculate displacement from last PUBLISHED keyframe (not last processed frame)
			double transDist = 0.0, rotAngle = 0.0;
			if (!lastPublishedPose.isNull() && !currentPose.isNull()) {
				transDist = currentPose.getDistance(lastPublishedPose);
				rotAngle = currentPose.getAngle(lastPublishedPose);
			}

			// Apply our own keyframe threshold check (since RTAB-Map's may not work as expected)
			const double linearThreshold = 0.1;   // 10cm
			const double angularThreshold = 0.175; // ~10 degrees
			bool sufficientMotion = lastPublishedPose.isNull() ||
			                        transDist >= linearThreshold ||
			                        rotAngle >= angularThreshold;

			bool shouldPublish = isNewNode && isInGraph && sufficientMotion;

			// Logging
			if (rtabmapCount % 30 == 0 || shouldPublish) {
				std::cout << "[rtabmap] refId=" << currentRefId
				          << " wmSize=" << wmSize
				          << " loopId=" << s.loopClosureId()
				          << " d_trans=" << transDist << "m"
				          << " d_rot=" << rotAngle << "rad";
				if (shouldPublish) {
					std::cout << " >>> KEYFRAME PUBLISHED <<<";
				}
				std::cout << std::endl;
			}
			rtabmapCount++;

			// Always publish map correction and tracking updates
			publishMapCorrection(s);

			// Only publish keyframe data when we have sufficient motion
			if (shouldPublish) {
				lastPublishedRefId = currentRefId;
				lastPublishedPose = currentPose;
				publishKeyframePacketIfNew(s);
				publishKeyframeUpdates(s);
			}

			return true;
		}
		return false;
	}

private:
	void publishTrackingPose(const rtabmap::Transform & T_wc, double stampMs)
	{
		double x, y, z, roll, pitch, yaw;
		transformToXYZRPY(T_wc, x, y, z, roll, pitch, yaw);
		json msg;
		msg["stamp_ms"] = stampMs;
		msg["frame"] = "world";
		msg["child"] = "camera";
		msg["T_wc"] = { x, y, z, roll, pitch, yaw };
		sendMultipart("rtabmap.tracking_pose", msg.dump());
	}

	static bool poseChanged(const rtabmap::Transform & a, const rtabmap::Transform & b)
	{
		if (a.isNull() || b.isNull()) return true;
		const double transThresh = 1e-3; // 1 mm
		const double rotThresh = 1e-3;   // ~0.06 deg
		double ax, ay, az, ar, ap, ayw;
		double bx, by, bz, br, bp, byw;
		transformToXYZRPY(a, ax, ay, az, ar, ap, ayw);
		transformToXYZRPY(b, bx, by, bz, br, bp, byw);
		double dt = std::sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by) + (az-bz)*(az-bz));
		double dr = std::fabs(ar-br) + std::fabs(ap-bp) + std::fabs(ayw-byw);
		return dt > transThresh || dr > rotThresh;
	}

	bool encodeThumb(const cv::Mat & bgr, std::vector<unsigned char> & outJpg, int maxWidth = 320)
	{
		if (bgr.empty()) return false;
		cv::Mat img = bgr;
		if (bgr.cols > maxWidth) {
			int newH = (int)((double)bgr.rows * (double)maxWidth / (double)bgr.cols);
			cv::resize(bgr, img, cv::Size(maxWidth, newH));
		}
		std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
		return cv::imencode(".jpg", img, outJpg, params);
	}

	void publishMapCorrection(const rtabmap::Statistics & s)
	{
		const rtabmap::Transform & T_w_odom = s.mapCorrection();
		if (T_w_odom.isNull()) return;
		json msg;
		// timestamp: prefer KF stamp or camera meta
		uint64_t ts_ns = 0; double depthUnits=0.0;
		if (cam_ && cam_->getLatestMeta(ts_ns, depthUnits)) {
			msg["ts_ns"] = ts_ns;
		}
		int refId = s.refImageId();
		if ((!msg.contains("ts_ns")) && rtabmap_ && rtabmap_->getMemory() && refId>0) {
			try {
				rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(refId, true, false, false, false);
				double stampMs = data.stamp();
				if (stampMs > 0.0) msg["ts_ns"] = (uint64_t)(stampMs * 1e6);
			} catch (...) {}
		}
		// map id from signature if available
		msg["map_id"] = 0;
		if (rtabmap_ && rtabmap_->getMemory() && refId>0) {
			const rtabmap::Signature * sig = rtabmap_->getMemory()->getSignature(refId);
			if (sig) msg["map_id"] = sig->mapId();
		}
		double t[3]; double q[4];
		transformToTQ(T_w_odom, t, q);
		msg["T_w_odom"] = { {"t", {t[0],t[1],t[2]}}, {"q", {q[0],q[1],q[2],q[3]}} };
		sendMultipart("rtabmap.map_correction", msg.dump());
	}

	void publishKeyframePacketIfNew(const rtabmap::Statistics & s)
	{
		int refId = s.refImageId();
		if (refId <= 0) return;
		rtabmap::Transform T_w_c;
		auto it = s.poses().find(refId);
		if (it != s.poses().end()) {
			T_w_c = it->second;
		}
		else if (!s.mapCorrection().isNull()) {
			if (rtabmap_ && rtabmap_->getMemory()) {
				rtabmap::Transform odom;
				int mapIdTmp=0, weight=0; std::string label; double stamp=0.0; rtabmap::Transform gt; std::vector<float> vel; rtabmap::GPS gps; rtabmap::EnvSensors sensors;
				if (rtabmap_->getMemory()->getNodeInfo(refId, odom, mapIdTmp, weight, label, stamp, gt, vel, gps, sensors)) {
					T_w_c = s.mapCorrection() * odom;
				}
			}
		}

		json pkt;
		cv::Mat bgr, depth;
		if (rtabmap_ && rtabmap_->getMemory()) {
			try {
				// Fetch images if publishFullRGBD is enabled
				rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(refId, publishFullRGBD_, false, false, false);
				if (!data.imageRaw().empty()) bgr = data.imageRaw().clone();
				if (publishFullRGBD_ && !data.depthRaw().empty()) {
					depth = data.depthRaw().clone();
				}
				if (data.cameraModels().size() > 0) {
					const rtabmap::CameraModel & m = data.cameraModels()[0];
					pkt["intrinsics"] = {
						{"fx", m.fx()},
						{"fy", m.fy()},
						{"cx", m.cx()},
						{"cy", m.cy()},
						{"width", m.imageWidth()},
						{"height", m.imageHeight()},
						{"baseline", 0.0}
					};
				}
				double stampMs = data.stamp();
				if (stampMs > 0.0) {
					uint64_t ts_ns2 = (uint64_t)(stampMs * 1e6);
					pkt["ts_ns"] = ts_ns2;
				}
			} catch (...) {}

			const rtabmap::Signature * sig = rtabmap_->getMemory()->getSignature(refId);
			if (sig) {
				pkt["map_id"] = sig->mapId();
			}
		}
		uint64_t tsNsTmp=0; double depthUnits=0.0;
		if (cam_ && cam_->getLatestMeta(tsNsTmp, depthUnits)) {
			pkt["depth_units_m"] = depthUnits;
		}
		double t[3]; double q[4];
		transformToTQ(T_w_c, t, q);
		pkt["T_w_c"] = { {"t", {t[0],t[1],t[2]}}, {"q", {q[0],q[1],q[2],q[3]}} };
		pkt["kf_id"] = refId;

		if (publishFullRGBD_) {
			// Full RGB-D mode: encode full-resolution images
			pkt["rgb"] = { {"uri", ""}, {"encoding", "jpeg"} };
			pkt["depth"] = { {"uri", ""}, {"encoding", "png_u16"} };

			std::vector<unsigned char> rgbJpg, depthPng;
			bool hasRgb = false, hasDepth = false;

			// Encode RGB as JPEG (85% quality for full-res)
			if (!bgr.empty()) {
				std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
				hasRgb = cv::imencode(".jpg", bgr, rgbJpg, params);
			}

			// Encode depth as PNG (16-bit, convert from float32 to uint16)
			if (!depth.empty()) {
				cv::Mat depthU16;
				if (depth.type() == CV_32FC1) {
					// Convert float depth (meters) back to uint16
					double scale = 1.0 / depthUnits;
					depth.convertTo(depthU16, CV_16UC1, scale);
				} else if (depth.type() == CV_16UC1) {
					depthU16 = depth;
				}
				if (!depthU16.empty()) {
					hasDepth = cv::imencode(".png", depthU16, depthPng);
				}
			}

			// Send with both images
			if (hasRgb && hasDepth) {
				sendMultipart("rtabmap.kf_packet", pkt.dump(), &rgbJpg, &depthPng);
			} else if (hasRgb) {
				sendMultipart("rtabmap.kf_packet", pkt.dump(), &rgbJpg);
			} else {
				sendMultipart("rtabmap.kf_packet", pkt.dump());
			}
		} else {
			// Thumbnail mode (default): only small RGB thumbnail
			pkt["rgb"] = { {"uri", ""}, {"encoding", "jpeg"} };
			pkt["depth"] = { {"uri", ""}, {"encoding", "png_u16"} };

			std::vector<unsigned char> jpg;
			if (!bgr.empty() && encodeThumb(bgr, jpg)) {
				sendMultipart("rtabmap.kf_packet", pkt.dump(), &jpg);
			} else {
				sendMultipart("rtabmap.kf_packet", pkt.dump());
			}
		}
	}

	void publishKeyframeUpdates(const rtabmap::Statistics & s)
	{
		const std::map<int, rtabmap::Transform> & poses = s.poses();
		for (const auto & kv : poses) {
			int id = kv.first;
			const rtabmap::Transform & T = kv.second;
			bool isNew = lastPoses_.find(id) == lastPoses_.end();
			bool changed = (!isNew && poseChanged(T, lastPoses_[id]));
			if (isNew) {
				json poseMsg;
				double x, y, z, roll, pitch, yaw;
				transformToXYZRPY(T, x, y, z, roll, pitch, yaw);
				poseMsg["kf_id"] = id;
				poseMsg["T_wc"] = { x, y, z, roll, pitch, yaw };
				sendMultipart("rtabmap.kf_pose", poseMsg.dump());
				std::cout << "[rtabmap] new keyframe kf_id=" << id
					<< " T_wc=[" << x << "," << y << "," << z << ","
					<< roll << "," << pitch << "," << yaw << "]" << std::endl;
			}
			else if (changed) {
				double x, y, z, roll, pitch, yaw;
				transformToXYZRPY(T, x, y, z, roll, pitch, yaw);
				json upd;
				upd["kf_id"] = id;
				upd["T_wc"] = { x, y, z, roll, pitch, yaw };
				sendMultipart("rtabmap.kf_pose_update", upd.dump());
			}
			lastPoses_[id] = T;
		}
	}

	void sendMultipart(const std::string & topic, const std::string & jsonStr, const std::vector<unsigned char> * bin1 = nullptr, const std::vector<unsigned char> * bin2 = nullptr)
	{
		zmq_msg_t t;
		zmq_msg_init_size(&t, topic.size());
		memcpy(zmq_msg_data(&t), topic.data(), topic.size());
		zmq_msg_send(&t, pub_, bin1 || bin2 || !jsonStr.empty() ? ZMQ_SNDMORE : 0);
		zmq_msg_close(&t);

		if (!jsonStr.empty()) {
			zmq_msg_t j;
			zmq_msg_init_size(&j, jsonStr.size());
			memcpy(zmq_msg_data(&j), jsonStr.data(), jsonStr.size());
			zmq_msg_send(&j, pub_, bin1 || bin2 ? ZMQ_SNDMORE : 0);
			zmq_msg_close(&j);
		}

		if (bin1) {
			zmq_msg_t b1;
			zmq_msg_init_size(&b1, bin1->size());
			memcpy(zmq_msg_data(&b1), bin1->data(), bin1->size());
			zmq_msg_send(&b1, pub_, bin2 ? ZMQ_SNDMORE : 0);
			zmq_msg_close(&b1);
		}

		if (bin2) {
			zmq_msg_t b2;
			zmq_msg_init_size(&b2, bin2->size());
			memcpy(zmq_msg_data(&b2), bin2->data(), bin2->size());
			zmq_msg_send(&b2, pub_, 0);
			zmq_msg_close(&b2);
		}
	}

private:
	void * ctx_ {nullptr};
	void * pub_ {nullptr};
	CameraZmq * cam_ {nullptr};
	rtabmap::Rtabmap * rtabmap_ {nullptr};
	bool publishFullRGBD_ {false};
	std::unordered_map<int, rtabmap::Transform> lastPoses_;
};

int main(int argc, char ** argv)
{
	(void)argc; (void)argv;
	const char * sub = std::getenv("SUB");
	const char * pub = std::getenv("PUB");
	std::string subEp = sub ? sub : "tcp://host.docker.internal:5555";
	std::string pubEp = pub ? pub : "tcp://*:6000";

	// Setup signal handlers
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	std::cout << "[bridge] starting with SUB=" << subEp << " PUB=" << pubEp << std::endl;

	std::cout << "[bridge] creating camera..." << std::endl;
	CameraZmq * cam = new CameraZmq(subEp);

	// Initialize camera before creating thread
	if (!cam->initialize()) {
		std::cerr << "[bridge] failed to initialize camera" << std::endl;
		return 1;
	}

	rtabmap::SensorCaptureThread cameraThread(cam);

	std::cout << "[bridge] creating odometry..." << std::endl;
	// Create odometry using F2M (Frame-to-Map) which is good for RGB-D
	rtabmap::Odometry * odom = rtabmap::Odometry::create();
	rtabmap::OdometryThread odomThread(odom);

	std::cout << "[bridge] creating rtabmap core/thread..." << std::endl;
	rtabmap::Rtabmap * rtabmap = new rtabmap::Rtabmap();

	// RTAB-Map parameters
	rtabmap::ParametersMap params;

	// RGB-D SLAM mode
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "true"));

	// KEYFRAME SELECTION - These control when a new node is added to the graph
	// A new node is added when movement from last node exceeds EITHER threshold
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDLinearUpdate(), "0.1"));   // 10cm translation
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDAngularUpdate(), "0.175")); // ~10 degrees rotation

	// Memory management (using RTAB-Map defaults from Table 2)
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "true"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemSTMSize(), "30"));           // default: 30 nodes
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemRehearsalSimilarity(), "0.2")); // default: 0.2

	// Disable time-based constraints
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), "0"));
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapCreateIntermediateNodes(), "false"));

	// Initialize RTAB-Map with empty database (in-memory only)
	rtabmap->init(params, "");
	std::cout << "[bridge] RTAB-Map initialized in mapping mode" << std::endl;

	rtabmap::RtabmapThread rtabmapThread(rtabmap);

	// Set detection rate (RTAB-Map default is 2 Hz)
	rtabmapThread.setDetectorRate(2.0f);  // Process at most 2 Hz
	rtabmapThread.setDataBufferSize(1);   // Keep only latest frame in buffer

	// Verify parameters
	std::cout << "[bridge] RTAB-Map parameters:" << std::endl;
	const rtabmap::ParametersMap & actualParams = rtabmap->getParameters();
	auto findParam = [&](const std::string& key) -> std::string {
		auto it = actualParams.find(key);
		return (it != actualParams.end()) ? it->second : "NOT_SET";
	};
	std::cout << "  RGBD/Enabled: " << findParam(rtabmap::Parameters::kRGBDEnabled()) << std::endl;
	std::cout << "  RGBD/LinearUpdate: " << findParam(rtabmap::Parameters::kRGBDLinearUpdate()) << " m" << std::endl;
	std::cout << "  RGBD/AngularUpdate: " << findParam(rtabmap::Parameters::kRGBDAngularUpdate()) << " rad" << std::endl;
	std::cout << "  Mem/RehearsalSimilarity: " << findParam(rtabmap::Parameters::kMemRehearsalSimilarity()) << std::endl;
	std::cout << "  Mem/STMSize: " << findParam(rtabmap::Parameters::kMemSTMSize()) << std::endl;

	// Read FULL_RGBD environment variable (default: false for thumbnail only)
	const char * fullRgbdEnv = std::getenv("FULL_RGBD");
	bool publishFullRGBD = (fullRgbdEnv && std::string(fullRgbdEnv) == "true");

	std::cout << "[bridge] creating publisher..." << std::endl;
	BridgePublisher publisher(pubEp, cam, rtabmap, publishFullRGBD);

	// Register event handlers (order matters: register before starting threads)
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	UEventsManager::addHandler(&publisher);

	// Create event pipe: camera -> odometry (following official RTAB-Map example)
	// Pipeline: Camera -> OdometryThread -> RtabmapThread
	UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

	std::cout << "[bridge] Event pipes configured:" << std::endl;
	std::cout << "  camera -> odometry -> rtabmap" << std::endl;

	std::cout << "[bridge] starting threads..." << std::endl;
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();
	std::cout << "[bridge] threads started, waiting for data..." << std::endl;

	// Keep running until signal received
	while (g_running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// Check if camera thread is still alive
		if (!cameraThread.isRunning()) {
			std::cerr << "[bridge] camera thread died, exiting" << std::endl;
			break;
		}
	}

	std::cout << "[bridge] shutting down threads..." << std::endl;
	cameraThread.join(true);
	odomThread.join(true);
	rtabmapThread.join(true);

	std::cout << "[bridge] shutdown complete" << std::endl;
	return 0;
}


