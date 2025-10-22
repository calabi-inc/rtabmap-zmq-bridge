#include "camera_zmq.h"

#include <cstdlib>
#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>

#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryF2M.h>
#include <rtabmap/core/OdometryThread.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UConversion.h>

#include <zmq.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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

class BridgePublisher : public UEventsHandler
{
public:
	BridgePublisher(const std::string & pubEndpoint, CameraZmq * cam)
		: cam_(cam)
	{
		ctx_ = zmq_ctx_new();
		pub_ = zmq_socket(ctx_, ZMQ_PUB);
		zmq_bind(pub_, pubEndpoint.c_str());
	}
	~BridgePublisher() override
	{
		if (pub_) { zmq_close(pub_); pub_ = nullptr; }
		if (ctx_) { zmq_ctx_term(ctx_); ctx_ = nullptr; }
	}

protected:
	void handleEvent(UEvent * event) override
	{
		if (event->getClassName().compare("OdometryEvent") == 0) {
			const rtabmap::OdometryEvent * e = (const rtabmap::OdometryEvent *)event;
			if (e->pose().isNull()) return;
			publishTrackingPose(e->pose(), e->info().stamp);
		}
		else if (event->getClassName().compare("RtabmapEvent") == 0) {
			const rtabmap::RtabmapEvent * e = (const rtabmap::RtabmapEvent *)event;
			const rtabmap::Statistics & s = e->getStats();
			publishKeyframeUpdates(s);
		}
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
		sendMultipart("slam.tracking_pose", msg.dump());
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

	void publishKeyframeUpdates(const rtabmap::Statistics & s)
	{
		const std::map<int, rtabmap::Transform> & poses = s.poses();
		for (const auto & kv : poses) {
			int id = kv.first;
			const rtabmap::Transform & T = kv.second;
			bool isNew = lastPoses_.find(id) == lastPoses_.end();
			bool changed = (!isNew && poseChanged(T, lastPoses_[id]));
			if (isNew) {
				// kf_packet with thumb + intrinsics
				json pkt;
				rtabmap::CameraModel model;
				if (cam_ && cam_->getLatestIntrinsics(model)) {
					pkt["intrinsics"] = {
						{"fx", model.fx()},
						{"fy", model.fy()},
						{"cx", model.cx()},
						{"cy", model.cy()},
						{"width", model.imageWidth()},
						{"height", model.imageHeight()}
					};
				}
				double x, y, z, roll, pitch, yaw;
				transformToXYZRPY(T, x, y, z, roll, pitch, yaw);
				pkt["kf_id"] = id;
				pkt["T_wc"] = { x, y, z, roll, pitch, yaw };
				// thumb
				std::vector<unsigned char> jpg;
				if (cam_ && cam_->getLatestColorThumb(jpg)) {
					// multipart with binary following JSON
					sendMultipart("slam.kf_packet", pkt.dump(), &jpg);
				} else {
					sendMultipart("slam.kf_packet", pkt.dump());
				}
				// Also kf_pose
				json poseMsg;
				poseMsg["kf_id"] = id;
				poseMsg["T_wc"] = { x, y, z, roll, pitch, yaw };
				sendMultipart("slam.kf_pose", poseMsg.dump());
			}
			else if (changed) {
				double x, y, z, roll, pitch, yaw;
				transformToXYZRPY(T, x, y, z, roll, pitch, yaw);
				json upd;
				upd["kf_id"] = id;
				upd["T_wc"] = { x, y, z, roll, pitch, yaw };
				sendMultipart("slam.kf_pose_update", upd.dump());
			}
			lastPoses_[id] = T;
		}
	}

	void sendMultipart(const std::string & topic, const std::string & jsonStr, const std::vector<unsigned char> * bin = nullptr)
	{
		zmq_msg_t t;
		zmq_msg_init_size(&t, topic.size());
		memcpy(zmq_msg_data(&t), topic.data(), topic.size());
		zmq_msg_send(&t, pub_, bin || !jsonStr.empty() ? ZMQ_SNDMORE : 0);
		zmq_msg_close(&t);

		if (!jsonStr.empty()) {
			zmq_msg_t j;
			zmq_msg_init_size(&j, jsonStr.size());
			memcpy(zmq_msg_data(&j), jsonStr.data(), jsonStr.size());
			zmq_msg_send(&j, pub_, bin ? ZMQ_SNDMORE : 0);
			zmq_msg_close(&j);
		}

		if (bin) {
			zmq_msg_t b;
			zmq_msg_init_size(&b, bin->size());
			memcpy(zmq_msg_data(&b), bin->data(), bin->size());
			zmq_msg_send(&b, pub_, 0);
			zmq_msg_close(&b);
		}
	}

private:
	void * ctx_ {nullptr};
	void * pub_ {nullptr};
	CameraZmq * cam_ {nullptr};
	std::unordered_map<int, rtabmap::Transform> lastPoses_;
};

int main(int argc, char ** argv)
{
	(void)argc; (void)argv;
	const char * sub = std::getenv("SUB");
	const char * pub = std::getenv("PUB");
	std::string subEp = sub ? sub : "tcp://host.docker.internal:5555";
	std::string pubEp = pub ? pub : "tcp://*:6000";

	CameraZmq * cam = new CameraZmq(subEp);
	rtabmap::CameraThread cameraThread(cam);
	// Default RGBD odometry
	rtabmap::OdometryThread odomThread(new rtabmap::OdometryF2M);
	rtabmap::Rtabmap rtabmap;
	rtabmap::RtabmapThread rtabmapThread(&rtabmap);

	BridgePublisher publisher(pubEp, cam);
	UEventsManager::addHandler(&publisher);
	UEventsManager::addHandler(&odomThread);
	UEventsManager::addHandler(&rtabmapThread);

	cameraThread.start();
	odomThread.start();
	rtabmapThread.start();

	// Wait until threads finish (they won't unless interrupted)
	cameraThread.join(true);
	odomThread.join(true);
	rtabmapThread.join(true);

	return 0;
}


