#include "armor_detector/armors_shared.hpp"

#include <mutex>

namespace rm_auto_aim {

namespace {
std::mutex cb_mutex;
DetectionCallback g_cb;
}

void set_detection_callback(DetectionCallback cb) {
	std::lock_guard<std::mutex> lk(cb_mutex);
	g_cb = std::move(cb);
}

void emit_detections(DetectionBundle&& bundle) {
	DetectionCallback cb_copy;
	{
		std::lock_guard<std::mutex> lk(cb_mutex);
		cb_copy = g_cb;
	}
	if (cb_copy) {
		cb_copy(std::move(bundle));
	}
}

}  // namespace rm_auto_aim
