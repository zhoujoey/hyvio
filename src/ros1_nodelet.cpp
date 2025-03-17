#ifdef ROS1
#include <ros1_nodelet.h>

namespace hyvio {
    
void Ros1Nodelet::onInit() {
    system_ptr.reset(new System(getPrivateNodeHandle()));
    if (!system_ptr->initialize()) {
        LOG(ERROR) << "Cannot initialize System Manager...";
        return;
    }
    return;
}

PLUGINLIB_EXPORT_CLASS(hyvio::Ros1Nodelet, nodelet::Nodelet);

} // end namespace hyvio

#endif