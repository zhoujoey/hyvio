#ifdef ROS1
#include <ros1_nodelet.h>

namespace hyvio {
    
void Ros1Nodelet::onInit() {
    std::string config_file;
    getPrivateNodeHandle().getParam("config_file", config_file);
    if (config_file == "") {
        LOG(ERROR) << "can not load params, config_file is empty";
        return;
    } else {
        LOG(INFO) << "param path is : " << config_file;
    }
    system_ptr.reset(new System(getPrivateNodeHandle()));
    if (!system_ptr->initialize(config_file)) {
        LOG(ERROR) << "Cannot initialize System Manager...";
        return;
    }
    return;
}

PLUGINLIB_EXPORT_CLASS(hyvio::Ros1Nodelet, nodelet::Nodelet);

} // end namespace hyvio

#endif