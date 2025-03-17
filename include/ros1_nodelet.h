//
// Created by xiaochen at 19-8-21.
// Nodelet for system manager.
//

#ifndef ROS1_NODELET_H
#define ROS1_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <System.h>

namespace hyvio {
    class Ros1Nodelet : public nodelet::Nodelet {
    public:
        Ros1Nodelet() { return; }
        ~Ros1Nodelet() {
            // debug log
            return; }

    private:
        virtual void onInit();
        SystemPtr system_ptr;
    };
} // end namespace hyvio

#endif  //SYSTEM_NODELET_H
