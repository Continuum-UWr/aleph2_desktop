#include "rostalker.h"

#include <input_msgs/InputMessage.h>
#include <input_msgs/DevicesListMessage.h>
#include <sstream>
#include <list>

struct RosTalkerInternals
{
};

//bool my_kurwa_send = false;

void RosTalker::Init(int argc, char **argv)
{
    ros::init(argc, argv, "input_manager", ros::init_options::AnonymousName);

    n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    pn = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));
    pub = n->advertise<input_msgs::DevicesListMessage>("input/devices_list", 1, true);
    n_name = ros::this_node::getName();
    n_name = n_name.substr(1);
}

void RosTalker::RegisterDevice(std::weak_ptr<DeviceState> dev)
{
    auto device_locked = dev.lock();

    ROS_INFO("Registered device: %s", device_locked->name.c_str());
    topics.push_back(std::make_tuple(
        pn->advertise<input_msgs::InputMessage>(device_locked->name, 10),
        dev));
    PublishDevices();
}

void RosTalker::Update()
{
	ros::Rate loop_rate(30);
    bool send = false;
    topics.erase(std::remove_if(topics.begin(), topics.end(),
                                [&send](std::tuple<ros::Publisher, std::weak_ptr<DeviceState>> entry) -> bool {
                                    if (std::get<1>(entry).expired())
                                    {
										ROS_INFO("Device lost!");
                                        send = true;
                                        return true;
                                    }

                                    auto device = std::get<1>(entry).lock();
                                    input_msgs::InputMessage msg;

                                    msg.axes = device->axes;
                                    msg.buttons = device->buttons;
                                    msg.buttons_pressed = std::move(device->presses);
                                    msg.buttons_released = std::move(device->releases);
                                    std::get<0>(entry).publish(msg);

                                    return false;
                                }),
                 topics.end());

    ros::spinOnce();
    if(send) {
        PublishDevices();
    }
    loop_rate.sleep();
}


void RosTalker::PublishDevices() {
    input_msgs::DevicesListMessage msg;
    
	msg.node_name = n_name;
    for (auto dev : topics) {
        msg.devices_list.push_back(std::get<1>(dev).lock()->name);
    }

	//msg.devices_list = devs;
    pub.publish(msg);
}

