#ifndef COMMON_PUB_H_
#define COMMON_PUB_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace ego_planner {
    extern ros::Publisher g_finish_pub;   // 声明
    inline void publishFinish() {         // 内联接口
        if (g_finish_pub) {
            std_msgs::Empty e;
            g_finish_pub.publish(e);
            ROS_INFO("[PLAN] finish_event published");
        }
    }
} // namespace ego_planner
#endif