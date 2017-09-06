/*
 * Copyright (c) 2017 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                  Dinesh Atchuthan on Aug 31 2017
 */

 // defining constants
#define NODE_NAME "optitrack_imuPose"
#define SUBSCRIBE_TOPIC_BASE "/optitrack/bodies"
#define PUBLISH_TOPIC_NAME "imu_pose"
#define OPTITRACK_FRAME "optitrack"
#define PUBLISH_RATE 200

#include <ros/ros.h>
#include <tf/tf.h>

#include <optitrack/or_pose_estimator_state.h>

class OptitrackIMU
{
    public:
    // constructor and destructor definitions
    OptitrackIMU(ros::NodeHandle& nh, std::string& topic_base,
                    std::string& publish_topic, std::string& optitrack_frame_id, int publish_rate);

    ~OptitrackIMU();

    private:
    ros::NodeHandle nh_;                        // private node handle
    std::string subscribe_topic_base_, publish_topic_;    // strings for topic configuration
    std::string optitrack_frame_id_;

    std::vector<ros::Subscriber> subs;          // imu subscribers for optitrack
    ros::Publisher pub;                         // publisher of built message

    ros::Timer publishTimer;                    // timer for periodic publishing of built message
    int publish_rate_;

    // helper variables
    ros::master::V_TopicInfo topics;

    typedef optitrack::or_pose_estimator_state::ConstPtr RawPose;

    std::map<int, RawPose> raw_messages;
    std::map<int, geometry_msgs::TransformStamped> lastStates;

    //helper functions
    void registerPose(geometry_msgs::TransformStamped &body,
                    optitrack::or_pose_estimator_state::ConstPtr msg);
    
    void publishIMUs(const ros::TimerEvent& event);
    
    // function definitions
    bool getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base);
    
    bool subscribeToTopics(std::string topic_base);

    void imu_callback(const optitrack::or_pose_estimator_state::ConstPtr& msg, const int id);
};

