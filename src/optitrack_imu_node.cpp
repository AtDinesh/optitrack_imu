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

 #include <optitrack_imu/optitrack_imu.h>

 // instantiate local variables
 OptitrackIMU::OptitrackIMU(ros::NodeHandle& nh, std::string& subscribe_topic_base,std::string& publish_topic, std::string& optitrack_frame_id,
                            nt publish_rate, bool publish_markers) :
                            nh_(nh), subscribe_topic_base_(subscribe_topic_base), publish_topic_(publish_topic), optitrack_frame_id_(optitrack_frame_id),
                            publish_rate_(publish_rate), publish_markers_(publish_markers)
{
    // wait if optitrack is not up
    while(!subscribeToTopics(subscribe_topic_base_))
    {
        ROS_INFO_STREAM_NAMED(NODE_NAME, "waiting for optitrack data to be available");
        sleep(1);
    }
    ROS_INFO_STREAM_NAMED(NODE_NAME, "optitrack data: OK");

    // initialize the publisher
    std::string full_publish_topic = std::string(NODE_NAME) + "/" + publish_topic_;

    if (subs.size()  != 0)
    {
        // create publisher of type TransformStamped
        pub = nh_.advertise<geometry_msgs::TransformStamped>(full_publish_topic, 1);
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>(full_publish_topic + "_markers", 1);

        // create a publish timer
        if(publish_rate_ > 0.0)
        {
            publishTimer = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                           &OptitrackPerson::publishPersons, this);
            ROS_INFO_STREAM_NAMED(NODE_NAME, "publishing: " << full_publish_topic
                                 << " at " << publish_rate_ << " hz");
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "publish rate cannot be < 0, nothing will be published");
        }
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "no IMU found. \"" << full_publish_topic << "\" will not be published");
    }
}

OptitrackIMU::~OptitrackIMU()
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "destroying OptitrackIMU class");
}

bool OptitrackIMU::subscribeToTopics(std::string topic_base)
{
    bool result = false;

    // get all topics from master
    if (ros::master::getTopics(topics))
    {
        // create subscriber for each topic
        for (auto topic : topics)
        {   
            int id = -1;
            std::string topic_name = topic.name.substr(topic.name.find("IMU"),topic.name.length());
            std::cout << topic_name << std::endl;
            sscanf(topic_name.c_str(),"imu_%d",&id);
            if ((id != -1) && !topic_name.empty())
            {
                subs.push_back(nh_.subscribe<optitrack_imu::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackIMU::imu_callback, this, _1, id)));
                ROS_DEBUG_STREAM_NAMED(NODE_NAME, "created subscriber for topic: " << topic.name);
                result = true;
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "cannot get topic list from the master");
    }

    return result;
}

// callback for the optitrack imu
void OptitrackIMU::imu_callback(const optitrack_imu::or_pose_estimator_state::ConstPtr& id, )
{
    if (msg->pos.size() != 0)
    {
        // update the map of pointers with latest pointer
        raw_messages[id] = msg;
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found IMU " << id);
    }
}

void OptitrackPerson::registerPose(geometry_msg::TransformStamped &trackedIMU, optitrack_person::or_pose_estimator_state::ConstPtr msg)
{
    trackedIMU.header.stamp.sec = msg->ts.sec;
    trackedIMU.header.stamp.nsec = msg->ts.nsec;
    
    trackedIMU.translation.x = msg->pos[0].x;
    trackedIMU.translation.y = msg->pos[0].y;
    trackedIMU.translation.z = msg->pos[0].z;
    trackedIMU.rotation.x = msg->pos[0].qx;
    trackedIMU.rotation.y = msg->pos[0].qy;
    trackedIMU.rotation.z = msg->pos[0].qz;
    trackedIMU.rotation.w = msg->pos[0].qw;
}

void OptitrackPerson::publishIMUs(const ros::TimerEvent& event)
{
    // create TransformSamped message
    geometry_msg::TransformStamped trackedIMU;

    // loop through all messages in raw_messages map
    for (auto imu : raw_messages)
    {
        geometry_msg::TransformStamped poseIMU;

            // check if the pointer is not null
            if (imu)
            {
                // TODO : implement here a low pass filter

                // put optitrack data in TransformStamped
                registerPose(trackedIMU,imu);
                lastStates[id] = trackedIMU;
            }
    }
    // add the header
    trackedIMU.header.frame_id = optitrack_frame_id_;

    // publish the trackedIMU message
    pub.publish(trackedIMU);

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "published imu");
}

// handler for something to do before killing the node
void sigintHandler(int sig){
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");
    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
    exit(sig); // necessary to interrupt during optitrackPerson initialisation
}

}