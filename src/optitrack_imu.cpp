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
}