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
 #include <signal.h>
 #include <optitrack_imu/optitrack_imu.h>

 // instantiate local variables
 OptitrackIMU::OptitrackIMU(ros::NodeHandle& nh, std::string& subscribe_topic_base,std::string& publish_topic, std::string& optitrack_frame_id,
                            int publish_rate) :
                            nh_(nh), subscribe_topic_base_(subscribe_topic_base), publish_topic_(publish_topic), optitrack_frame_id_(optitrack_frame_id),
                            publish_rate_(publish_rate)
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

        // create a publish timer
        if(publish_rate_ > 0.0)
        {
            publishTimer = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                           &OptitrackIMU::publishIMUs, this);
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
        // filter topic list for person only topics
        if (getSubTopics(topics, topic_base))
        {
            for (auto topic : topics)
            {   
                int id = -1;
                try{
                    std::string topic_name = topic.name.substr(topic.name.find("imu"),topic.name.length());
                    std::cout << topic_name << std::endl;
                    sscanf(topic_name.c_str(),"imu_%d",&id);

                    if ((id != -1) && (!topic_name.empty()))
                    {
                        subs.push_back(nh_.subscribe<optitrack::or_pose_estimator_state>(topic.name, 1, boost::bind(&OptitrackIMU::imu_callback, this, _1, id)));
                        ROS_INFO_STREAM_NAMED(NODE_NAME, "created subscriber for topic: " << topic.name);
                        result = true;
                    }
                }catch (...){}
            }
        }
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "cannot get topic list from the master");
    }

    return result;
}

bool OptitrackIMU::getSubTopics(ros::master::V_TopicInfo& topics, const std::string& topic_base)
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "looking for \"" << topic_base << "\" string in all (" << topics.size() << ") topics");

    topics.erase(
        std::remove_if(
        std::begin(topics),
        std::end(topics),
        [topic_base](ros::master::TopicInfo topic) -> bool
            {
              return (topic.name.find(topic_base) == std::string::npos);
            }
        ),
        std::end(topics));

    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found " << topics.size() << " topics with \"" << topic_base << "\"");

    return true;
}

// callback for the optitrack imu
void OptitrackIMU::imu_callback(const optitrack::or_pose_estimator_state::ConstPtr& msg, const int id)
{
    if (msg->pos.size() != 0)
    {
        // update the map of pointers with latest pointer
        raw_messages[id] = msg;
        ROS_DEBUG_STREAM_NAMED(NODE_NAME, "found IMU " << id);
    }
}

void OptitrackIMU::registerPose(geometry_msgs::TransformStamped &trackedIMU, optitrack::or_pose_estimator_state::ConstPtr msg)
{
    trackedIMU.header.stamp.sec = msg->ts.sec;
    trackedIMU.header.stamp.nsec = msg->ts.nsec;
    
    trackedIMU.transform.translation.x = msg->pos[0].x;
    trackedIMU.transform.translation.y = msg->pos[0].y;
    trackedIMU.transform.translation.z = msg->pos[0].z;
    trackedIMU.transform.rotation.x = msg->pos[0].qx;
    trackedIMU.transform.rotation.y = msg->pos[0].qy;
    trackedIMU.transform.rotation.z = msg->pos[0].qz;
    trackedIMU.transform.rotation.w = msg->pos[0].qw;
}

void OptitrackIMU::publishIMUs(const ros::TimerEvent& event)
{
    // create TransformSamped message
    geometry_msgs::TransformStamped trackedIMU;

    // loop through all messages in raw_messages map
    if(!raw_messages.empty())
    {
        for (auto imu : raw_messages)
        {
            geometry_msgs::TransformStamped poseIMU;
            // TODO : implement here a low pass filter

            // put optitrack data in TransformStamped
            registerPose(trackedIMU,imu.second);
            lastStates[imu.first] = trackedIMU;
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

// the main method starts a rosnode and initializes the optitrack_imu class
int main(int argc, char **argv)
{
    // starting the optitrack_person node
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "started " << NODE_NAME << " node");

    // getting topic parameters
    std::string subscribe_topic_base, publish_topic_name, optitrack_frame_id;
    int publish_rate;
    nh.param<std::string>("topic_base", subscribe_topic_base, SUBSCRIBE_TOPIC_BASE);
    nh.param<std::string>("published_topic", publish_topic_name, PUBLISH_TOPIC_NAME);
    nh.param<std::string>("optitrack_frame_id", optitrack_frame_id, OPTITRACK_FRAME);
    nh.param<int>("publish_rate", publish_rate, PUBLISH_RATE);

    // look for sigint
    signal(SIGINT, sigintHandler);
    
    // initializing OptitrackPerson class and passing the node handle to it
    OptitrackIMU optitrackIMU(nh, subscribe_topic_base, publish_topic_name, optitrack_frame_id, publish_rate);
    
    //start spinning the node
    ros::spin();

    return 0;
}