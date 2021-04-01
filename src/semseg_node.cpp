#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <perception_pipeline/PredictLabels.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <string>
#include <vector>

ros::Publisher semseg_pub;
int threads = 1;
std::vector<ros::ServiceClient> clientList;

boost::mutex mio; //shared by subscriber callbacks
int service_counter = 1;

int findMyServiceID(int threads)
{
    boost::lock_guard<boost::mutex> lock(mio);
    int myID = service_counter;
    if (service_counter < threads)
    {
        service_counter++;
    }
    else
    {
        service_counter = 1;
    }
    return myID;
}

void predicationCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    //Create the request and response objects.
    perception_pipeline::PredictLabels::Request req;
    perception_pipeline::PredictLabels::Response resp;

    sensor_msgs::PointCloud2 inputPointCloud = *cloud;

    req.input = inputPointCloud;
    int serviceId = findMyServiceID(threads);

    std::string serviceName = "prediction_service_" + std::to_string(serviceId);
    ROS_INFO("\nmy service name: %s", serviceName.c_str());

    ROS_INFO("Calling the predictor service: %s", serviceName.c_str());

    ros::service::waitForService(serviceName, ros::Duration(0));
    ros::ServiceClient client = clientList[serviceId - 1];
    bool success = client.call(req, resp);

    if (!success)
    {
        ROS_ERROR("Calling Predictor Service Failed!");
    }
    else
    {
        sensor_msgs::PointCloud2 outputPointCloud = resp.output;
        semseg_pub.publish(outputPointCloud);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "semseg_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");

    threads = priv_nh_.param<int>("num_threads", 1);
    ROS_INFO("Configured for %s threads", std::to_string(threads).c_str());

    semseg_pub = nh.advertise<sensor_msgs::PointCloud2>("env/semseg_labeled", 1);

    ros::SubscribeOptions ops;
    ops.template init<sensor_msgs::PointCloud2>("env/semseg_input", 1, predicationCallback);
    ops.allow_concurrent_callbacks = true;
    ros::Subscriber sub = nh.subscribe(ops);

    for (int i = 1; i <= threads; i++)
    {
        auto serviceName = "prediction_service_" + std::to_string(i);
        ROS_INFO("Adding service with name: %s", serviceName.c_str());
        ros::ServiceClient predictorClient = nh.serviceClient<perception_pipeline::PredictLabels>(serviceName, true);
        clientList.push_back(predictorClient);
    }
    auto size = clientList.size();
    ROS_INFO("%s services registred.", std::to_string(size).c_str());

    // ros::spin();
    ros::MultiThreadedSpinner spinner(threads); // 2 threads
    spinner.spin();

    return 0;
}