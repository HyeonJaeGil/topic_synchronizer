#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <mutex>


class ScanSynchronizer
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan1_sub_;
    ros::Subscriber scan2_sub_;
    ros::Publisher scan1_pub_;
    ros::Publisher scan2_pub_;
    ros::Timer timer;
    int frequency;
    std::mutex loop_mutex_;
    std::deque<sensor_msgs::LaserScan> scan1_deque_;
    std::deque<sensor_msgs::LaserScan> scan2_deque_;
    double scan1_del_t_;
    double scan2_del_t_;
public:
    ScanSynchronizer(ros::NodeHandle* nh);
    void timerCallback(const ros::TimerEvent&);
    void scan1Callback(const sensor_msgs::LaserScanConstPtr msg);
    void scan2Callback(const sensor_msgs::LaserScanConstPtr msg);
    void publishLoop(int frequency);
};

ScanSynchronizer::ScanSynchronizer(ros::NodeHandle* nh)
    :frequency(15)
{
    scan1_sub_ = nh_.subscribe("/scan", 100, 
                    &ScanSynchronizer::scan1Callback, this);
    scan2_sub_ = nh_.subscribe("/gl3/scan/filtered", 100, 
                    &ScanSynchronizer::scan2Callback, this);
    timer = nh_.createTimer(ros::Duration(1/frequency), 
       boost::bind(&ScanSynchronizer::timerCallback, this, _1));
    scan1_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan1_new", 100);
    scan2_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan2_new", 100);
    publishLoop(frequency);
}

void ScanSynchronizer::scan1Callback(const sensor_msgs::LaserScanConstPtr msg)
{
    if(!scan1_deque_.empty())
        scan1_del_t_ = abs(msg->header.stamp.toSec() 
                    - scan1_deque_.back().header.stamp.toSec());
    scan1_deque_.push_back(*msg);
}

void ScanSynchronizer::scan2Callback(const sensor_msgs::LaserScanConstPtr msg)
{
    if(!scan2_deque_.empty())
    scan2_del_t_ = abs(msg->header.stamp.toSec() 
                    - scan2_deque_.back().header.stamp.toSec());
    scan2_deque_.push_back(*msg);
}

void ScanSynchronizer::timerCallback(const ros::TimerEvent&){return;}


void ScanSynchronizer::publishLoop(int frequency)
{
    const std::lock_guard<std::mutex> lock(loop_mutex_);
    
    ros::Rate loop_rate(15);
    while(ros::ok())
    {
        double current_time = ros::Time::now().toSec();
        double threshold;
        if(!scan1_deque_.empty()){
            while(abs(scan1_deque_.front().header.stamp.toSec() 
                    - current_time) >  scan1_del_t_)
                scan1_deque_.pop_front();

            auto scan1_msg = scan1_deque_.front();
            scan1_deque_.pop_front();
            scan1_pub_.publish(scan1_msg);
        }
        if(!scan2_deque_.empty()){
            while(abs(scan2_deque_.front().header.stamp.toSec() 
                    - current_time) >  scan2_del_t_)
                scan2_deque_.pop_front();

            auto scan2_msg = scan2_deque_.front();
            scan2_deque_.pop_front();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_synchronizer");
    ros::NodeHandle nh("~");
    ScanSynchronizer scan_synchronizer(&nh);

    ROS_INFO("Starting scan_synchronizer node ...");
    ros::spin();
    return 0;
}