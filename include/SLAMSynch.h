#include <ros/ros.h>
#include "SLAM.h"
#include <sgtdv_msgs/ConeArr.h>

class SLAMSynch
{
public:
    SLAMSynch();
    ~SLAMSynch();

    void SetMapPublisher(ros::Publisher mapPublisher);
    void SetCarStatePublisher(ros::Publisher carStatePublisher);
    void DoMap(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
    void DoPose(); //TODO: Specify msg type and something else

private:
    SLAM m_slam;
    sgtdv_msgs::ConeArr m_cones;
    bool m_receivedMap;

};