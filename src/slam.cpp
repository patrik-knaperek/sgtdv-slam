/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include "../include/SLAM.h"


SLAM::SLAM()
{
    // m_muUpdate = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
    // m_lastPose.theta = 0.f;
    // m_lastPose.x = 0.f;
    // m_lastPose.y = 0.f;
    // m_posePrevious << 0.0, 0.0, 0.0;
    // Pose m_posePrevious << 0.0, 0.0, 0.0;

    SetupMatrices();

}

SLAM::~SLAM()
{

}

void SLAM::SetMapPublisher(ros::Publisher mapPublisher)
{
    m_mapPublisher = mapPublisher;
}

void SLAM::SetCarPosePublisher(ros::Publisher carPosePublisher)
{
    m_carPosePublisher = carPosePublisher;
}

void SLAM::CarStateCallbackSim(const fsd_common_msgs::CarState::ConstPtr& msg)
{  
    m_poseActual.x = msg->car_state.x;
    m_poseActual.y = msg->car_state.y;
    m_poseActual.yaw = msg->car_state.theta;
    if(m_posePrevious.x == 0.0){
        m_posePrevious = m_poseActual;
    }

//   pubCarState();
    PoseDiff(m_poseActual, m_posePrevious);
    m_posePrevious = m_poseActual;
    EkfPredict();
}

void SLAM::CarStateCallbackReal(const sgtdv_msgs::CarPose::ConstPtr& msg)
{  
  m_poseActual.x = msg->position.x;
  m_poseActual.y = msg->position.y;
  m_poseActual.yaw = msg->yaw;

  // pubCarState();
}

void SLAM::ConesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  m_coneAbsVect.clear();
  float const *temp;

  for (int i = 0; i < msg->width; i++)
    {

      temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
      geometry_msgs::Point32 point;    

      point.x = *temp;
      point.y = *(temp + 1);

      if(*(temp + 9) > 0.85){m_coneColor = 1;}   // 1 = blue
      if(*(temp + 10) > 0.85){m_coneColor = 2;}   // 2 = yellow
      if(*(temp + 11) > 0.85){m_coneColor = 3;}   // 3 = orange

      m_coneRange = sqrt(pow(point.x,2) + pow(point.y,2));
      m_coneBearing = atan2(point.y, point.x);

      m_coneAbsX = m_odomX + m_coneRange * cos(m_coneBearing + m_odomYaw);
      m_coneAbsY = m_odomY + m_coneRange * sin(m_coneBearing + m_odomYaw);

      DataAssEuclid();

      vector<double> newRow;
      newRow = {m_coneAbsX, m_coneAbsY, m_coneColor};

      m_coneAbsVect.push_back(newRow);
    }
  
  // pubCones();
  EkfUpdate();

} 

void SLAM::ConesCallbackReal(const sgtdv_msgs::ConeArr::ConstPtr& msg)
{
  m_coneAbsVect.clear();
  float const *temp;
  geometry_msgs::Point32 point;

  for (int i = 0; i < msg->cones.size(); i++)
    {
      if(*(temp + 9) > 0.85){m_coneColor = 1;}   // 1 = blue
      if(*(temp + 10) > 0.85){m_coneColor = 2;}   // 2 = yellow
      if(*(temp + 11) > 0.85){m_coneColor = 3;}   // 3 = orange

      m_coneRange = sqrt(pow(point.x,2) + pow(point.y,2));
      m_coneBearing = atan2(point.y, point.x);

      m_coneAbsX = m_odomX + m_coneRange * cos(m_coneBearing + m_odomYaw);
      m_coneAbsY = m_odomY + m_coneRange * sin(m_coneBearing + m_odomYaw);

      DataAssEuclid();

      vector<double> newRow;
      newRow = {m_coneAbsX, m_coneAbsY, cone.color};

      m_coneAbsVect.push_back(newRow);
    }
  
//   pubCones();
}

void SLAM::PoseDiff(Pose &poseActual, Pose &posePrevious)
{
  // m_posePrevious<< m_pose.x, m_pose.y, m_pose.yaw;
  
    float xDiff = poseActual.x - posePrevious.x;
    float yDiff = poseActual.y - posePrevious.y;

    m_positionDiff = sqrt(xDiff * xDiff + yDiff * yDiff);
    m_rotationDiff = poseActual.yaw - posePrevious.yaw;
    m_positionBearing = atan2(yDiff, xDiff);

    // cout << m_poseDiff << "-----"<< endl;

}

void SLAM::EkfPredict(){

    m_N = m_stateVector.rows();

    Eigen::MatrixXf F(3,m_N);
    Eigen::MatrixXf G;
    Eigen::Matrix3f JacobianMotion = Eigen::Matrix3f::Zero();

   
    // std::cout << "m_N" << m_N << std::endl;
    F << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,m_N-3);
   

  
    Eigen::Vector3f motionModel = MotionModelDiff();
    JacobianMotion(0,2) = -m_positionDiff * sin(m_positionBearing);
    JacobianMotion(1,2) = m_positionDiff * cos(m_positionBearing);

    // std::cout << "F" << F << std::endl;
    // int a = m_N/2-3;
    m_stateVector += F.transpose() * m_motion;
    G = Eigen::MatrixXf::Identity(m_N, m_N)+ F.transpose() * JacobianMotion * F;
    
    
    m_covMatrix = G * m_covMatrix * G.transpose() + F.transpose() * m_RT * F; 

    
}

void SLAM::EkfUpdate(){

    Eigen::Vector2f coordsDelta;
    Eigen::Vector2f zHat;
    Eigen::MatrixXf F(5, m_stateVector.rows());
    float q, sq;

    std::cout << "cone vector size" << m_coneAbsVect.size() << std::endl; 

    

    for (int i = 0; i < m_coneAbsVect.size(); i++){

        m_N = MapConesCounter(m_stateVector);

        if(m_N == 0){
            continue;
        } 
        coordsDelta << m_stateVector(i,0)- m_stateVector(0,0), m_stateVector(i+1,0)- m_stateVector(1,0);
    //     q = coordsDelta.transpose() * coordsDelta;
    //     sq = sqrt(q);
    //     zHat << sq, atan2(coordsDelta(1), coordsDelta(0)-m_stateVector(2,0));
    //     F = Eigen::MatrixXf::Zero(5, m_stateVector.rows());
    //     F.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
    //     F(3,i) = 1;
    //     F(4,i+1) = 1;

    }



}


// void SLAM::EkfPredict(const Pose &pose)
// {
//     int n = m_muUpdate.size().height;
//     float *muUpdateRow2 = m_muUpdate.ptr<float>(2);

//     float cosRes = cos(*(muUpdateRow2));
//     float sinRes = sin(*(muUpdateRow2));

//     *(m_motion.ptr<float>(0)) = m_traveledDistance * cosRes;
//     *(m_motion.ptr<float>(1)) = m_traveledDistance * sinRes;
//     *(m_motion.ptr<float>(2)) = m_rotationDiff;

//     *(m_Jakobian.ptr<float>(0) + 2) = -m_traveledDistance * sinRes;
//     *(m_Jakobian.ptr<float>(1) + 2) = m_traveledDistance * cosRes;

//     cv::Mat1f F = cv::Mat::zeros(3, n - 3, CV_32FC1);
//     F.at<float>(0, 0) = 1.f;
//     F.at<float>(1, 1) = 1.f;
//     F.at<float>(2, 2) = 1.f;

//     cv::Mat1f transpF = F.t();
//     m_muPredict = m_muUpdate + (transpF * m_motion);
    
//     cv::Mat1f G = cv::Mat::eye(n, n, CV_32FC1);
//     G += transpF * m_Jakobian * F;
//     cv::Mat1f noise = transpF * m_RT * F;

//     m_covPredict = G * m_covUpdate * G.t() + noise;
// }

// void SLAM::EkfUpdate()
// {
//     for(size_t i = 0; i < m_observations.size(); i++)
//     {
//         Observation obs = m_observations[i];
//         size_t recordsCount = m_muPredict.size().height;
//         size_t currentKnownCones = (recordsCount - 3) / 2;
//         std::vector<float> euclideanConeDist;
//         euclideanConeDist.reserve(currentKnownCones);   //needs one less but 0 - 1 as size_t would give max number of 64bit uint

//         cv::Point2f globalConePos(
//             m_muPredict.at<float>(0, 0) + obs.distance * cos(obs.alpha + m_muPredict.at<float>(2, 0)),
//             m_muPredict.at<float>(1, 0) + obs.distance * sin(obs.alpha + m_muPredict.at<float>(2, 0))
//         );

//         float min = std::numeric_limits<float>::max();
//         size_t minIdx = 0;

//         for (size_t j = 0; j < currentKnownCones; j++)
//         {            
//             float tempX = globalConePos.x - m_muUpdate.at<float>(i * 2 + 3);
//             float tempY = globalConePos.y - m_muUpdate.at<float>(i * 2 + 4);
//             float tempDist(sqrt(tempX * tempX + tempY * tempY));

//                 minIdx = j;
//                 min = tempDist;
//             }

//             euclideanConeDist.push_back(tempDist);
//         }

//         size_t predictRowIndex = 2 * minIdx + 3;

//         if (currentKnownCones == 0 || euclideanConeDist[minIdx] >= THRESHOLD_DISTANCE)
//         {
//             m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.x)));
//             m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.y)));

//             m_covPredict.push_back(cv::Mat::zeros(2, m_covPredict.size().width, CV_32FC1));
//             cv::hconcat(m_covPredict, cv::Mat::zeros(m_covPredict.size().height, 2, CV_32FC1), m_covPredict);

//             m_covPredict.at<float>(recordsCount, recordsCount) = INF;
//             m_covPredict.at<float>(recordsCount + 1, recordsCount + 1) = INF;

//             predictRowIndex = m_muPredict.size().height - 2;
//         }

//         size_t N = m_muPredict.size().height;
        
//         m_delta << m_muPredict.at<float>(predictRowIndex, 0) - m_muPredict.at<float>(0, 0), m_muPredict.at<float>(predictRowIndex, 1) - m_muPredict.at<float>(1, 0);

//         float q = m_delta.t().dot(m_delta);
//         float sq = sqrt(q);

//         float z_theta = std::atan2(m_delta.at<float>(0, 0), m_delta.at<float>(1, 0));
//         m_Zhat << sq, z_theta - m_muPredict.at<float>(2, 0);

//         cv::Mat1f F = cv::Mat::zeros(5, N, CV_32FC1);
//         F.at<float>(0, 0) = 1.f;
//         F.at<float>(1, 1) = 1.f;
//         F.at<float>(2, 2) = 1.f;
//         F.at<float>(3, predictRowIndex) = 1;
//         F.at<float>(4, predictRowIndex + 1) = 1;

//         m_Hz << -sq * m_delta.at<float>(1), 0.f, sq * m_delta.at<float>(0), sq * m_delta.at<float>(1),
//             m_delta.at<float>(1), -m_delta.at<float>(0), -q, -m_delta.at<float>(1), m_delta.at<float>(0);
        
//         cv::Mat1f H = 1.f / q * (m_Hz * F);
//         cv::Mat1f Htransp = H.t();

//         cv::Mat1f K = (m_covPredict * Htransp) * ( (H * m_covPredict) * Htransp + m_QT).inv();

//         m_Zdiff << obs.distance, obs.alpha;
//         m_Zdiff += -m_Zhat + M_PI;

//         ModuloMatMembers(m_Zdiff, 2 * M_PI);
//         m_Zdiff -= M_PI;        

//         m_muUpdate = m_muPredict + K * m_Zdiff;
//         m_covUpdate = cv::Mat::eye(N, N, CV_32FC1) - (K * H) * m_covPredict;
//     }
// }

Eigen::Vector3f SLAM::MotionModelDiff(){

    m_motion(0) = m_positionDiff * cos(m_positionBearing);
    m_motion(1) = m_positionDiff * sin(m_positionBearing);
    m_motion(2) = m_rotationDiff;

    return m_motion;
}

void SLAM::DataAssEuclid(){
  
  vector<double> newRow1, newRow2;
  vector<double> euclidVect;  
  euclidVect.clear();
 
  if(m_coneMap.empty() == true){

    newRow1 = {m_coneAbsX, m_coneAbsY, m_coneColor};
    m_coneMap.push_back(newRow1);   

  } 
  else{
    vector<vector<double>>::iterator iter;        

    for(int i=0; i < m_coneMap.size(); i++)
    {
      // cout << i << "/"  <<  m_coneAbsX << ";" << m_coneMap[i][0] << ";"<< m_coneAbsY << ";" << m_coneMap[i][1] << endl;
      m_euclidDist = sqrt(pow(m_coneAbsX - m_coneMap[i][0], 2) + pow(m_coneAbsY - m_coneMap[i][1], 2));
      euclidVect.push_back(m_euclidDist);
    } 
    int minElementIndex = std::min_element(euclidVect.begin(),euclidVect.end()) - euclidVect.begin();

    // //print map
    //   for (int i = 0; i < m_coneMap.size(); i++)
    // {
    //     for (int j = 0; j < m_coneMap[i].size(); j++)
    //     {
    //         cout << m_coneMap[i][j] << "   "; 
    //     }        
    // }
    // cout << endl;    

    if(euclidVect[minElementIndex] < m_euclidThresh){
      m_coneMap[minElementIndex][0] = m_coneAbsX;
      m_coneMap[minElementIndex][1] = m_coneAbsY;
    }
    else{
      newRow2 = {m_coneAbsX, m_coneAbsY, m_coneColor};
      m_coneMap.push_back(newRow2); 
    }
  }

}

void SLAM::SetupMatrices()
{ 
    m_RT = Eigen::Matrix3f::Zero();
    m_QT = Eigen::Matrix2f::Zero();

    m_stateVector = Eigen::VectorXf::Zero(3); 
    m_covMatrix = Eigen::MatrixXf::Zero(3,3);
    
     
    // std::cout << "RT" << m_RT << std::endl;
    //  std::cout << "F" << m_F << std::endl;
    // std::cout << "QT" << m_QT << std::endl;
    // std::cout << "state vector" << m_stateVector << std::endl;
    // std::cout << "cov matrix" << m_covMatrix << std::endl;
    // std::cout << "jacobian motion" << m_JacobianMotion << std::endl;
    // m_F << Eigen::MatrixXf::Identity(3,3), Eigen::MatrixXf::Zero(3,3);
    // m_F.block(0,0,3,3) = Eigen::MatrixXf::Identity(3,3);

}

int SLAM::MapConesCounter(const Eigen::VectorXf stateVector){
    return (stateVector.rows()-3)/2;
}



// double SLAM::GetDistance(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const
// {
//     double relativeX = p2.x - p1.x;
//     double relativeY = p2.y - p1.y;
//     double xBuff = p1.x * p2.x;
//     double yBuff = p1.y * p2.y;
//     return std::sqrt(xBuff * xBuff + yBuff * yBuff);
// }

int main(int argc, char **argv)
{
  SLAM slam; 

  ros::init(argc, argv, "SLAM");
  ros::NodeHandle nh;
  
  ros::Subscriber CarStateSubSim = nh.subscribe("estimation/slam/state_throttle", 1, &SLAM::CarStateCallbackSim, &slam);
  ros::Subscriber ConesSubSim = nh.subscribe("fssim/camera/cones", 1, &SLAM::ConesCallbackSim, &slam);
  ros::Subscriber CarStateSubReal = nh.subscribe("car_state", 1, &SLAM::CarStateCallbackReal, &slam);
  ros::Subscriber ConesSubReal = nh.subscribe("fusion_cones", 1, &SLAM::ConesCallbackReal, &slam);
//   slam.pubCarPose = nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
//   slam.pubMap = nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1);
//   slam.pubCarPoseMarker = nh.advertise<visualization_msgs::Marker>("slam/pose/marker", 1);
//   slam.pubMapMarker = nh.advertise<visualization_msgs::Marker>("slam/map/marker", 1);

  ros::spin();

  return 0;
}