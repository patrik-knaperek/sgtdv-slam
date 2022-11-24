/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include "../include/SLAM.h"


SLAM::SLAM()
{
    m_muUpdate = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
    m_lastPose.theta = 0.f;
    m_lastPose.x = 0.f;
    m_lastPose.y = 0.f;

    SetupNoiseMatrices();

    m_motion = cv::Mat1f(3, 1);
    m_Jakobian = cv::Mat1f::zeros(3, 3);
    m_Hz = cv::Mat1f(2, 5);
    m_Zhat = cv::Mat1f(2, 1);
    m_delta = cv::Mat1f(2, 1);
    m_Zdiff = cv::Mat1f(2, 1);
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

void SLAM::Do(const SLAMMsg &msg)
{
    Pose pose;
    sgtdv_msgs::ConeArr cones;
    sgtdv_msgs::CarPose carPose;

    m_observations.clear();

    SetupMatrices(msg.cones->cones.size() * 2 + 3);
    InitPose(pose, msg.pwc->pose);
    InitObservations(msg);

    DataAssociation(msg.cones, msg.pwc->pose.position);
    EkfPredict(pose);
    EkfUpdate();

    m_carPosePublisher.publish(carPose);
    m_mapPublisher.publish(cones);

    m_lastPose = pose;
}

void SLAM::SetupMatrices(size_t size)
{
    m_muUpdate.resize(size);
    m_covUpdate = cv::Mat1f(size, size);
    cv::setIdentity(m_covUpdate, std::numeric_limits<float>::min());
    ZeroDiagonal(m_covUpdate, 3);
}

void SLAM::ZeroDiagonal(cv::Mat1f &mat, size_t rowCount) const
{
    for(size_t i = 0; i < rowCount; i++)
    {
        *(mat.ptr<float>(i) + i) = 0.f;
    }
}

void SLAM::InitPose(Pose &pose, const geometry_msgs::Pose &msg)
{
    pose.x = msg.position.x;
    pose.y = msg.position.y;
    //pose.theta = msg.orientation;  //TODO: kvaterniony do stupnov

    float xDiff = pose.x - m_lastPose.x;
    float yDiff = pose.y - m_lastPose.y;
    m_traveledDistance = sqrt(xDiff * xDiff + yDiff * yDiff);
    m_rotationDiff = pose.theta - m_lastPose.theta;
}

void SLAM::InitObservations(const SLAMMsg &msg)
{
    m_observations.reserve(msg.cones->cones.size());

    for (size_t i = 0; i < msg.cones->cones.size(); i++)
    {
        Observation temp;

        temp.distance = cv::norm(cv::Vec2f( msg.cones->cones[i].coords.x,  msg.cones->cones[i].coords.y)
         - cv::Vec2f(msg.pwc->pose.position.x, msg.pwc->pose.position.y));

        temp.alpha = std::atan2(msg.cones->cones[i].coords.y, msg.cones->cones[i].coords.x);

        m_observations.push_back(temp);
    }
}

void SLAM::EkfPredict(const Pose &pose)
{
    int n = m_muUpdate.size().height;
    float *muUpdateRow2 = m_muUpdate.ptr<float>(2);

    float cosRes = cos(*(muUpdateRow2));
    float sinRes = sin(*(muUpdateRow2));

    *(m_motion.ptr<float>(0)) = m_traveledDistance * cosRes;
    *(m_motion.ptr<float>(1)) = m_traveledDistance * sinRes;
    *(m_motion.ptr<float>(2)) = m_rotationDiff;

    *(m_Jakobian.ptr<float>(0) + 2) = -m_traveledDistance * sinRes;
    *(m_Jakobian.ptr<float>(1) + 2) = m_traveledDistance * cosRes;

    cv::Mat1f F = cv::Mat::zeros(3, n - 3, CV_32FC1);
    F.at<float>(0, 0) = 1.f;
    F.at<float>(1, 1) = 1.f;
    F.at<float>(2, 2) = 1.f;

    cv::Mat1f transpF = F.t();
    m_muPredict = m_muUpdate + (transpF * m_motion);
    
    cv::Mat1f G = cv::Mat::eye(n, n, CV_32FC1);
    G += transpF * m_Jakobian * F;
    cv::Mat1f noise = transpF * m_RT * F;

    m_covPredict = G * m_covUpdate * G.t() + noise;
}

void SLAM::EkfUpdate()
{
    for(size_t i = 0; i < m_observations.size(); i++)
    {
        Observation obs = m_observations[i];
        size_t recordsCount = m_muPredict.size().height;
        size_t currentKnownCones = (recordsCount - 3) / 2;
        std::vector<float> euclideanConeDist;
        euclideanConeDist.reserve(currentKnownCones);   //needs one less but 0 - 1 as size_t would give max number of 64bit uint

        cv::Point2f globalConePos(
            m_muPredict.at<float>(0, 0) + obs.distance * cos(obs.alpha + m_muPredict.at<float>(2, 0)),
            m_muPredict.at<float>(1, 0) + obs.distance * sin(obs.alpha + m_muPredict.at<float>(2, 0))
        );

        float min = std::numeric_limits<float>::max();
        size_t minIdx = 0;

        for (size_t j = 0; j < currentKnownCones; j++)
        {            
            float tempX = globalConePos.x - m_muUpdate.at<float>(i * 2 + 3);
            float tempY = globalConePos.y - m_muUpdate.at<float>(i * 2 + 4);
            float tempDist(sqrt(tempX * tempX + tempY * tempY));

            if (tempDist < min)
            {
                minIdx = j;
                min = tempDist;
            }

            euclideanConeDist.push_back(tempDist);
        }

        size_t predictRowIndex = 2 * minIdx + 3;

        if (currentKnownCones == 0 || euclideanConeDist[minIdx] >= THRESHOLD_DISTANCE)
        {
            m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.x)));
            m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.y)));

            m_covPredict.push_back(cv::Mat::zeros(2, m_covPredict.size().width, CV_32FC1));
            cv::hconcat(m_covPredict, cv::Mat::zeros(m_covPredict.size().height, 2, CV_32FC1), m_covPredict);

            m_covPredict.at<float>(recordsCount, recordsCount) = INF;
            m_covPredict.at<float>(recordsCount + 1, recordsCount + 1) = INF;

            predictRowIndex = m_muPredict.size().height - 2;
        }

        size_t N = m_muPredict.size().height;
        
        m_delta << m_muPredict.at<float>(predictRowIndex, 0) - m_muPredict.at<float>(0, 0), m_muPredict.at<float>(predictRowIndex, 1) - m_muPredict.at<float>(1, 0);

        float q = m_delta.t().dot(m_delta);
        float sq = sqrt(q);

        float z_theta = std::atan2(m_delta.at<float>(0, 0), m_delta.at<float>(1, 0));
        m_Zhat << sq, z_theta - m_muPredict.at<float>(2, 0);

        cv::Mat1f F = cv::Mat::zeros(5, N, CV_32FC1);
        F.at<float>(0, 0) = 1.f;
        F.at<float>(1, 1) = 1.f;
        F.at<float>(2, 2) = 1.f;
        F.at<float>(3, predictRowIndex) = 1;
        F.at<float>(4, predictRowIndex + 1) = 1;

        m_Hz << -sq * m_delta.at<float>(1), 0.f, sq * m_delta.at<float>(0), sq * m_delta.at<float>(1),
            m_delta.at<float>(1), -m_delta.at<float>(0), -q, -m_delta.at<float>(1), m_delta.at<float>(0);
        
        cv::Mat1f H = 1.f / q * (m_Hz * F);
        cv::Mat1f Htransp = H.t();

        cv::Mat1f K = (m_covPredict * Htransp) * ( (H * m_covPredict) * Htransp + m_QT).inv();

        m_Zdiff << obs.distance, obs.alpha;
        m_Zdiff += -m_Zhat + M_PI;

        ModuloMatMembers(m_Zdiff, 2 * M_PI);
        m_Zdiff -= M_PI;        

        m_muUpdate = m_muPredict + K * m_Zdiff;
        m_covUpdate = cv::Mat::eye(N, N, CV_32FC1) - (K * H) * m_covPredict;
    }
}

void SLAM::DataAssociation(const sgtdv_msgs::ConeArr::ConstPtr &cones, const geometry_msgs::Point &carPosition)
{
    for(std::list<sgtdv_msgs::Cone>::const_iterator it = m_coneCandidates.begin(); it != m_coneCandidates.end(); ++it)
    {
        it->coords;
    }
}

void SLAM::SetupNoiseMatrices()
{
    m_RT = (cv::Mat1f(3, 3) << 0.0000001f, 0.f, 0.f, 0.f, 0.0000001f, 0.f, 0.f, 0.f, 0.0000001f);
    m_QT = (cv::Mat1f(2, 2) << 0.0000001f, 0.f, 0.f, 0.0000001f);
}

double SLAM::GetDistance(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const
{
    double relativeX = p2.x - p1.x;
    double relativeY = p2.y - p1.y;
    double xBuff = p1.x * p2.x;
    double yBuff = p1.y * p2.y;
    return std::sqrt(xBuff * xBuff + yBuff * yBuff);
}

void SLAM::ModuloMatMembers(cv::Mat1f &mat, float modulo)
{    
    mat.copyTo(m_buff1);
    m_buff1 /= modulo;
    m_buff1.convertTo(m_buff2, CV_32SC1);
    mat -= m_buff2 * modulo;
}