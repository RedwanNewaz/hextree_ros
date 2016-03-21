#ifndef COMM_FUNCTION
#define COMM_FUNCTION
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "geometry_msgs/Quaternion.h"
#include <numeric>
#include <functional>





extern unsigned int ros_header_timestamp_base;
inline static int getMS(ros::Time stamp = ros::Time::now())
{
        if(ros_header_timestamp_base == 0)
                ros_header_timestamp_base = stamp.sec;
        int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;
        return mss;
}


struct ControlCommand
{
    inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
    inline ControlCommand(double roll, double pitch, double yaw, double gaz)
    {
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
        this->gaz = gaz;
    }
    double yaw, roll, pitch, gaz;
};

inline float min_ele_vec( std::vector<float>& v){
  std::vector<float>::iterator result = std::min_element(v.begin(), v.end());
  return result[0];
}

inline geometry_msgs::Pose euler2quaternion(float roll, float pitch, float yaw){

    geometry_msgs::Pose pose;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    pose.orientation.x=q.getX();
    pose.orientation.y=q.getY();
    pose.orientation.z=q.getZ();
    pose.orientation.w=q.getW();
    return pose;
}

inline std::string num2str(float a){
       std::stringstream debug_cntrle;
       debug_cntrle<<a;
       return debug_cntrle.str();

}

inline double mean( const std::vector<float>& v){
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}

inline double var(double mean, const std::vector<float>& v){
    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    return std::sqrt(sq_sum / v.size() - mean * mean);
}

inline bool nan_array(const float *a, int n){
    for( int i=0;i<n;i++)
        if(isnan(double(a[i])))
            return true;
    return false;
}

#endif // COMM_FUNCTION

