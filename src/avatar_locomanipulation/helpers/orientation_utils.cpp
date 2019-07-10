#include <avatar_locomanipulation/helpers/orientation_utils.hpp>

namespace math_utils{
  void convert(const Eigen::Quaterniond & from, Eigen::AngleAxisd & to){
    to = from.normalized();
    if (to.angle() > M_PI){
      to.angle() -= (2*M_PI);
    }else if (to.angle() < -M_PI){
      to.angle() += (2*M_PI);
    }
  }

  void convert(const Eigen::AngleAxisd & from, Eigen::Vector3d & to){
    to = from.axis()*from.angle();
  }

  void convert(const Eigen::Quaterniond & from, Eigen::Vector3d & to){
    Eigen::AngleAxisd aa_tmp;
    convert(from, aa_tmp);
    convert(aa_tmp, to);
  }

  void compute_quat_error(const Eigen::Quaterniond & des, const Eigen::Quaterniond & current, Eigen::Vector3d & error){
    Eigen::AngleAxisd axis_angle;
    axis_angle = des*current.inverse();
    error = axis_angle.axis() * axis_angle.angle();
  }


  void printQuat(const Eigen::Quaterniond & quat){
    std::cout <<  quat.x() << " " <<
                  quat.y() << " " <<
                  quat.z() << " " <<
                  quat.w() << " " << std::endl;
  }


}
