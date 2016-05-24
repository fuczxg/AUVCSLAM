#ifndef LCM_UTILS_H
#define LCM_UTILS_H

#include "conversions.h"
#include "isam_coop.h"
#include <lcmtypes/isam_coop.hpp>
#include <Eigen/LU>
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace coop;

static rot3_t rotate_ypr(rot3_t rot_in, Vector6d pose){
  rot3_t rot_out;
  for(int i =0; i<3; i++){
    rot_out.mu[i] = rot_in.mu[i] - pose(i+3);
    for(int j = 0; j<3; j++){
      rot_out.omega[i][j] = rot_in.omega[i][j];
    }
  }
  return rot_out;
};

static depth_t shift_z(depth_t z_in, Vector6d pose){
  depth_t z_out;
  z_out.mu = z_in.mu - pose(2);
  z_out.omega = z_in.omega;
  return z_out;
};

static odom_xy_t rotate_odom(odom_xy_t odom_in, Vector6d pose){
  odom_xy_t odom_out = odom_in;
  double yaw = pose(3);
  Eigen::Matrix2d R;
  R << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
  Vector2d pos_mu_in;
  Matrix2d pos_omega_in;
  conversions::lcm2eigen(odom_in.pos2, pos_mu_in, pos_omega_in);
  Matrix2d pos_sigma_in = pos_omega_in.inverse();
  pos2_t pos_out;
  Vector2d pos_mu_out = R*pos_mu_in;
  Matrix2d pos_sigma_out = R*pos_sigma_in*R.transpose();
  Matrix2d pos_omega_out = pos_sigma_out.inverse();
  conversions::eigen2lcm(pos_mu_out,pos_omega_out,pos_out);
  odom_out.pos2 = pos_out;
  return odom_out;
}

static int64_t time_in_ms(){
  boost::posix_time::ptime p(boost::posix_time::microsec_clock::universal_time());
  return (int64_t)(p.time_of_day().total_milliseconds());
}

#endif
