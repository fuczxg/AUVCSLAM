#include "protobufs/factors.pb.h"
#include <lcmtypes/isam_coop.hpp>


void lcm2proto(coop::virtual_pose3_pose3_t vpp_p, virtual_pose3d_pose3d_measurement* proto_p){

  coop::pose3_t lcm_p = vpp_p.pose;
  proto_p->set_mu_x(lcm_p.mu[0]);
  proto_p->set_mu_y(lcm_p.mu[1]);
  proto_p->set_mu_z(lcm_p.mu[2]);
  proto_p->set_mu_psi(lcm_p.mu[3]);
  proto_p->set_omega_xx(lcm_p.omega[0][0]);
  proto_p->set_omega_xy(lcm_p.omega[0][1]);
  proto_p->set_omega_yy(lcm_p.omega[1][1]);
  proto_p->set_t1(vpp_p.t[0]);
  proto_p->set_t2(vpp_p.t[1]);
}

void lcm2proto(coop::global_t global_lcm, global_measurement* global_proto){
  global_proto->set_x(global_lcm.pos.mu[0]);
  global_proto->set_y(global_lcm.pos.mu[1]);
  global_proto->set_z(global_lcm.depth.mu);
  global_proto->set_psi(global_lcm.attitude.mu[0]);
  global_proto->set_theta(global_lcm.attitude.mu[1]);
  global_proto->set_phi(global_lcm.attitude.mu[2]);
  global_proto->set_t(global_lcm.t);
}

void lcm2proto(coop::relative_range_t range_lcm, relative_range_measurement* range_proto){
  range_proto->set_send_time(range_lcm.send_t);
  range_proto->set_receive_time(range_lcm.receive_t);
  range_proto->set_send_id(range_lcm.send_id);
  range_proto->set_receive_id(range_lcm.receive_id);
  range_proto->set_range(range_lcm.mu);
}

void lcm2proto(coop::virtual_pose3_point3_t vpp_p, virtual_pose3d_point3d_measurement* proto_p){
  coop::point3_t lcm_p = vpp_p.point;
  proto_p->set_mu_x(lcm_p.mu[0]);
  proto_p->set_mu_y(lcm_p.mu[1]);
  proto_p->set_mu_z(lcm_p.mu[2]);
  proto_p->set_omega_xx(lcm_p.omega[0][0]);
  proto_p->set_omega_xy(lcm_p.omega[0][1]);
  proto_p->set_omega_yy(lcm_p.omega[1][1]);
  proto_p->set_omega_zz(lcm_p.omega[2][2]);
  proto_p->set_t(vpp_p.t);
  proto_p->set_feature_id(vpp_p.feature_id);
}

void proto2lcm(virtual_pose3d_pose3d_measurement proto_p, 
	       coop::virtual_pose3_pose3_t& pose_pose_lcm){
  coop::pose3_t lcm_p;
  lcm_p.mu[0] = proto_p.mu_x();
  lcm_p.mu[1] = proto_p.mu_y();
  lcm_p.mu[2] = proto_p.mu_z();
  lcm_p.mu[3] = proto_p.mu_psi();

  lcm_p.omega[0][0] = proto_p.omega_xx();
  lcm_p.omega[0][1] = proto_p.omega_xy();
  lcm_p.omega[1][1] = proto_p.omega_yy();

  pose_pose_lcm.pose = lcm_p;
  pose_pose_lcm.t[0] = proto_p.t1();
  pose_pose_lcm.t[1] = proto_p.t2();
}


void proto2lcm(virtual_pose3d_point3d_measurement proto_p, 
	       coop::virtual_pose3_point3_t& pose_point_lcm){
  coop::point3_t lcm_p;
  lcm_p.mu[0] = proto_p.mu_x();
  lcm_p.mu[1] = proto_p.mu_y();
  lcm_p.mu[2] = proto_p.mu_z();

  lcm_p.omega[0][0] = proto_p.omega_xx();
  lcm_p.omega[0][1] = proto_p.omega_xy();
  lcm_p.omega[1][1] = proto_p.omega_yy();
  lcm_p.omega[2][2] = proto_p.omega_zz();

  pose_point_lcm.point = lcm_p;
  pose_point_lcm.t = proto_p.t();
  pose_point_lcm.feature_id = proto_p.feature_id();
}
 
void proto2lcm(relative_range_measurement proto_range,
	       coop::relative_range_t& lcm_range){
  lcm_range.send_t = proto_range.send_time();
  lcm_range.receive_t = proto_range.receive_time();
  lcm_range.send_id = proto_range.send_id();
  lcm_range.receive_id = proto_range.receive_id();
  lcm_range.mu = proto_range.range();
  // omega not included in proto since never sent (assumed known and const)
}
 
void proto2lcm(global_measurement global_proto,
	       coop::global_t& global_lcm){
  
  global_lcm.t = global_proto.t();
  global_lcm.pos.mu[0] = global_proto.x();
  global_lcm.pos.mu[1] = global_proto.y();
  global_lcm.depth.mu = global_proto.z();
  global_lcm.attitude.mu[0] = global_proto.psi();
  global_lcm.attitude.mu[1] = global_proto.theta();
  global_lcm.attitude.mu[2] = global_proto.phi();
  // all covariance not included since not transmitted
}
