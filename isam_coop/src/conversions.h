#pragma once
#include <Eigen/LU>
#include <lcmtypes/isam_coop.hpp>

class conversions{
 public:

static void lcm2eigen(coop::pose3_t pose, Eigen::MatrixXd &mu, Eigen::MatrixXd &omega){
  mu.resize(6,1);
  omega.resize(6,6);
  for(int i=0; i<6; i++){
    mu(i,0)=pose.mu[i];
  }
  for (int i = 0; i<6; i++){
    for (int j = 0; j<6; j++){
      omega(i,j)=pose.omega[i][j];
    }
  }
};

static void lcm2eigen(coop::pos2_t pos, Eigen::Vector2d &mu, Eigen::Matrix2d &omega){
  for(int i=0; i<2; i++){
    mu(i)=pos.mu[i];
  }
  for (int i = 0; i<2; i++){
    for (int j = 0; j<2; j++){
      omega(i,j)=pos.omega[i][j];
    }
  }
};

static void lcm2eigen(coop::rot3_t rot3, Eigen::Vector3d &mu, Eigen::Matrix3d &omega){
  for(int i=0; i<3; i++){
    mu(i)=rot3.mu[i];
  }
  for (int i = 0; i<3; i++){
    for (int j = 0; j<3; j++){
      omega(i,j)=rot3.omega[i][j];
    }
  }
};


static void lcm2eigen(coop::point3_t point, Eigen::MatrixXd &mu, Eigen::MatrixXd &omega){
  mu.resize(3,1);
  omega.resize(3,3);
  for(int i=0; i<3; i++){
    mu(i,0)=point.mu[i];
  }

  for (int i = 0; i<3; i++){
    for (int j = 0; j<3; j++){
      omega(i,j)=point.omega[i][j];
    }
  }
};

static void lcm2eigen(coop::sidescan_detection_t detection, Eigen::MatrixXd &omega){
  omega.resize(3,3);
  for (int i = 0; i<3; i++){
    for (int j = 0; j<3; j++){
      omega(i,j)=detection.omega[i][j];
    }
  }
};

static void eigen2lcm(Eigen::VectorXd mu, Eigen::MatrixXd omega, coop::pose3_t &p){
  for (int i=0; i<6; i++)
    p.mu[i] = mu[i];

  Eigen::MatrixXd sigma = omega.inverse();

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      p.omega[i][j] = omega(i,j);
      p.sigma[i][j] = sigma(i,j);
    }
  }
};


static void eigen2lcm(Eigen::VectorXd mu, Eigen::MatrixXd omega, coop::point3_t &p){
  for (int i=0; i<3; i++)
    p.mu[i] = mu[i];

  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      p.omega[i][j] = omega(i,j);
    }
  }
};

static void eigen2lcm(Eigen::VectorXd mu, Eigen::MatrixXd omega, coop::pos2_t &p){
  for (int i=0; i<2; i++)
    p.mu[i] = mu[i];

  Eigen::MatrixXd sigma = omega.inverse();

  for (int i=0; i<2; i++){
    for (int j=0; j<2; j++){
      p.omega[i][j] = omega(i,j);
      p.sigma[i][j] = sigma(i,j);
    }
  }
};


static void eigen2lcm(Eigen::VectorXd mu, Eigen::MatrixXd omega, coop::rot3_t &p){
  for (int i=0; i<3; i++)
    p.mu[i] = mu[i];

  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      p.omega[i][j] = omega(i,j);
};  

static void eigen2lcm(double mu, Eigen::MatrixXd omega, coop::sidescan_detection_t &detection){
  detection.mu = mu;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      detection.omega[i][j] = omega(i,j);
};

static void eigen2lcm(double mu, Eigen::MatrixXd omega, coop::depth_t &z){
  z.mu = mu;
  z.omega = omega(0,0);
};


}; //class
