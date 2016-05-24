#pragma once

#include "isam/isam.h"
#include "lcmtypes/isam_coop.hpp"
#include <vector>
#include <Eigen/LU>
#include <string>
#include <visualization/viewer.hpp>
#include "isam_coop.h"


using namespace std;
using namespace isam;
using namespace Eigen;
using namespace coop;

class FactorGraph{
 public:
  FactorGraph(){};
  FactorGraph(int, int, bool viewer=false, const char* = NULL);
  virtual ~FactorGraph();

 public: // These are the inteface functions for adding factors: send in lcm types that correspond to measurements
  void add_global(global_t);
  void add_prior(pose3_t);
  bool add_odom_xy_global_yprz(odom_xy_global_yprz_t);
  bool add_sidescan_detection(sidescan_detection_t); // 
  bool add_virtual_pose_pose(virtual_pose3_pose3_t);
  bool add_virtual_pose_point(virtual_pose3_point3_t);
  bool add_relative_range(relative_range_t);


 public: //accessors
  bool      get_current_pose(int, pose3_t&);
  bool      get_current_pose(pose3_t& pose){get_current_pose(_id,pose);};
  int       num_poses(int id){return _poses[id].size();};
  int       num_features(){return _features.size();};
  list<int> get_feature_ids();
  int64_t   get_current_time(int);
  bool      get_trajectory(int, trajectory_t&);
  Pose3d    get_pose_estimate(int64_t, int); // get pose estimate from time and robot id 
  point3_t  get_point3_t(int); // get feature estimate from feature id
  int       get_id(){return _id;};
  void      get_sidescan_detections_t(int64_t,int,sidescan_detections_t&);
  odom_xy_global_yprz_t get_odom_xy_global_yprz(int64_t,int);

 public: // updaters
  void update();
  void batch();
  void update_viewer();  
  void print();

 public: 
  MatrixXd marginalize_poses_dense(int64_t, int64_t, int id = 1); //TODO there should be a more general one that takes a
                                            //     of times (for poses) or ids (for features)?
  MatrixXd marginalize_poses_sparse(int64_t, int64_t, int id = 1); // TODO some kind of way to specify the sparsity pattern that we want or virtual measurement types or something..



 private: // utility functions for building/managing nodes and factors
  Pose3dTS_Node*  find_pose_from_time_and_id(int64_t t, int id); 
  Pose3dTS_Node*  find_closest_pose_from_time_and_id(int64_t t, int id);
  Pose3dTS_Node*  build_pose(int64_t t, int id);
  Point3dID_Node* find_point_from_feature_id(int id);
  Point3dID_Node* build_point(int id);
  Pose3dTS_Node*  find_next_pose_from_pose(Pose3dTS_Node*);

 private: // internal functions for adding of factors
  void add_odom_xy(odom_xy_t,Pose3dTS_Node*, Pose3dTS_Node*);
  void add_global_ypr(rot3_t,Pose3dTS_Node*);
  void add_global_z(depth_t,Pose3dTS_Node*);
  void add_global_xy(pos2_t,Pose3dTS_Node*);

 private: // internal getter functions (could be public if necessary)
  odom_xy_t get_odom_xy(int64_t, int, Pose3dTS_Node* = NULL);
  rot3_t    get_global_ypr(int64_t, int, Pose3dTS_Node* = NULL);
  depth_t   get_global_z(int64_t, int, Pose3dTS_Node* = NULL);


 private:
  int _id; // the id of the robot where this is running. Note that we will use convention of starting with 1 for the id's. The reasoning for this is that in AUV Acomms the 0 slot is usually reserved for broadcast
  int _N; // The number of robots

  vector<vector<Pose3dTS_Node*> > _poses; // all the vehicle poses stored locally
  vector<Point3dID_Node*> _features; // all locally tracked features

  Slam _slam;

 private:
  bool _lcm_viewer;
  Viewer* _viewer;
  LinkCollection* _range_collection;
  LinkCollection* _feature_collection;
  LinkCollection* _virtual_collection;
  int _virtual_link_count;
  int _range_link_count;
  int _feature_link_count;
  lcm_t* lcm;

};
