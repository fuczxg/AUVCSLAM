#include "FactorGraph.h"
#include "conversions.h"
#include <lcm/lcm.h> // to send to viewer
#include <assert.h>
#include <fstream>
#include <lcm_utils.h>

#define DEBUG 1

FactorGraph::FactorGraph(int N, int id, bool lcm_viewer, const char* provider):_id(id),_N(N),_lcm_viewer(lcm_viewer){

  for(int i=0; i<=N ; i++){ // weird that it goes to N but the 0 vector will be left empty
    vector<Pose3dTS_Node*> v;
    _poses.push_back(v);
  }
  if(_lcm_viewer){
    lcm = lcm_create(provider);
    _viewer = new Viewer(lcm); // viewer wants the c structure  not c++ wrapper
    _feature_collection = new LinkCollection(1, string("Sidescan Detections"));
    _range_collection = new LinkCollection(4, string("Range Measurements"));
    _virtual_collection = new LinkCollection(5, string("Virtual Measurements"));
    _virtual_link_count = 0;
    _range_link_count = 0;
    _feature_link_count = 0;
  }
  Properties props;
  props.method = GAUSS_NEWTON;
  _slam.set_properties(props);


}

FactorGraph::~FactorGraph(){
  if(_lcm_viewer){
    delete _viewer;
    delete _virtual_collection;
    delete _feature_collection;
    delete _range_collection;
  }
}



// add_global: Find the right pose and add the global factor. If there's no pose at the correct time build it.
void FactorGraph::add_global(global_t global){
  pose3_t prior;
  prior.t = global.t;
  prior.id = global.id;
  for (int i = 0; i<2; i++){
    prior.mu[i] = global.pos.mu[i];
    for (int j = 0; j<2; j++){
      prior.omega[i][j] = global.pos.omega[i][j];
    }
  }
  prior.mu[2] = global.depth.mu;
  prior.omega[2][2] = global.depth.omega;
  for (int i = 0; i<3 ; i++){
    prior.mu[i+3] = global.attitude.mu[i];
    for (int j = 0; j<3 ; j++){
      prior.omega[i+3][j+3] = global.attitude.omega[i][j];
    }
  }
  add_prior(prior);

}

void FactorGraph::add_prior(pose3_t prior){
  Pose3dTS_Node* pose  = find_pose_from_time_and_id(prior.t,prior.id);
  if (pose == NULL){
    pose = build_pose(prior.t,prior.id);
  }
  MatrixXd mu = MatrixXd::Zero(6,1);
  MatrixXd omega = MatrixXd::Zero(6,6); 
  conversions::lcm2eigen(prior,mu,omega);
  Pose3d measure(mu);
  Noise inf = Information(omega);
  Pose3d_Factor* prior_factor = new Pose3d_Factor(dynamic_cast<Pose3d_Node*>(pose), measure, inf);
  _slam.add_factor(prior_factor);
  if(1) cout << "adding prior factor: " << endl << *prior_factor << endl;
  batch();
}

// add_odom_xy_global_yprz_t: sets up the pose nodes and then calls the factor adding functions
bool FactorGraph::add_odom_xy_global_yprz(odom_xy_global_yprz_t odom){
  if (_poses[odom.id].size()==0){
    cout << "Rejecting odometry since no prior" << endl;
    return false;
  }
  Pose3dTS_Node* old_pose = _poses[odom.id].back();
  if (_poses[odom.id].back()->ts() != odom.t[0])
    cout << "Warning: Time mismatch for last pose time " << _poses[odom.id].back()->ts() << "and received" << odom.t[0] << endl;

  // for now just build a new pose and connect it to the last one for that id
  Pose3dTS_Node* pose =   build_pose(odom.t[1],odom.id);

  // manually initialize the new pose since all the factors are partial
  pos2_t pos2 = odom.odom_xy.pos2;
  rot3_t rot3 = odom.attitude;
  depth_t depth = odom.depth;
  Pose3d old  = (dynamic_cast<Pose3d_Node*>(old_pose))->value();
  Pose3d measure(pos2.mu[0], pos2.mu[1],0,0,0,0);
  Pose3d p2 = (MatrixXd(old.vector() + measure.vector()));
  Pose3d predict(p2.x(),p2.y(),depth.mu,rot3.mu[0],rot3.mu[1],rot3.mu[2]);
  pose->init(predict);

  // call private factor adding functions
  add_odom_xy(odom.odom_xy,old_pose,pose);
  add_global_ypr(odom.attitude,pose);
  add_global_z(odom.depth,pose);

  return true;

}


bool FactorGraph::add_relative_range(relative_range_t range){

  if (_poses[range.send_id].size()==0){
    cout << "Rejecting odometry since no prior for sender" << endl;
    return false;
  }
  if (_poses[range.receive_id].size()==0){
    cout << "Rejecting odometry since no prior for receiver" << endl;
    return false;
  }

  if (1) cout << "add_range looking for sender pose with time " << range.send_t
       << " and id " << range.send_id << endl;
  Pose3dTS_Node* sender_pose = find_pose_from_time_and_id(range.send_t,range.send_id);
  if (1) cout << "add_range looking for receiver pose with time " << range.receive_t
       << " and id " << range.receive_id << endl;
  Pose3dTS_Node* receiver_pose = find_pose_from_time_and_id(range.receive_t,range.receive_id);

  if (sender_pose == NULL)
    sender_pose =find_closest_pose_from_time_and_id(range.send_t,range.send_id);
  if (receiver_pose == NULL)
    receiver_pose = find_closest_pose_from_time_and_id(range.receive_t,range.receive_id);

  if (sender_pose == NULL || receiver_pose == NULL){
    cout << "unable to find pose" <<endl;
    return false;
  }
  MatrixXd mat(1,1);
  
  mat <<  range.omega;
  Noise inf = Information(mat);
  Pose3d_Pose3d_RangeFactor* range_measure
    = new Pose3d_Pose3d_RangeFactor(sender_pose,receiver_pose,range.mu,inf);
  _slam.add_factor(range_measure);
  if(_lcm_viewer)
    _range_collection->add(_range_link_count++,6,sender_pose->unique_id(),2,receiver_pose->unique_id());
  
  return true;
}

//virtual_pose_pose
bool FactorGraph::add_virtual_pose_pose(virtual_pose3_pose3_t virtual_pose_pose){
  int id = virtual_pose_pose.id;
  int64_t start_time = virtual_pose_pose.t[0];
  int64_t end_time = virtual_pose_pose.t[1];
  if (_poses[id].size()==0){
    cout << "Rejecting odometry since no prior" << endl;
    return false;
  }
  Pose3dTS_Node* old_pose = find_pose_from_time_and_id(start_time,id);
  if (old_pose == NULL){
    cout << "[warning] {add_virtual_pose_pose} no node for id " << id << " and time  " << start_time << " skipping.." << endl;
    return false;
  }

  // could decompose in the case that start_time does not correspond to the last
  // pose in the chain but not sure it matters...

  Pose3dTS_Node* pose = build_pose(end_time,id);
  MatrixXd mu,omega;
  conversions::lcm2eigen(virtual_pose_pose.pose,mu,omega);
  Pose3d measure(mu);
  Noise inf = Information(omega);
  Virtual_Pose3d_Pose3d_Factor* vppf = 
    new Virtual_Pose3d_Pose3d_Factor(dynamic_cast<Pose3d_Node*>(old_pose), 
			     dynamic_cast<Pose3d_Node*>(pose), measure, inf);
  if (DEBUG) cout << "adding virtual pose pose factor: " << endl << *vppf << endl;
  _slam.add_factor(vppf);
  if(_lcm_viewer)
    _virtual_collection->add(_virtual_link_count++,6,old_pose->unique_id(),6,pose->unique_id());

  return true;
}


bool FactorGraph::add_virtual_pose_point(virtual_pose3_point3_t virtual_pose_point){
  int id = virtual_pose_point.id;
  if (DEBUG) cout << "in FactorGraph trying to add virtual pose point at time: " << endl << "t = " << virtual_pose_point.t << endl ;

  Pose3dTS_Node* pose = find_pose_from_time_and_id(virtual_pose_point.t, id);
  if (pose == NULL){
    cout << "[warning] {add_virtual_pose_point}: no pose for virtual_pose_point measurement at time " << virtual_pose_point.t << " for id " << id << "skipping.. " <<  endl;
    return false;
  }
  // TODO there is also data association going on here (currently done with ids):
  Point3dID_Node* point  = find_point_from_feature_id(virtual_pose_point.feature_id);
  if(point == NULL)
    point = build_point(virtual_pose_point.feature_id);
  MatrixXd mu,omega;
  conversions::lcm2eigen(virtual_pose_point.point,mu,omega);
  Point3d measure(mu);
  Noise inf = Information(omega);
  Virtual_Pose3d_Point3d_Factor* virtual_factor = 
    new Virtual_Pose3d_Point3d_Factor(dynamic_cast<Pose3d_Node*>(pose),
			      dynamic_cast<Point3d_Node*>(point),
			      measure,
			      inf);
  if (DEBUG) cout << "pose point factors: " << *virtual_factor << endl;
  _slam.add_factor(virtual_factor);
  if(_lcm_viewer)
    _virtual_collection->add(_virtual_link_count++,6,pose->unique_id(),3,point->unique_id());
  return true;
}


bool FactorGraph::add_sidescan_detection(sidescan_detection_t detection){
  if (_poses[detection.id].size()==0){
    cout << "Rejecting odometry since no prior" << endl;
    return false;
  }

  Pose3dTS_Node* pose = find_pose_from_time_and_id(detection.t,detection.id); 
  
  if (pose == NULL){ // times might not be exact pick the closest one.
    cout << "No exact time match find closest" << endl;
    pose = find_closest_pose_from_time_and_id(detection.t,detection.id);
  }

  if (pose == NULL) cout << "Error unable to find pose for time = " << detection.t << " and id " << detection.id << endl;

  // TODO: decide if it's a detection of previously tracked target. EG do something like nearest neighbor or JCBB. Start with using feature id

  Point3dID_Node* point = find_point_from_feature_id(detection.feature_id);
  if (point == NULL)
    point = build_point(detection.feature_id);

  // here's the unique thing for a sidescan detection. It is always orthogonal to direction of motion. Therefore take the measurement and put it into "y" direction
  MatrixXd omega;
  conversions::lcm2eigen(detection,omega);
  Point3d measure(0.0,detection.mu,0.0); // TODO deal with depth/z, this sidescan detection should be
                                         // its own factor not use Pose3d_Point3d_Factor
  Noise inf = Information(omega);
  Pose3d_Point3d_Factor* detection_factor = 
    new Pose3d_Point3d_Factor(dynamic_cast<Pose3d_Node*>(pose),
			      dynamic_cast<Point3d_Node*>(point),
			      measure,
			      inf);
  if (DEBUG){ 
    cout << "Adding sidescan detection factor: " << *detection_factor << endl;
    cout << "Between pose with id = " << detection.id << " and time t = " << detection.t << "and point with feature id " << detection.feature_id << endl;
  }
  _slam.add_factor(detection_factor);
  if(_lcm_viewer)
    _feature_collection->add(_feature_link_count++,2,pose->unique_id(),3,point->unique_id());
  return true;
}


bool FactorGraph::get_current_pose(int id, pose3_t &p){
  if (_poses[id].size()  == 0){
    cout << "In get current pose but no pose to get " << endl;
    return false;
  }
  list<Node*> node; 
  node.push_back(_poses[id].back());
  Covariances::node_lists_t node_lists;
  node_lists.push_back(node);
  conversions::eigen2lcm(_poses[id].back()->vector0(),
	    (_slam.covariances().marginal(node_lists).front()).inverse(),
	    p);
  return true;
}

bool FactorGraph::get_trajectory(int id, trajectory_t &traj){
  if (_poses[id].size()  == 0){
    cout << "In get trajectory but no poses" << endl;
    return false;
  }
  traj.num_poses = _poses[id].size();
  traj.poses.resize(traj.num_poses);
  Covariances::node_lists_t node_lists;
  list<Node*> nodes;
  for(int i = 0; i<_poses[id].size(); i++){
    nodes.push_back(_poses[id][i]);
    node_lists.push_back(nodes);
    nodes.clear();
  }
  list<MatrixXd> cov_blocks = _slam.covariances().marginal(node_lists);
  int i = 0;
  for(list<MatrixXd>::iterator it = cov_blocks.begin(); it!= cov_blocks.end(); it++,i++){
    pose3_t p;
    conversions::eigen2lcm(_poses[id][i]->vector0(),
			   (*it).inverse(),
			   p);
    p.id = id;
    p.t  = _poses[id][i]->ts();
    traj.poses[i] = p;
  }
  return true;
}

list<int> FactorGraph::get_feature_ids(){
  list<int> ids;
  for(int i=0; i<_features.size(); i++){
    ids.push_back(_features[i]->id());
  }
  return ids;
}

int64_t FactorGraph::get_current_time(int id){
  if (_poses[id].size()  == 0){
    cout << "In get current time but no poses" << endl;
    return 0;
  }  
  return _poses[id].back()->ts();
}

Pose3d FactorGraph::get_pose_estimate(int64_t t, int id){
  Pose3dTS_Node* pose_node = find_pose_from_time_and_id(t,id);
  assert(pose_node != NULL);
  return pose_node->value();
}

point3_t FactorGraph::get_point3_t(int f_id){
  point3_t p;
  Point3dID_Node* point_node = find_point_from_feature_id(f_id);
  assert(point_node != NULL);
  list<Node*> node;
  node.push_back(point_node);
  Covariances::node_lists_t node_lists;
  node_lists.push_back(node);
  conversions::eigen2lcm(point_node->vector(),
			 (_slam.covariances().marginal(node_lists).front()).inverse(),p);
  return p;
}



void FactorGraph::get_sidescan_detections_t(int64_t t, int id, sidescan_detections_t& ds){
  ds.n = 0;
  // Step 0: find the right pose node at the given time and id
  Pose3dTS_Node* pose_node = find_pose_from_time_and_id(t,id);
  list<Factor*> factors = pose_node->factors();
  list<Factor*>::iterator it;
  // Step 1: find all the sidescan measurement factors 
  for(it = factors.begin(); it!=factors.end(); it++){
    if((*it)->name() == "Pose3d_Point3d_Factor"){
      sidescan_detection_t d;
      // Step 2: find the Point node on the other end of the factor
      vector<Node* > node_vec = (*it)->nodes();
      Point3dID_Node* point_id_node;
      if (node_vec[0]->unique_id() == pose_node->unique_id())
	point_id_node = dynamic_cast<Point3dID_Node*>(node_vec[1]);
      else
	point_id_node = dynamic_cast<Point3dID_Node*>(node_vec[0]);

      // Step 3: find the measurement mean and informaiton matrix
      Pose3d_Point3d_Factor* f = dynamic_cast<Pose3d_Point3d_Factor*>(*it);
      double mu = (f->measurement()).y(); // 
      MatrixXd omega = ((f->sqrtinf()).transpose())*(f->sqrtinf());

      // Step 4: add all the stuff to the detection
      d.id              = id;
      d.t               = t;
      d.feature_id      = point_id_node->id();
      conversions::eigen2lcm(mu,omega,d);

      // Step 5: add this detection to the larger set of detections
      ds.n = ds.n + 1;
      ds.detections.resize(ds.n);
      ds.detections[ds.n-1] = d;
    }
  }
}


odom_xy_global_yprz_t FactorGraph::get_odom_xy_global_yprz(int64_t t, int id){
  odom_xy_global_yprz_t odom;
  Pose3dTS_Node* pose_node = find_pose_from_time_and_id(t,id);
  odom_xy_t odom_xy = get_odom_xy(t,id,pose_node);
  Pose3dTS_Node* next_node = find_next_pose_from_pose(pose_node);
  rot3_t rot3 = get_global_ypr(odom_xy.t[1],id,next_node);
  depth_t z = get_global_z(odom_xy.t[1],id,next_node);

  odom.odom_xy  = odom_xy;
  odom.attitude = rot3;
  odom.depth    = z;
  odom.t[0]     = odom_xy.t[0];
  odom.t[1]     = odom_xy.t[1];
  odom.id       = odom_xy.id;
  
  return odom;
}



// keep the poses at start_t and end_t but get rid of all of the ones in between
// don't like how this function is written. Ideally it would be a list of times or feature ids to be removed?
// Currently returning the marginal information, should change this?
// TODO For cooperative localization work this should work with no landmarks at all. 
// TODO should make an LCM type for a dense factor? 
MatrixXd FactorGraph::marginalize_poses_dense(int64_t start_t, int64_t end_t, int id){
  
  cout << "1: " << time_in_ms() << endl;

  // step 1: Get the information matrix:
  SparseSystem Js = _slam.jacobian();
  MatrixXd J(Js.num_rows(), Js.num_cols());
  for (int r=0; r<Js.num_rows(); r++) {
    for (int c=0; c<Js.num_cols(); c++) {
      J(r,c) = Js(r,c);
    }
  }

  cout << "2: " << time_in_ms() << endl;

  //TODO really should go through and find the markov blanket explicitly. For now I just know that its all landmarks as well as the start and end poses.

  Pose3dTS_Node* start_node = find_pose_from_time_and_id(start_t, id);
  Pose3dTS_Node* end_node   = find_pose_from_time_and_id(end_t,   id);

  MatrixXd J2(J.rows(),J.cols());

  int current_col =0;
  // first and last pose
  for(int i = 0; i< start_node->dim(); i++){
    J2.col(i) = J.col(start_node->start() + i);
    J2.col(i+start_node->dim()) = J.col(end_node->start() + i);
  }
  current_col += start_node->dim() + end_node->dim();

  // landmarks:
  for(int l = 0; l<_features.size(); l++){
    for(int i = 0; i<_features[l]->dim(); i++){
      J2.col(current_col+i) = J.col(_features[l]->start()+i);
    }
    current_col += _features[l]->dim();
  }

  // intermediate poses
  Pose3dTS_Node* next_node = find_next_pose_from_pose(start_node);
  while(next_node != end_node){
    for(int i = 0; i< next_node->dim(); i++){
      J2.col(current_col+i) = J.col(next_node->start() + i);
    }
    current_col+=next_node->dim();
    next_node = find_next_pose_from_pose(next_node);
  }

  cout << "3: " << time_in_ms() << endl;

  MatrixXd H(J2.cols(),J2.cols());
  H = J2.transpose() * J2; // inf matrix
  //  ofstream myfile;
  //  myfile.open("H.txt");
  //  myfile << H;
  //  myfile.close();


  cout << "4: " << time_in_ms() << endl;
  

  // step 4: schur complement.
  MatrixXd A(12 + _features.size()*3, 12 + _features.size()*3);
  A = H.topLeftCorner(12 + _features.size()*3, 12 + _features.size()*3);
  MatrixXd B(12 + _features.size()*3, 6*(_poses[id].size()-2));
  B = H.topRightCorner(12 + _features.size()*3, 6*(_poses[id].size()-2));
  MatrixXd C(6*(_poses[id].size()-2),12 + _features.size()*3);
  C = H.bottomLeftCorner(6*(_poses[id].size()-2),12 + _features.size()*3);
  MatrixXd D(6*(_poses[id].size()-2),6*(_poses[id].size()-2));
  D = H.bottomRightCorner(6*(_poses[id].size()-2),6*(_poses[id].size()-2));
    
  MatrixXd H_m(12+_features.size()*3, 12+_features.size()*3);
  H_m = A - B*(D.inverse())*C;

  cout << "5: " << time_in_ms() << endl;

  // TODO for dense should build constraint here and add as well as remove all of the poses that are being marginalized (including the factors which I think happens automatically)  maybe look at how Nick C-B does the dense constraints... it's a bit tricky maybe to get general and I'm not sure I really need it... Might be able to directly use the GLC formulation

  return H_m;

}


// TODO - cleaner API for returning sparse factors than as one big matrix
MatrixXd FactorGraph::marginalize_poses_sparse(int64_t start_t, int64_t end_t, int id){



  MatrixXd H_m = marginalize_poses_dense(start_t, end_t);
  MatrixXd J3 = MatrixXd::Zero(H_m.rows(),H_m.cols());
  MatrixXd I6 = MatrixXd::Identity(6,6);
  MatrixXd I3 = MatrixXd::Identity(3,3);
  
  J3.block(0,0,6,6) = I6; // TODO don't like hardcoded 6's
  J3.block(6,0,6,6) = I6;
  J3.block(6,6,6,6) = -I6;
  
  for(int i = 12; i<J3.rows(); i+=3){
    J3.block(i,0,3,3) = I3;
    J3.block(i,i,3,3) = -I3;
  }

  MatrixXd X(H_m.rows(),H_m.cols());
  X = J3*H_m.inverse()*J3.transpose(); //equation 10 from Mazuran_RSS_2014

  cout << "6: " << time_in_ms() << endl;

  // TODO: Project into consistency constraint space.
  //TODO add the constraints  
  return X;
}

// return the next pose in the pose chain
Pose3dTS_Node* FactorGraph::find_next_pose_from_pose(Pose3dTS_Node* pose_node){
  Pose3dTS_Node* other_node;
  list<Factor*> factors = pose_node->factors();
  list<Factor*>::iterator it;
  for(it = factors.begin();it != factors.end();it++){
    if((*it)->name() == "Pose3d_Pose3d_PartialXY_Factor"){
      vector<Node* > node_vec = (*it)->nodes();
      if (node_vec[0]->unique_id() == pose_node->unique_id())
	other_node = dynamic_cast<Pose3dTS_Node*>(node_vec[1]);
      else
	other_node = dynamic_cast<Pose3dTS_Node*>(node_vec[0]);
      if (other_node->ts() > pose_node->ts()){ // we want this one
	return other_node;
      }
    }
  }
  // TODO throw an error
  return NULL;
}


void FactorGraph::update(){
  _slam.update();
}

void FactorGraph::batch(){
  cout << "in FG starting batch" << endl;
  _slam.batch_optimization();
}


void FactorGraph::add_odom_xy(odom_xy_t odom, Pose3dTS_Node* old_pose, Pose3dTS_Node* new_pose){
  Vector2d mu;
  Matrix2d omega;
  conversions::lcm2eigen(odom.pos2,mu,omega);
  Noise inf = Information(omega);
  Pose3d_Pose3d_PartialXY_Factor* odom_factor = 
    new Pose3d_Pose3d_PartialXY_Factor(dynamic_cast<Pose3d_Node*>(old_pose), 
			     dynamic_cast<Pose3d_Node*>(new_pose), mu, inf);
  _slam.add_factor(odom_factor);

}

void FactorGraph::add_global_ypr(rot3_t rot3, Pose3dTS_Node* pose){
  Vector3d mu;
  Matrix3d omega;
  conversions::lcm2eigen(rot3,mu,omega);
  Rot3d mu_rot3(mu(0),mu(1),mu(2));
  Noise inf = Information(omega);
  Pose3d_PartialYPR_Factor* global_factor = new Pose3d_PartialYPR_Factor(dynamic_cast<Pose3d_Node*>(pose), mu_rot3, inf);
  _slam.add_factor(global_factor);
}

void FactorGraph::add_global_xy(pos2_t pos2, Pose3dTS_Node* pose){
  Vector2d mu;
  Matrix2d omega;
  conversions::lcm2eigen(pos2,mu,omega);
  Noise inf = Information(omega);
  Pose3d_PartialXY_Factor* global_factor = new Pose3d_PartialXY_Factor(dynamic_cast<Pose3d_Node*>(pose), mu, inf);
  _slam.add_factor(global_factor);
}

void FactorGraph::add_global_z(depth_t depth, Pose3dTS_Node* pose){
  double mu;
  MatrixXd omega(1,1);
  mu = depth.mu;
  omega(0,0) = depth.omega;
  Noise inf = Information(omega);
  Pose3d_PartialZ_Factor* global_factor = new Pose3d_PartialZ_Factor(dynamic_cast<Pose3d_Node*>(pose),mu,inf);
  _slam.add_factor(global_factor);
}


odom_xy_t FactorGraph::get_odom_xy(int64_t t, int id, Pose3dTS_Node* pose_node){
  if (pose_node == NULL)
    pose_node = find_pose_from_time_and_id(t,id);
  odom_xy_t odom_xy;
  pos2_t    pos2;
  list<Factor*> factors = pose_node->factors();
  list<Factor*>::iterator it;
  for(it = factors.begin();it != factors.end();it++){
    if((*it)->name() == "Pose3d_Pose3d_PartialXY_Factor"){
      vector<Node* > node_vec = (*it)->nodes();
      Pose3dTS_Node* other_node;
      if (node_vec[0]->unique_id() == pose_node->unique_id())
	other_node = dynamic_cast<Pose3dTS_Node*>(node_vec[1]);
      else
	other_node = dynamic_cast<Pose3dTS_Node*>(node_vec[0]);
      if (other_node->ts() > pose_node->ts()){ // we want this odom
	Pose3d_Pose3d_PartialXY_Factor* f = dynamic_cast<Pose3d_Pose3d_PartialXY_Factor*>(*it);
	VectorXd mu    = f->measurement(); // 
	MatrixXd omega = ((f->sqrtinf()).transpose())*(f->sqrtinf());
	conversions::eigen2lcm(mu, omega, pos2);
	odom_xy.pos2 = pos2;
	odom_xy.t[0] = pose_node->ts();
	odom_xy.t[1] = other_node->ts();
	odom_xy.id   = id;
      }
    }
  }
  return odom_xy;
}

rot3_t FactorGraph::get_global_ypr(int64_t t, int id, Pose3dTS_Node* pose_node){
  if (pose_node == NULL)
    pose_node = find_pose_from_time_and_id(t,id);
  rot3_t rot3;
  list<Factor*> factors = pose_node->factors();
  list<Factor*>::iterator it;
  for(it = factors.begin();it != factors.end();it++){
    if((*it)->name() == "Pose3d_PartialYPR_Factor"){
      //get the other node in the factor and then discard if it is next one (want previous)
      Pose3d_PartialYPR_Factor* f = dynamic_cast<Pose3d_PartialYPR_Factor*>(*it);
      Rot3d    mu_rot3 = f->measurement();
      VectorXd mu(3);
      mu<<mu_rot3.yaw(),mu_rot3.pitch(),mu_rot3.roll();
      MatrixXd omega = ((f->sqrtinf()).transpose())*(f->sqrtinf());
      conversions::eigen2lcm(mu, omega, rot3);
    }
  }
  return rot3;
}

depth_t FactorGraph::get_global_z(int64_t t, int id, Pose3dTS_Node* pose_node){
  if (pose_node == NULL)
    pose_node = find_pose_from_time_and_id(t,id);
  depth_t z;
  list<Factor*> factors = pose_node->factors();
  list<Factor*>::iterator it;
  for(it = factors.begin();it != factors.end();it++){
    if((*it)->name() == "Pose3d_PartialZ_Factor"){
      //get the other node in the factor and then discard if it is next one (want previous)
      Pose3d_PartialZ_Factor* f = dynamic_cast<Pose3d_PartialZ_Factor*>(*it);
      double   mu    = f->measurement(); // 
      MatrixXd omega = ((f->sqrtinf()).transpose())*(f->sqrtinf());
      conversions::eigen2lcm(mu, omega, z);
    }
  }
  return z;
}


// general node building function. 
Pose3dTS_Node* FactorGraph::build_pose(int64_t t, int id){
  Pose3dTS_Node* pose = new Pose3dTS_Node(t);
  _slam.add_node(pose); // add it to _slam
  _poses[id].push_back(pose); // store it locally in the correct spot
  return pose;
}
  


//Find the Pose in the vector where the time t id. Search backwards since much likelier to be near the back of the vector. If never found return NULL pointer
Pose3dTS_Node* FactorGraph::find_pose_from_time_and_id(int64_t t, int id){
  for(int i = _poses[id].size() - 1; i >= 0; i--){
    if (_poses[id][i]->ts() == t)
      return _poses[id][i];
  }
  return NULL;
}

//Find the pose in the vector where the time difference is minimum.

Pose3dTS_Node* FactorGraph::find_closest_pose_from_time_and_id(int64_t t, int id){
  if(_poses[id].empty()) return NULL;
  int64_t min_t = abs(_poses[id].back()->ts() - t);
  int min_i = _poses[id].size()-1;
  for(int i = _poses[id].size()-1;i>=0;i--){
    if(abs(_poses[id][i]->ts() - t) < min_t){
      min_t = abs(_poses[id][i]->ts() - t);
      min_i = i;
    }
    if(_poses[id][i]->ts() - t < 0){ // as soon as difference becomes negative, return the node
      if (1) cout << "{find_closest_pose_from_time_and_id} Looking for: " << t << " found " << _poses[id][i]->ts() << endl;
      if (min_t > 1000) // 1 sec
	cout << "[WARNING] {find_closest_pose_from_time_and_id} time difference quite high suggests a problem: " << min_t << endl;
      return _poses[id][min_i];
    }
  }
  cout << "Warning: find_closest_pose_from_time_and_id - returning first node" << endl;
  return _poses[id][0];
}

//try to find a feature based on id otherwise build a bran new one add it and return it. In the future this should be replaced by something more rigorous in terms of data association
Point3dID_Node* FactorGraph::find_point_from_feature_id(int id){
  for (int i =0; i<_features.size(); i++){
    if(_features[i]->id() == id){
      return _features[i];
    }
  }
  return NULL;
}

Point3dID_Node* FactorGraph::build_point(int id){
   Point3dID_Node* new_point = new Point3dID_Node(id);
  _slam.add_node(new_point);
  _features.push_back(new_point);
  return new_point;
}

    

void FactorGraph::update_viewer(){
  ObjectCollection own_pose_collection(2,string("Own Poses"),VS_OBJ_COLLECTION_T_POSE3D);
  ObjectCollection other_pose_collection(6,string("Other Poses"),VS_OBJ_COLLECTION_T_POSE3D);
  ObjectCollection feature_collection(3,string("Features"),VS_OBJ_COLLECTION_T_SQUARE);
  for (int j = 0; j<_poses[_id].size(); j++)
    own_pose_collection.add(_poses[_id][j]->unique_id(),_poses[_id][j]->value());
  for(int i = 1; i<= _N; i++){
    if (i == _id) continue;
    for(int j = 0; j < _poses[i].size(); j++){
      other_pose_collection.add(_poses[i][j]->unique_id(),_poses[i][j]->value());
    }
  }
  for(int i =0; i< _features.size(); i++){
    feature_collection.add(_features[i]->unique_id(),
			   Pose3d(_features[i]->value(),Rot3d(0,0,0)));
  }
  _viewer->sendCollection(own_pose_collection,true);
  _viewer->sendCollection(other_pose_collection,true);
  _viewer->sendCollection(feature_collection,true);
  _viewer->sendCollection(*_range_collection,true);
  _viewer->sendCollection(*_feature_collection,true);
  _viewer->sendCollection(*_virtual_collection,true);

}

void FactorGraph::print(){
  _slam.print_graph();
}
