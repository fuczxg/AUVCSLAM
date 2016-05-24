#include "PacketManager.h"
#include <fstream>
#include <vector>
#include "conversions.h"
#include "lcm_utils.h"


#define DEBUG 1

using namespace std;
using namespace coop;
using namespace Eigen;

PacketManager::PacketManager(int _N, int _id):N(_N),id(_id){
  for(int i = 0; i<=N; i++){
    acks.push_back(false);
    outgoing_CPs.push_back(0); // these need to be overwritten through initialization
  }
  have_new_global = false;
  _p = Packet::CL;
}

PacketManager::~PacketManager(){
}

// procedure: packet_received
// description: we have received a packet from "in_id" and we can set the ack bit and the received CP time
void PacketManager::packet_received(int in_id, bool in_ack){
  acks[in_id] = true; // we received successfully set out ack to go out on the next packet
  if (in_ack) outgoing_CPs[in_id] = last_trans_time; // we received a good ack, means they got our last transmission so update the outgoing CP to the last time we transmitted.
  reset_global();
}

void PacketManager::reset_global(){
  cout << "in reset global" << endl;
  // now do a check to see if the global that we have is older than any CP time (so we don't need to send it any more)
  bool reset_global=true;
  for(int i=1; i<=N; i++){
    if(outgoing_CPs[i] <= global.t && i!=id)
      reset_global=false;
  }
  if (reset_global){
    cout << "resetting global" << endl;
    have_new_global=false;
  }
}


void PacketManager::initialize_CPs(int64_t t){
  if (DEBUG) cout << "In packet manager initializing all contact point times to " << t << endl;
  for(int i=1; i<=N; i++){
    outgoing_CPs[i] = t;
  }
}

int64_t PacketManager::min_CP(){
  int64_t min = 100000000;
  for(int i=1; i<=N; i++){
    if(i==id) continue; // don't consider the CP for own robot
    cout << "outgoing CP" << i  << " = " << outgoing_CPs[i] << endl;
    if(outgoing_CPs[i] < min) min = outgoing_CPs[i];
  }
  cout << "min_outgoing cp: " << min << endl;
  cout << "global.t: " << global.t << endl;
  return min;
}


////////////////////////////////////////////////////////////////////////////////
// procedure: add_global
// add a global to the queue to be sent on the next trans. 
// it is actually sent when we call generate()
////////////////////////////////////////////////////////////////////////////////
void PacketManager::add_global(coop::global_t new_global){
  cout << "In packet manager adding global" << endl;
  have_new_global=true;
  global = new_global;
}

vector<int64_t> PacketManager::get_start_times(){
  vector<int64_t> start_times;
  if (_p == Packet::CL){
    for(int i=1; i<=N; i++){
      if(i==id) continue;
      start_times.push_back(outgoing_CPs[i]);
    }
  }
  else if(_p == Packet::CSLAM){   
    start_times.push_back(min_CP());
  }

  return start_times;
}

/////////////////////////////////////////////////////////////////////////////////
// procdure: generate
//  function used to generate the acomms packets from the 
// own vehicle factor graph.
// Steps:
// 1 - build a new "root shifted" graph out of the chunk that we want to send as 
//     determined by the minimum contact point
// 2 - marginalize and sparsify
// 3 - build packet and send.
////////////////////////////////////////////////////////////////////////////////
bool PacketManager::generate(FactorGraph* coop_fg, acomms_packet_t& outgoing_packet){

  int64_t end_time = coop_fg->get_current_time(id);
  if (end_time == 0) return false;

  outgoing_packet.num_poses = 0;
  outgoing_packet.num_landmarks = 0;

  vector<int64_t> start_times = get_start_times(); // this could depend on packet type
  for (int i = 0; i<start_times.size(); i++){
    int64_t start_time = start_times[i]; 
    Pose3d x1 = coop_fg->get_pose_estimate(start_time,id);
    cout << " before make local factor graph: " << time_in_ms() << endl;
    FactorGraph* local_fg = make_local_factor_graph(coop_fg, x1, start_time, end_time);
    if (local_fg == NULL) return false;
    cout << " before marginalize poses: " << time_in_ms() << endl;
    Eigen::MatrixXd X = local_fg->marginalize_poses_sparse(start_time, end_time);
    cout << " before add poses to packet: " << time_in_ms() << endl;
    add_pose_pose_to_packet(X, outgoing_packet, local_fg, x1, start_time, end_time);
    if(_p == Packet::CSLAM)
      add_pose_points_to_packet(X, outgoing_packet, local_fg, x1, end_time);
    delete local_fg;
  }

  // add ack_bits
  outgoing_packet.num_acks = N+1;
  outgoing_packet.acks.resize(N+1);
  for (int n=0; n<=N; n++){
    outgoing_packet.acks[n] = acks[n];
  }


  //  reset ack_bits
  for(int n=1; n<=N; n++)
    acks[n] = false;

  //  add global if there is one:
  outgoing_packet.has_global = have_new_global;
  if(have_new_global){
    outgoing_packet.global_pose = global;
  }

  outgoing_packet.num_ranges=0;
  outgoing_packet.ranges.resize(0);

  cout << "setting last_trans_time to" << end_time << endl;
  last_trans_time = end_time;
  
  return true;
}


FactorGraph* PacketManager::make_local_factor_graph(FactorGraph* coop_fg, Pose3d x1, int64_t start_time, int64_t end_time){

  cout << "Generating packet with start time " << start_time << " and end time " << end_time << endl;

  if (start_time == end_time){
    cout << "Rejecting packet generation with equal start and end times" << endl;
    return NULL;
  }


  FactorGraph* local_fg = new FactorGraph(1,1,true);

  // 1: start with the root and add prior at 0,0,0 (defines frame of ref)
  pose3_t p;
  for(int i = 0; i<6; i++){
    p.mu[i] = 0;
    for(int j= 0; j<6; j++){
      if (i==j)
	p.omega[i][j] = 100;
      else
	p.omega[i][j] = 0;
    }
  }
  p.id = 1;
  p.t  = start_time;
  local_fg->add_prior(p);


  int64_t current_time = start_time;
  do{
    if(_p == Packet::CSLAM){
      // add any feature detections from current time
      sidescan_detections_t ds;
      coop_fg->get_sidescan_detections_t(current_time,id,ds);
      for (int i = 0; i< ds.n; i++){
	sidescan_detection_t d = ds.detections[i];
	d.id = 1;
	local_fg->add_sidescan_detection(d);
      }
    }
    odom_xy_global_yprz_t odom = coop_fg->get_odom_xy_global_yprz(current_time,id);
    odom.id = 1;
    // transform globals into  local frame:
    odom.attitude = rotate_ypr(odom.attitude,x1.vector());
    odom.depth    = shift_z(odom.depth,x1.vector());
    odom.odom_xy  = rotate_odom(odom.odom_xy,x1.vector()); // NB: this odom is actually displacement in the world frame. therefore need to rotate it into the x1 frame.
    local_fg->add_odom_xy_global_yprz(odom);
    current_time = odom.t[1];
    // TODO if there is a global position factor should also add if it exists
  }
  while(current_time < end_time);

  local_fg->update();
  local_fg->update_viewer();
  return local_fg;
}
 
void PacketManager::add_pose_pose_to_packet(Eigen::MatrixXd &X, 
					    acomms_packet_t& outgoing_packet,
					    FactorGraph* local_fg,
					    Pose3d x1,
					    int64_t start_time,
					    int64_t end_time){
  coop::virtual_pose3_pose3_t v_pose;
  v_pose.t[0] = start_time;
  v_pose.t[1] = end_time;
  Pose3d x2_r = local_fg->get_pose_estimate(end_time,1);
  Pose3d x2_w = Pose3d(Point3d(x1.rot().wRo()*x2_r.trans().vector())
			       ,x2_r.rot()); // transmit relative in world frame. 
  Eigen::VectorXd mu = x2_w.vector();
  Eigen::MatrixXd omega(6,6);
  omega = (X.block(6,6,6,6)).inverse(); // TODO don't like the 6,6,6,6
  coop::pose3_t global_pose;
  conversions::eigen2lcm(mu, omega, global_pose);
  v_pose.pose = global_pose;
  outgoing_packet.num_poses = outgoing_packet.num_poses + 1;
  outgoing_packet.virtual_pose_poses.resize(outgoing_packet.num_poses);
  outgoing_packet.virtual_pose_poses[outgoing_packet.num_poses - 1] = v_pose;
}

void PacketManager::add_pose_points_to_packet(Eigen::MatrixXd &X,
					      acomms_packet_t& outgoing_packet,
					      FactorGraph* local_fg,
					      Pose3d x1,
					      int64_t start_time){

  int num_features = local_fg->num_features();
  cout << "Num features = " << num_features << endl;
  outgoing_packet.num_landmarks = outgoing_packet.num_landmarks + num_features;
  outgoing_packet.virtual_pose_points.resize(outgoing_packet.num_landmarks);

  list<int> ids = local_fg->get_feature_ids();
  list<int>::iterator it;
  int i = outgoing_packet.num_landmarks - num_features;
  int ic = 12;
  for(it = ids.begin(); it!= ids.end(); it++, ic+=3){ // TODO don't like the 3 being hcoded
    coop::point3_t local_point = local_fg->get_point3_t(*it);
    Point3d li(local_point.mu[0],
	       local_point.mu[1],
	       local_point.mu[2]);
    Eigen::VectorXd mu =  x1.rot().wRo() * li.vector(); // transmit in world frame
    Eigen::MatrixXd omega(3,3);
    omega = (X.block(ic,ic,3,3)).inverse();

    coop::point3_t global_point;
    conversions::eigen2lcm(mu,omega,global_point);
    coop::virtual_pose3_point3_t v_point;
    v_point.t = start_time;
    v_point.point = global_point;
    v_point.feature_id = *it;
    outgoing_packet.virtual_pose_points[i] = v_point;
    i++;
  }
}
