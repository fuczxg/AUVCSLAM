#include <vector>
#include "isam_coop.h"
#include <lcmtypes/isam_coop.hpp>
#include "FactorGraph.h"
#include "Packet.h"

class PacketManager{
 public:
  PacketManager(int, int);
  ~PacketManager();

  void packet_received(int,bool);
  void initialize_CPs(int64_t);
  bool generate(FactorGraph*, coop::acomms_packet_t&);
  void add_global(coop::global_t new_global);
  void set_packet_type(Packet::PacketType p){_p=p;};

 private:
  void reset_global();
  FactorGraph* make_local_factor_graph(FactorGraph*, Pose3d, int64_t, int64_t);
  void add_pose_pose_to_packet(Eigen::MatrixXd&, coop::acomms_packet_t&, FactorGraph*, Pose3d, int64_t, int64_t);
  void add_pose_points_to_packet(Eigen::MatrixXd&, coop::acomms_packet_t&, FactorGraph*, Pose3d, int64_t);
  int64_t min_CP();    
  vector<int64_t> get_start_times();

  std::vector<bool> acks;
  std::vector<int64_t> outgoing_CPs;
  int N; // number of robots
  int id; // the id of the local robot
  int64_t last_trans_time;
  bool have_new_global;
  coop::global_t global;
  Packet::PacketType _p;

};
