#pragma once

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/isam_coop.hpp>
//#include <visualization/viewer.hpp>
#include <FactorGraph.h>
#include <PacketManager.h>
#include <string>


using namespace std;

class CoopIsam{
 public:
  CoopIsam(int, int, string);
  ~CoopIsam();

 public:
  void run(); // this is how we get things going.

 private:
  void lcm_subscribe();
  void lcm_publish_pose();
  void lcm_publish_trajectory();
  void request_prior();
  //lcm callbacks:
  void on_odom_xy_global_yprz(const lcm::ReceiveBuffer* rbuf, 
			      const string& channel, 
			      const coop::odom_xy_global_yprz_t* msg);

  void on_sidescan_detections(const lcm::ReceiveBuffer* rbuf, 
			      const string& channel, 
			      const coop::sidescan_detections_t* msg);


  void on_range(const lcm::ReceiveBuffer* rbuf, 
		const string& channel, 
		const coop::relative_range_t* msg);

  void process_range(const coop::relative_range_t* msg);

  void on_global(const lcm::ReceiveBuffer* rbuf, 
		 const string& channel, 
		 const coop::global_t* msg);

  void process_global(const coop::global_t* msg);

  void on_command(const lcm::ReceiveBuffer* buf,
		  const string& channel,
		  const coop::command_t* msg);

  void on_acomms_in(const lcm::ReceiveBuffer* rbuf,
		    const string& channel,
		    const coop::acomms_packet_t* msg);



 private:
  lcm::LCM* _lcm;
  FactorGraph* _coop_fg;
  PacketManager* _packet_manager;

  int _id;
  int _N;
};
