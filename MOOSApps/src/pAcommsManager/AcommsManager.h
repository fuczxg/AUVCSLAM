/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: AcommsManager.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef AcommsManager_HEADER
#define AcommsManager_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h" 
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/isam_coop.hpp>
#include "protobufs/packet.pb.h"
#include "protobufs/factors.pb.h"
#include <string>
#include <Eigen/LU>
#include "dccl.h"
#include "HoverAcomms.h"

class AcommsManager : public AppCastingMOOSApp
{
 public:
   AcommsManager();
   ~AcommsManager();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool buildReport();
   void read_mission_file();

 private:
   void handle_acomms_receive(std::string);

 private: 
   void lcm_subscribe();
   static void * lcm_handle(void* data);
   std::string provider;
   double packet_type;
  
 private:
   void on_acomms_packet(const lcm::ReceiveBuffer* rbuf,
			       const std::string& channel,
			       const coop::acomms_packet_t* msg);


 private: // Configuration variables
   int id;
   int N;
   coop::global_t global;
   coop::virtual_pose3_pose3_t pose_pose;
   double range_covariance;
   bool deployed;
   bool start_on_deploy;
   bool background_mode;
   double m_sound_speed;

   std::string m_runtype;

   dccl::Codec codec;

 private: // State variables

  int incoming_id;
  int64_t incoming_time;
  int64_t m_start_time;
  int64_t m_current_time;
  bool m_acomms_received;
  HoverAcomms::DriverStatus m_driver_status;


 private:
   lcm::LCM* _lcm;

   int incoming_count;
   int outgoing_count;

};

#endif 
