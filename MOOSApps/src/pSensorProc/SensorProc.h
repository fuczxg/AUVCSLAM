/************************************************************/
/*    NAME: Liam Paull                                              */
/*    ORGN: MIT                                             */
/*    FILE: SensorProc.h                                          */
/*    DATE: Aug 11, 2014                                                */
/************************************************************/

#ifndef SensorProc_HEADER
#define SensorProc_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/isam_coop.hpp>
#include <string>
#include <Eigen/LU>
#include "random"

using namespace std;
using namespace Eigen;

class SensorProc : public AppCastingMOOSApp {
 public:
   SensorProc();
   ~SensorProc();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   void read_mission_file();
   bool buildReport();

 private: // lcm setup
   void lcm_subscribe();
   static void * lcm_handle(void* data);
   string provider;

 private: //lcm callbacks:
   void on_pose(const lcm::ReceiveBuffer* rbuf,
		const string& channel,
		const coop::pose3_t* msg);
   void on_trajectory(const lcm::ReceiveBuffer* rbuf,
		      const string& channel,
		      const coop::trajectory_t* msg);
   void on_command(const lcm::ReceiveBuffer* rbuf,
		   const string& channel,
		   const coop::command_t* msg);


 private: // publish to lcm functions
   void send_global();
   void send_odom();
   void accumulate_odom();
   void reset_odom();

   coop::pose3_t propagate_current_pose();
   void          publish_pose(coop::pose3_t);

 private: // config vars

   int _id;
   string prefix;
   string m_runtype;
   int m_downsample_rate;
   int m_propagate_count;

 private:
   lcm::LCM* _lcm;
   int64_t _t_new;
   int64_t _t_old;
   int64_t _t_inter;
   bool initialized;

   //sensor data and covariances
   coop::rot3_t    compass;
   coop::pos2_t    dvl;
   coop::pos2_t    gps;
   coop::depth_t   depth;


   double gps_yaw;
   double gps_speed;

   bool dvl_x;
   bool dvl_y;
   bool gps_x;
   bool gps_y;
   bool compass_yaw;
   bool gps_yaw_in;
   bool gps_speed_in;

   bool deployed;
   bool start_on_deploy;
   bool m_background_mode; // in background mode we do not post to NAV
   
   bool add_noise;
   default_random_engine generator;
   normal_distribution<double> distribution[2];

   coop::pose3_t    current_pose;
   coop::odom_xy_t  accumulated_xy_odom;

 private:
   pthread_mutex_t  pose_update_mutex;
};

#endif 
