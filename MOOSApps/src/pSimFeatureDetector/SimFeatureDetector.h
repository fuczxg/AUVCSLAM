/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimFeatureDetector.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimFeatureDetector_HEADER
#define SimFeatureDetector_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <Eigen/LU>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/isam_coop.hpp>
#include <string>

using namespace std;
using namespace Eigen;

class SimFeatureDetector : public AppCastingMOOSApp
{
 public:
   SimFeatureDetector();
   ~SimFeatureDetector();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool buildReport();


 private:
   void handle_detection_report(string,coop::sidescan_detection_t*);

 private: // Configuration variables
   double swath_width_desired; // how big do we want the sensor swath
   double pd_desired; // can be used to test reliability with which features are detected
   Matrix3d sidescan_detection_covariance;
   int _id; // 
   string vname; // 

 private: // State variables
   bool sensor_config_requested;
   bool sensor_config_set;
   Vector3d sim_pose; // the actual pose of the vehicle
   
   int64_t m_current_time;
   int64_t m_start_time;

   lcm::LCM* _lcm;
   string provider;
   string in_prefix;
   bool deployed;
   bool start_on_deploy;
};

#endif 
