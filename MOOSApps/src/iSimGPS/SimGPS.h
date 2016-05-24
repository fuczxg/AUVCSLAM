/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimGPS.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimGPS_HEADER
#define SimGPS_HEADER

#include "random"
#include "MOOS/libMOOS/MOOSLib.h"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;

class SimGPS : public CMOOSApp
{
 public:
   SimGPS();
   ~SimGPS();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables
   Matrix2d GPS_covariance;
   double max_depth;

 private: // State variables
   Vector2d sim_gps;
   default_random_engine generator;
   normal_distribution<double>distribution;
   double sim_depth;
   bool add_noise;
   string in_prefix;
};

#endif 
