/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimDepth.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimDepth_HEADER
#define SimDepth_HEADER

#include "random"
#include "MOOS/libMOOS/MOOSLib.h"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;

class SimDepth : public CMOOSApp
{
 public:
   SimDepth();
   ~SimDepth();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables
   double depth_covariance;

 private: // State variables
   double sim_depth;
   default_random_engine generator;
   normal_distribution<double> distribution;
   bool add_noise;
   string in_prefix;
};

#endif 
