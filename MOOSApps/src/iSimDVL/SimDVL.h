/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimDVL.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimDVL_HEADER
#define SimDVL_HEADER

#include "random"
#include "MOOS/libMOOS/MOOSLib.h"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;

class SimDVL : public CMOOSApp
{
 public:
   SimDVL();
   ~SimDVL();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables
   Matrix2d DVL_covariance;

 private: // State variables
   Vector2d sim_dvl;
   default_random_engine generator;
   normal_distribution<double> distribution[2];

   bool add_noise;
   string in_prefix;
};

#endif 
