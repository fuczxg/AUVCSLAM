/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimCompass.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef SimCompass_HEADER
#define SimCompass_HEADER

#include "random"
#include "MOOS/libMOOS/MOOSLib.h"
#include "Eigen/LU"


using namespace std;
using namespace Eigen;

class SimCompass : public CMOOSApp
{
public:
SimCompass();
~SimCompass();

protected:
bool OnNewMail(MOOSMSG_LIST &NewMail);
bool Iterate();
bool OnConnectToServer();
bool OnStartUp();
void RegisterVariables();

private: // Configuration variables
Matrix3d compass_covariance;

private: // State variables
 Vector3d sim_compass; // roll pitch yaw
 default_random_engine generator;
 normal_distribution<double> distribution[3];
 bool add_noise;
 string in_prefix;

};

#endif 
