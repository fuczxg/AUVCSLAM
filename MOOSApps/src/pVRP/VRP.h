/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: VRP.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef VRP_HEADER
#define VRP_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class VRP : public CMOOSApp
{
 public:
   VRP();
   ~VRP();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
   unsigned int m_iterations;
   double       m_timewarp;
};

#endif 
