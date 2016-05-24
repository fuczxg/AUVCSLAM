/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: AcommsScheduler.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef AcommsScheduler_HEADER
#define AcommsScheduler_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"


class AcommsScheduler : public AppCastingMOOSApp
{
 public:
   AcommsScheduler();
   ~AcommsScheduler();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   bool buildReport();

 private: // Configuration variables
   int    num_auvs;
   double slot_time;

 private: // State variables
   double last_transmit_time;
   int    transmit_id;
   bool   deployed;
   bool   start_on_deploy;
};


#endif 
