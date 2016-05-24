/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimDVL.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SimDVL.h"
#include <moos_utils.h>


//---------------------------------------------------------
// Constructor

SimDVL::SimDVL()
{
  sim_dvl << 0.0,0.0;
  add_noise = true;
}

//---------------------------------------------------------
// Destructor

SimDVL::~SimDVL()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimDVL::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key == in_prefix + "_SPEED")
      sim_dvl(0) = dval;

   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimDVL::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimDVL::Iterate()
{
  if (add_noise){
    Notify("DVL_X",sim_dvl(0) + distribution[0](generator));
    Notify("DVL_Y",sim_dvl(1) + distribution[1](generator));
  }
  else {
    Notify("DVL_X",sim_dvl(0));
    Notify("DVL_Y",sim_dvl(1));
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimDVL::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "DVL_COVARIANCE") {
	DVL_covariance = read_matrix(value);
      }
      if(param == "ADD_NOISE"){
	add_noise = (tolower(value) == "true");
      }
      if(param == "BACKGROUND_MODE"){
	if (tolower(value) == "true")
	  in_prefix = "NAV";
	else
	  in_prefix = "SIM";
      }
    }
  }
  
  distribution[0] = normal_distribution<double>(0.0,sqrt(DVL_covariance(0,0)));
  distribution[1] = normal_distribution<double>(0.0,sqrt(DVL_covariance(1,1)));

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SimDVL::RegisterVariables()
{
  Register(in_prefix + "_*","*", 0);
}

