/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimCompass.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SimCompass.h"
#include <moos_utils.h>

//---------------------------------------------------------
// Constructor

SimCompass::SimCompass()
{
  sim_compass << 0.0,0.0,0.0;
  add_noise = true;
  in_prefix = "SIM";
}

//---------------------------------------------------------
// Destructor

SimCompass::~SimCompass()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimCompass::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key == in_prefix + "_ROLL")
      sim_compass(0) = dval;
    if(key == in_prefix + "_PITCH")
      sim_compass(1) = dval;
    if(key == in_prefix + "_HEADING")
      sim_compass(2) = dval*PI/180;

   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimCompass::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimCompass::Iterate()
{

  if (add_noise){
    Notify("COMPASS_ROLL",sim_compass(0) + distribution[0](generator));
    Notify("COMPASS_PITCH",sim_compass(1) + distribution[1](generator));
    Notify("COMPASS_YAW",sim_compass(2) + distribution[2](generator));
  }
  else{
    Notify("COMPASS_ROLL",sim_compass(0));
    Notify("COMPASS_PITCH",sim_compass(1));
    Notify("COMPASS_YAW",sim_compass(2));
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimCompass::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "COMPASS_COVARIANCE") {
        compass_covariance = read_matrix(value);
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
  
  distribution[0] = normal_distribution<double>(0.0,sqrt(compass_covariance(0,0)));
  distribution[1] = normal_distribution<double>(0.0,sqrt(compass_covariance(1,1)));
  distribution[2] = normal_distribution<double>(0.0,sqrt(compass_covariance(2,2)));

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SimCompass::RegisterVariables()
{
  Register(in_prefix + "_*","*", 0);
}

