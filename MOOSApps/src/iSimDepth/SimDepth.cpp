/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimDepth.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SimDepth.h"

//---------------------------------------------------------
// Constructor

SimDepth::SimDepth()
{
  sim_depth = 0.0;
  add_noise = true;
}

//---------------------------------------------------------
// Destructor

SimDepth::~SimDepth()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimDepth::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    if (key == in_prefix + "_DEPTH")
      sim_depth = dval;
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimDepth::OnConnectToServer()
{
   // register for variables here
   // possibly look at the mission file?
   // m_MissionReader.GetConfigurationParam("Name", <string>);
   // m_Comms.Register("VARNAME", 0);
	
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimDepth::Iterate()
{
  if(add_noise)
    Notify("DEPTH_DEPTH",sim_depth + distribution(generator));
  else
    Notify("DEPTH_DEPTH",sim_depth);
    
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimDepth::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "DEPTH_COVARIANCE") {
        depth_covariance = atof(value.c_str());
      }
      else if (param == "ADD_NOISE") {
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
  
  distribution = normal_distribution<double>(0.0,sqrt(depth_covariance));

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SimDepth::RegisterVariables()
{
  Register(in_prefix + "_DEPTH","*", 0);
}

