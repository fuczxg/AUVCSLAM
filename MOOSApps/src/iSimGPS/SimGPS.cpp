/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimGPS.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SimGPS.h"
#include <moos_utils.h>

//---------------------------------------------------------
// Constructor

SimGPS::SimGPS()
{
  sim_gps << 0.0,0.0;
  add_noise = true;
}

//---------------------------------------------------------
// Destructor

SimGPS::~SimGPS()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimGPS::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key==in_prefix + "_X")
      sim_gps(0) = dval;
    if(key==in_prefix + "_Y")
      sim_gps(1) = dval;
    if(key==in_prefix + "_DEPTH")
      sim_depth = dval;
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimGPS::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimGPS::Iterate()
{

  if (sim_depth < max_depth){
    if (add_noise){
      Notify("GPS_X",sim_gps(1) + distribution(generator)); // conversion from MOOS coords to NED embedded here.
      Notify("GPS_Y",sim_gps(0) + distribution(generator));
    }
    else{
      Notify("GPS_X",sim_gps(1)); // conversion from MOOS coords to NED embedded here.
      Notify("GPS_Y",sim_gps(0));
    }
      
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimGPS::OnStartUp()
{
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "GPS_COVARIANCE") {
        GPS_covariance = read_matrix(value);
      }
      if(param == "MAX_DEPTH"){
	max_depth = atof(value.c_str());
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
  
  distribution = normal_distribution<double>(0.0,sqrt(GPS_covariance(0,0)));

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SimGPS::RegisterVariables()
{
  Register(in_prefix + "_*","*", 0);
}

