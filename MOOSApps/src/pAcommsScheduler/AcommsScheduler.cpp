/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: AcommsScheduler.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "AcommsScheduler.h"
#include "moos_utils.h"


using namespace std;

//---------------------------------------------------------
// Constructor

AcommsScheduler::AcommsScheduler()
{
  last_transmit_time = time_in_ms();
  transmit_id = 1;
  deployed = false;
  start_on_deploy = false;
}

//---------------------------------------------------------
// Destructor

AcommsScheduler::~AcommsScheduler()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool AcommsScheduler::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key == "DEPLOY"){
      deployed = (tolower(sval)=="true");
      last_transmit_time = time_in_ms();
    }
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool AcommsScheduler::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool AcommsScheduler::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if (start_on_deploy && !deployed)
    return(true);

  double current_time = time_in_ms();

  if(current_time > last_transmit_time + slot_time){ // initiate a transmission
    Notify("SCHEDULER_TRANSMIT", double(transmit_id));
    reportEvent("Sending initiation to vehicle" + intToString(transmit_id));
    last_transmit_time = current_time;
    transmit_id++;
    if(transmit_id > num_auvs)
      transmit_id -= num_auvs;
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AcommsScheduler::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "NUM_AUVS") 
        num_auvs = atoi(value.c_str());
      else if(param == "SLOT_TIME")
        slot_time = atof(value.c_str()) * 1000; // s -> ms
      else if(param == "START_ON_DEPLOY")
	start_on_deploy = (tolower(value) == "true");
    }
  }
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void AcommsScheduler::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DEPLOY","*",0);
}

bool AcommsScheduler::buildReport(){
  m_msgs << "Number of vehicles: " << num_auvs << endl;
  m_msgs << "Slot time:          " << slot_time << endl << endl;
  m_msgs << "last transmit time: " << last_transmit_time << endl;
  m_msgs << "last transmit id  : " << transmit_id << endl;
  return(true);
}
