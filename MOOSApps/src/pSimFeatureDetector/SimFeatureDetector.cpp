/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: SimFeatureDetector.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SimFeatureDetector.h"
#include <moos_utils.h>

using namespace std;

//---------------------------------------------------------
// Constructor

SimFeatureDetector::SimFeatureDetector()
{
  sensor_config_requested = false;
  sensor_config_set = false;
  provider="udpm://239.255.76.56:7667";
  in_prefix = "SIM";
  deployed = false;
  start_on_deploy = false;
}

//---------------------------------------------------------
// Destructor

SimFeatureDetector::~SimFeatureDetector()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SimFeatureDetector::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);


  vector<string> detection_reports;

  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if(key == "UHZ_CONFIG_ACK")
      sensor_config_set=true; // don't really care the actual parameters we got...
    if(key == "UHZ_DETECTION_REPORT")
      detection_reports.push_back(sval); // push all the strings into a vec and process below
    if(key == in_prefix + "_X")
      sim_pose(1)=dval;
    if(key == in_prefix + "_Y")
      sim_pose(0)=dval;
    if(key == in_prefix + "_HEADING")
      sim_pose(2)=dval*PI/180;
    if(key == "DEPLOY")
      deployed = (tolower(sval) == "true");
  }

  m_current_time = time_in_ms();
  coop::sidescan_detections_t s;
  s.n = detection_reports.size();
  s.detections.resize(s.n);
  for(int i=0;i<s.n;i++){
    coop::sidescan_detection_t detection;
    handle_detection_report(detection_reports[i],&detection);
    s.detections[i]=detection;
  }
  if(s.n > 0){
    _lcm->publish("SIDESCAN_DETECTIONS",&s);
    Notify("ADD_ODOM",m_current_time); // force a node to get added at this instant (since we might be heavily downsampling node addition)
  }
  return(true);
}

void SimFeatureDetector::handle_detection_report(string report, 
						 coop::sidescan_detection_t* detection)
{

  detection->t = m_current_time;
  detection->id = _id;

  //Step 1: get the hazard point out of the report string:
  Vector2d haz_point;
  int feature_id;
  vector<string> svec = parseString(report,',');
  unsigned int i, vsize = svec.size();
  for(i=0;i<vsize;i++){
    string param = biteStringX(svec[i], '=');                                  
    string value = svec[i];
    if(param=="x")
      haz_point(1)=atof(value.c_str());
    if(param=="y")
      haz_point(0)=atof(value.c_str());
    if(param=="label")
      feature_id = atoi(value.c_str());
  }
  //Step 2: Convert to local coords:
  Vector2d sim_point(sim_pose(0),sim_pose(1));
  Matrix2d rot2d;
  double ct = cos(-sim_pose(2));
  double st = sin(-sim_pose(2));
  rot2d << ct, -st, st, ct;
  Vector2d haz_point_local = rot2d*(haz_point-sim_point);
  //Step 3: Calculate the measurement:
  detection->mu=haz_point_local(1); // the measurement is the "y" value in local coords

  //Step 4: information matrix 
  Matrix3d sidescan_detection_information = 
    sidescan_detection_covariance.inverse();

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      detection->omega[i][j] = sidescan_detection_information(i,j);

  detection->feature_id = feature_id;
}


//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SimFeatureDetector::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SimFeatureDetector::Iterate()
{
  AppCastingMOOSApp::Iterate();


  if(!sensor_config_requested){
    string request = "vname=" + vname ;
    request += ",width=" + doubleToStringX(swath_width_desired,2);
    request += ",pd="    + doubleToStringX(pd_desired,2);
    sensor_config_requested = true;
    Notify("UHZ_SENSOR_REQUEST",request);
  }

  if (start_on_deploy && !deployed)
    return(true);

  if(sensor_config_set){
    string request = "vname=" + vname;
    Notify("UHZ_SENSOR_REQUEST",request);
  }

  AppCastingMOOSApp::PostReport();

  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SimFeatureDetector::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration("pSimFeatureDetector", sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "SIDESCAN_DETECTION_COVARIANCE")
        sidescan_detection_covariance = read_matrix(value);
      else if(param == "SWATH_WIDTH")
        swath_width_desired=atof(value.c_str());
      else if(param == "PD")
	pd_desired = atof(value.c_str());
      else if(param == "VEHICLE_ID")
	_id = atoi(value.c_str());
      else if(param == "VEHICLE_NAME")
	vname = value;
      else if(param == "LCM_PROVIDER")
	provider = "udpm://239.255.76.56:" + value;
      else if(param == "BACKGROUND_MODE"){
	if (tolower(value) == "true")
	  in_prefix = "NAV";
	else
	  in_prefix = "SIM";
      }
      else if(param == "START_ON_DEPLOY")
	start_on_deploy = (tolower(value) == "true");
    }
  }
  
  _lcm = new lcm::LCM(provider);
  
  m_start_time = time_in_ms();

  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void SimFeatureDetector::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("UHZ_DETECTION_REPORT","*", 0);
  Register("UHZ_CONFIG_ACK","*", 0);
  Register(in_prefix + "_*","*",0);
  Register("DEPLOY","*",0);
}

bool SimFeatureDetector::buildReport(){
  m_msgs << "sidescan_detection covariance: " << endl;
  m_msgs << sidescan_detection_covariance << endl;
  m_msgs << "---------------------------------" << endl;

  return(true);

}
