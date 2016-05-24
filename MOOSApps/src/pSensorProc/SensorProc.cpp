/************************************************************/
/*    NAME: Liam Paull                                              */
/*    ORGN: MIT                                             */
/*    FILE: SensorProc.cpp                                        */
/*    DATE: Aug 11, 2014                                                */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "SensorProc.h"
#include "moos_utils.h"
#include "isam_coop/conversions.h"

//---------------------------------------------------------
// Constructor

SensorProc::SensorProc(){

  _t_new = 0;
  _t_old = 0;
  initialized = false;
  provider = "udpm://239.255.76.56:7667";
  prefix = "NAV";
  m_runtype = "simulation";
  m_downsample_rate = 1;
  m_background_mode = false;

  dvl_x = false;
  dvl_y = false;
  gps_x = false;
  gps_y = false;
  compass_yaw = false;
  gps_yaw_in = false;
  gps_speed_in = false;

  deployed = false;
  start_on_deploy = false;
  add_noise = true;
  m_propagate_count = 0;

  pthread_mutex_init(&pose_update_mutex, NULL);
}

//---------------------------------------------------------
// Destructor

SensorProc::~SensorProc()
{
  delete _lcm;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool SensorProc::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  // TODO should have some way of determining if data is stale and throwing a warning

  // since we are running with kayaks, just set the depth, roll and pitch to 0
  if(m_runtype == "hover"){
    depth.mu = 0.0;
    compass.mu[1] = 0.0; 
    compass.mu[2] = 0.0;
  }

  MOOSMSG_LIST::iterator p;
   
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    double dval = msg.GetDouble();
    string sval = msg.GetString();

    if(m_runtype == "hover"){
      if(MOOSStrCmp(key,"RTK_Y")){ // very accurate position
	gps.mu[0] = dval;
	gps_x = true;
      }
      if(MOOSStrCmp(key,"RTK_X")){
	gps.mu[1] = dval;
	gps_y = true;
      }
      if(MOOSStrCmp(key,"RTK_SPEED")){ // speed as determined by GPS (not RTK .. seems to be slightly too fast)
	gps_speed = dval;
	gps_speed_in = true;
      }
      if(MOOSStrCmp(key,"COMPASS_HEADING_FILTERED")){ // direction that vessel is pointing
 	compass.mu[0] = dval*PI/180;
	compass_yaw = true;
      }
      if(MOOSStrCmp(key,"RTK_HEADING")){ // direction of motion
	gps_yaw = dval*PI/180;
	gps_yaw_in = true;
      }

    }
    if(m_runtype == "simulation"){
      if(MOOSStrCmp(key,"DVL_X")){
	dvl.mu[0] = dval;
	dvl_x = true;
      }
      if(MOOSStrCmp(key,"DVL_Y")){
	dvl.mu[1] = dval;
	dvl_y = true;
      }
      if(MOOSStrCmp(key,"GPS_X")){
	gps.mu[0] = dval;
	gps_x = true;
      }
      if(MOOSStrCmp(key,"GPS_Y")){
	gps.mu[1] = dval;
	gps_y = true;
      }
      if(MOOSStrCmp(key,"COMPASS_YAW")){
	compass.mu[0] = dval;
	compass_yaw = true;
      }
      if(MOOSStrCmp(key,"COMPASS_PITCH")){
	compass.mu[1] = dval;
      }
      if(MOOSStrCmp(key,"COMPASS_ROLL")){
	compass.mu[2] = dval;
      }
      if(MOOSStrCmp(key,"DEPTH_DEPTH")){
	depth.mu = dval;
      }
    }
    if(MOOSStrCmp(key,"DEPLOY"))
      deployed = (tolower(sval) == "true");
    
    if(MOOSStrCmp(key,"ADD_ODOM")){ // TODO should be asynchronous mail handling.
      accumulate_odom();
      send_odom();
      m_propagate_count = 0;
    }
    if(MOOSStrCmp(key,"GENERATE_PACKET")){ // add new pose on back end and then request packet generation
      reportEvent("generating packet request received" );
      accumulate_odom();
      send_odom();
      m_propagate_count = 0;
      coop::command_t c;
      c.command = coop::command_list_t::GENERATE_PACKET;
      _lcm->publish("COMMAND",&c);
    }
  }
  return(true);
}


void SensorProc::accumulate_odom(){

  double yaw = compass.mu[0];
  double Syawyaw = 1/compass.omega[0][0];
  Matrix2d DVL_covariance;
  DVL_covariance << 
    dvl.sigma[0][0], 
    dvl.sigma[0][1], 
    dvl.sigma[1][0], 
    dvl.sigma[1][1];
  int64_t t_current = time_in_ms();
  double dt = double(t_current - _t_inter)/1000; // ms -> s
  // -------------------
  // For calculation of XY odom covariance, see Eustice_JFR_2011
  // Possible avenue for future work: update the angle error model (use exp map or VMF dist)
  double sy = sin(yaw);
  double cy = cos(yaw);
  double dx = dvl.mu[0];
  double dy = dvl.mu[1];
  Vector2d dvl_mu(dvl.mu[0], dvl.mu[1]);
  Matrix2d R;
  R << cy, -sy, sy, cy;
  Matrix2d Rp;
  Rp << -sy, -cy, cy, -sy; 
  Matrix2d U;
  U << dx*dx, dx*dy, dx*dy, dy*dy;
  Vector2d xy_odom_mu = R*dt*dvl_mu;


  Matrix2d S1 = dt*dt*R*DVL_covariance*R.transpose();
  Matrix2d S2 = dt*dt*Rp*U*Rp.transpose()*Syawyaw;
  Matrix2d S3 = dt*dt*Rp*DVL_covariance*Rp.transpose()*Syawyaw;
  Matrix2d xy_odom_covariance = S1+S2+S3;
  Matrix2d accumulated_xy_inf;
  Vector2d accumulated_xy_pos;
  conversions::lcm2eigen(accumulated_xy_odom.pos2,
			 accumulated_xy_pos,
			 accumulated_xy_inf);

  Matrix2d accumulated_xy_cov;
  if (accumulated_xy_pos(0) == 0 && accumulated_xy_pos(1) == 0){
    accumulated_xy_cov = xy_odom_covariance;
  }
  else{
    // this will crash if accumulated_xy_inf is all zeros which is what it is if just reset.
    accumulated_xy_cov = accumulated_xy_inf.inverse() + xy_odom_covariance;
  }
  accumulated_xy_inf = accumulated_xy_cov.inverse();
 
  accumulated_xy_pos += xy_odom_mu;
  coop::pos2_t pos;
  conversions::eigen2lcm(accumulated_xy_pos, 
			 accumulated_xy_inf, 
			 pos);
  accumulated_xy_odom.pos2 = pos;

  _t_inter = t_current;

}

coop::pose3_t SensorProc::propagate_current_pose()
{
  coop::pose3_t propagated_pose;
  // propagated_pose = current_pose + accumulated_odom (NB accumulated_odom is already in global frame)
  propagated_pose = current_pose;
  for(int i = 0; i<2 ; i++)
    propagated_pose.mu[i] += accumulated_xy_odom.pos2.mu[i];
  propagated_pose.mu[3] = compass.mu[0];
  return propagated_pose;
}
void SensorProc::send_odom(){

  if (accumulated_xy_odom.pos2.mu[0] == 0 && accumulated_xy_odom.pos2.mu[1] == 0)
    return;


  _t_new = time_in_ms();

  coop::odom_xy_global_yprz_t odom_yprz;
  odom_yprz.t[0] = _t_old;
  odom_yprz.t[1] = _t_new;
  odom_yprz.id = _id;

  odom_yprz.odom_xy     = accumulated_xy_odom;
  odom_yprz.attitude    = compass;
  odom_yprz.depth       = depth;

  _lcm->publish("ODOM_XY_GLOBAL_YPRZ",&odom_yprz);

  Notify("ACCUMULATED_XY_ODOM", "X="+doubleToString(accumulated_xy_odom.pos2.mu[0])+",Y="+doubleToString(accumulated_xy_odom.pos2.mu[1]));

  _t_old = _t_new;

  // send an update command. This might look a little overly complicated
  // but I think it will work better in the long run to have a clean
  // interface for sending commands to the the isam_coop
  coop::command_t c;
  c.command = coop::command_list_t::UPDATE;
  _lcm->publish("COMMAND",&c);

  reset_odom();
}

void SensorProc::reset_odom(){
  for (int i = 0; i<2; i++){
    accumulated_xy_odom.pos2.mu[i]=0;
    for (int j = 0; j<2; j++)
      accumulated_xy_odom.pos2.omega[i][j]=0;
  }
  _t_inter = _t_old;
}
     

void SensorProc::send_global(){
  coop::global_t global;
  global.t = _t_new;
  global.id = _id;
  global.pos      = gps;
  global.depth    = depth;
  global.attitude = compass;

  _lcm->publish("GLOBAL",&global);

}


//---------------------------------------------------------
// Procedure: OnConnectToServer

bool SensorProc::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SensorProc::Iterate()
{
  AppCastingMOOSApp::Iterate();
  if(start_on_deploy && !deployed)
    return(true);

  // ------------------------------
  // the following block is specific to operating with the hovergroup kayaks
  // the DVL is faked from the GPS and compass.

  if (m_runtype == "hover" && compass_yaw && gps_yaw_in && gps_speed_in){
    double angle_diff = gps_yaw - compass.mu[0]; // in RADs // this is how much crabbing
    // what about angle wraps?
    if (angle_diff > PI)
      angle_diff = 2*PI - angle_diff;
    if (angle_diff < -PI)
      angle_diff = 2*PI + angle_diff;

    if (gps_speed < 0.3){ // use raw compass at low speeds (GPS heading unreliable)
      dvl.mu[0] = gps_speed;
      dvl.mu[1] = 0.0;
    }
    else{
      dvl.mu[0] = gps_speed*cos(angle_diff);
      dvl.mu[1] = gps_speed*sin(angle_diff);
    }

    if (add_noise){
      dvl.mu[0] += distribution[0](generator);
      dvl.mu[1] += distribution[1](generator);
    }

    dvl_x = true;
    dvl_y = true;
  }

  

  if (dvl_x && dvl_y && initialized){
    accumulate_odom();
    pthread_mutex_lock(&pose_update_mutex);
    publish_pose(propagate_current_pose()); // updates on front end
    pthread_mutex_unlock(&pose_update_mutex);
  }

  if(m_propagate_count == m_downsample_rate){
    send_odom(); // triggers a new back end pose
    m_propagate_count = 0;
  }
  else
    m_propagate_count++;

  if (gps_x && gps_y && compass_yaw && !initialized){ // got gps // using initialized to only send first gps reading
    reportEvent("Sending initialization from GPS");
    _t_new = time_in_ms();
    send_global();
    _t_old = _t_new;
    reset_odom();
    initialized = true;
  }


  Notify(prefix+"_SPEED",Vector2d(dvl.mu[0],dvl.mu[1]).norm()); // this is basically just for viewer purpose


  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SensorProc::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  read_mission_file();

  // initialize the lcm channel
  reportEvent("LCM URL: " + provider);
  _lcm = new lcm::LCM(provider.c_str());

  lcm_subscribe();
   // spawn the lcm_subscribe thread
   pthread_t lcm_subscribe_thread;
   if(pthread_create(&lcm_subscribe_thread,NULL,SensorProc::lcm_handle,this)){
     cout << "Error creating LCM subscribe thread" << endl;
   }
   // not joining because this thread only executes a while loop that never finishes and I need the MOOS thread to keep going. Is this horrible?	
   

   MOOS::Pause(1000);
   RegisterVariables();	
   return(true);
}

void SensorProc::read_mission_file(){
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration("pSensorProc", sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++){
      string original = *p;
      string param = stripBlankEnds(toupper(biteString(*p,'=')));
      string value = stripBlankEnds(*p);
      
      if(param == "VEHICLE_ID")
	_id = atoi(value.c_str());
      else if(param == "DVL_COVARIANCE"){
	Matrix2d DVL_covariance = read_matrix(value);
	Matrix2d DVL_information = DVL_covariance.inverse();
	for (int i=0; i<2; i++){
	  for (int j=0; j<2; j++){
	    dvl.omega[i][j] = DVL_information(i,j);
	    dvl.sigma[i][j] = DVL_covariance(i,j);
	  }
	}
      }
      else if(param == "GPS_COVARIANCE"){
	Matrix2d GPS_covariance = read_matrix(value);
	Matrix2d GPS_information = GPS_covariance.inverse();
	for (int i=0; i<2; i++)
	  for (int j=0; j<2; j++)
	    gps.omega[i][j] = GPS_information(i,j);
      }
      else if(param == "COMPASS_COVARIANCE"){
	Matrix3d compass_covariance = read_matrix(value);
	Matrix3d compass_information = compass_covariance.inverse();
	for (int i=0; i<3; i++)
	  for (int j=0; j<3; j++)
	    compass.omega[i][j] = compass_information(i,j);
      }
      else if(param == "DEPTH_COVARIANCE"){
	double depth_covariance = atof(value.c_str());
	depth.omega = 1/depth_covariance;
      }
      else if(param == "LCM_PROVIDER")
	provider = "udpm://239.255.76.56:" + value;
      else if(param == "PREFIX")
	prefix = value;
      else if(param == "RUNTYPE")
	m_runtype = value;
      else if(param == "START_ON_DEPLOY")
	start_on_deploy = (tolower(value) == "true");
      else if(param == "DOWNSAMPLE_RATE")
	m_downsample_rate = atoi(value.c_str());
      else if(param == "BACKGROUND_MODE")
	m_background_mode = (tolower(value) == "true");
      else
	cout << "unhandled param: " << param << endl;
    }
  }

  distribution[0] = normal_distribution<double>(0.0,sqrt(dvl.sigma[0][0]));
  distribution[1] = normal_distribution<double>(0.0,sqrt(dvl.sigma[1][1]));


  if(m_background_mode)
    prefix = "ISAM_COOP";

}


//---------------------------------------------------------
// Procedure: RegisterVariables

void SensorProc::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DVL_*", "*", 0.0); // Everything about the DVL
  Register("COMPASS_*", "*", 0.0); // Compass
  if(m_runtype == "simulation")
    Register("GPS_*","*",0.0); // GPS
  if(m_runtype == "hover"){
    Register("RTK_X","*",0.0);
    Register("RTK_Y","*",0.0);
    Register("RTK_SPEED","*",0.0);
    Register("RTK_HEADING","*",0.0);
  }
  Register("DEPTH_DEPTH","*",0.0); //Depth
  Register("DEPLOY","*",0.0);
  Register("ADD_ODOM", "*",0.0);
  Register("GENERATE_PACKET", "*",0.0);
}

void * SensorProc::lcm_handle(void* data){
  while(0==((SensorProc *) data)->_lcm->handle());
}

void SensorProc::lcm_subscribe(){
  _lcm->subscribe(string("CURRENT_POSE"),&SensorProc::on_pose,this);
  _lcm->subscribe(string("TRAJECTORY"),&SensorProc::on_trajectory,this);
  _lcm->subscribe(string("COMMAND"),&SensorProc::on_command,this);
}

void SensorProc::on_pose(const lcm::ReceiveBuffer* rbuf,
		       const string& channel,
		       const coop::pose3_t* msg){

  if (msg->id == _id){
    pthread_mutex_lock(&pose_update_mutex);
    for (int i=0; i<6; i++){
      current_pose.mu[i] = msg->mu[i];
      for(int j=0; j<6; j++){
	current_pose.sigma[i][j]=msg->sigma[i][j];
      }
    }
    publish_pose(current_pose);
    Notify("ISAM_COOP_ALIVE","true");
    pthread_mutex_unlock(&pose_update_mutex);
  }
}

void SensorProc::publish_pose(coop::pose3_t pose){
    double heading = pose.mu[3]*180/PI;
    if (heading > 360.0)
      heading -= 360.0;
    if (heading < 0.0)
      heading += 360.0;
    Notify(prefix + "_X",pose.mu[1]); // conversion to MOOS coords
    Notify(prefix + "_Y",pose.mu[0]); 
    Notify(prefix + "_DEPTH",pose.mu[2]); 
    Notify(prefix + "_PITCH",pose.mu[4]); 
    Notify(prefix + "_ROLL",pose.mu[5]); 
    Notify(prefix + "_HEADING",heading);
    Notify("SIGMA_XX",pose.sigma[1][1]);
    Notify("SIGMA_YY",pose.sigma[0][0]);
    Notify("SIGMA_ZZ",pose.sigma[2][2]);
    Notify("SIGMA_PSIPSI",pose.sigma[3][3]);
}

void SensorProc::on_trajectory(const lcm::ReceiveBuffer* rbuf,
			     const string& channel,
			     const coop::trajectory_t* msg){

  //TODO

}

void SensorProc::on_command(const lcm::ReceiveBuffer* rbuf,
			    const string& channel,
			    const coop::command_t* msg){
  if(msg->command == coop::command_list_t::REQUEST_PRIOR)
    initialized = false;

}

bool SensorProc::buildReport(){

  if (initialized)
    m_msgs << "GPS fix obtained ... we are good to go" << endl;
  else{
    m_msgs << "Waiting for gps fix..." << endl;
    return(true);
  }
  
  m_msgs << "---------------------------"  << endl;
  m_msgs << "Current system time: " << _t_new  << endl;
  m_msgs << "---------------------------"  << endl;

  m_msgs << "Current pose (X,Y,DEPTH,YAW,PITCH,ROLL) {XX,YY,ZZ,PSIPSI} : " << endl;
  m_msgs << "( " << current_pose.mu[1];
  m_msgs << ", " << current_pose.mu[0];
  m_msgs << ", " << current_pose.mu[2];
  m_msgs << ", " << current_pose.mu[3];
  m_msgs << ", " << current_pose.mu[4];
  m_msgs << ", " << current_pose.mu[5] << ")" << endl;
  m_msgs << "{ " << current_pose.sigma[1][1];
  m_msgs << ", " << current_pose.sigma[0][0];
  m_msgs << ", " << current_pose.sigma[2][2];
  m_msgs << ", " << current_pose.sigma[3][3] << "}" << endl;

  m_msgs << "Accumulated odom: (X,Y)  {XX, YY} : " << endl;
  m_msgs << "( " << accumulated_xy_odom.pos2.mu[0] <<
    ", " << accumulated_xy_odom.pos2.mu[1] <<
    ")"  << endl 
	 << "{ " << accumulated_xy_odom.pos2.omega[0][0] <<
    ", " << accumulated_xy_odom.pos2.omega[1][1] << " } " << endl;


  m_msgs << "Latest sensor data: " <<endl;
  m_msgs << "Compass: " << compass.mu[0] << ", "<<  
    compass.mu[1] << ", " << 
    compass.mu[2] << endl;
  m_msgs << "dvl    : " << dvl.mu[0] << ", "     << dvl.mu[1]     <<endl;
  m_msgs << "gps    : " << gps.mu[0] << ", "    << gps.mu[1]     <<endl;
  m_msgs << "depth  : " << depth.mu   << endl;

  

  return(true);
}
