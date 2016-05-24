/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT                                             */
/*    FILE: AcommsManager.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "AcommsManager.h"
#include "protobuf_util.h"
#include <moos_utils.h>
#include <list>

using namespace std;

//---------------------------------------------------------
// Constructor

AcommsManager::AcommsManager()
{
  // all of these are defaults, should be overwritten by mission file
  id = 1;
  N = 1;
  provider="udpm://239.255.76.56:7667";
  packet_type = 0; // CL
  background_mode = false;
  m_runtype = "simulation";
  m_acomms_received = false;
  m_driver_status   = HoverAcomms::READY;
  incoming_count = 0;
  outgoing_count = 0;
  codec.load<packet_master>();
  for(int i = 0; i<6; i++){
    pose_pose.pose.mu[i] = 0;
    for(int j = 0; j<6; j++){
      pose_pose.pose.omega[i][j] = 0;
    }
  }
  
}

//---------------------------------------------------------
// Destructor

AcommsManager::~AcommsManager()
{
}


//#########################################################
// Acomms packet reception:
// 

//---------------------------------------------------------
// Procedure: OnNewMail

bool AcommsManager::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;

  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key   = msg.GetKey();
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

    if (key == "ACOMMS_RECEIVED"){
      incoming_count++;
      handle_acomms_receive(sval);
      reportEvent("finished received data");
    }
    
    if (key == "ACOMMS_DRIVER_STATUS"){ // TODO this should be async to get more accurate if nec.
      if (m_driver_status == HoverAcomms::READY && 
	  static_cast<HoverAcomms::DriverStatus>(int(dval)) == HoverAcomms::RECEIVING ){ // this is best guess of actual time that that ranging ping is received, the time when the driver flips from ready to receiving
	reportEvent("Modem driver starting receive");
	incoming_time = time_in_ms();
	Notify("ADD_ODOM", incoming_time); // sends request to pSensorProc to add a node at this time. Will be used once we actually get all the data to attach the relative range etc to.
      }
      m_driver_status = static_cast<HoverAcomms::DriverStatus>(int(dval));
    }

    else if(key == "SCHEDULER_TRANSMIT"){
      if (int(dval) == id){ // it is our turn
	double outgoing_time = time_in_ms();
	Notify("GENERATE_PACKET",outgoing_time); // send to pSensorProc so that we can add a pose at the most recent time (timing is important here since the relative ranges attach to these poses and the tof is precise)
      }
    }
    else if (key == "DEPLOY"){
      deployed = (tolower(sval)=="true");
    }
  }

  m_current_time = time_in_ms();
  return(true);
}

void AcommsManager::handle_acomms_receive(string s){
  HoverAcomms::AcommsReception reception;
  if (reception.parseFromString(s))
    reportEvent("Succesful reception"); 
  if(reception.getStatus() == HoverAcomms::GOOD){ // TODO handle partial packets
    packet_master packet;
    bool decode_okay = true;
    
    codec.decode(reception.getData(), &packet);
    reportEvent("Received: " + packet.DebugString());
    vector<coop::virtual_pose3_point3_t> pose_point_lcm_vec;
    vector<coop::virtual_pose3_pose3_t>  pose_pose_lcm_vec;
    vector<coop::relative_range_t>       relative_range_lcm_vec;
    vector<bool> acks;
    coop::global_t global_lcm;

    // initialize acks to false:
    for (int i=0; i<= N; i++)
      acks.push_back(false);

    incoming_id = packet.src_id();


    // now we chop all the pieces out and deal with them.
    // part 1 the pose pose measurement
    for(int i =0; i<packet.pose_poses_size(); i++){
      virtual_pose3d_pose3d_measurement pose_pose_proto = packet.pose_poses(i);
      coop::virtual_pose3_pose3_t pose_pose_lcm;
      proto2lcm(pose_pose_proto, pose_pose_lcm);
      // roll and pitch are not transmitted therefore explicitly set to zero here.
      pose_pose_lcm.pose.mu[4]=0.0;
      pose_pose_lcm.pose.mu[5]=0.0;
      // copy over the covariances for depth and compass explicitly (they never change)
      for (int j = 2; j<6; j++)
	for (int k = 2; k<6; k++)
	  pose_pose_lcm.pose.omega[j][k] = pose_pose.pose.omega[j][k];
      
      pose_pose_lcm.id = incoming_id;
      pose_pose_lcm_vec.push_back(pose_pose_lcm);
    }

    // part 2 the pose point measurements
    for(int i = 0; i < packet.pose_points_size(); i++){
      virtual_pose3d_point3d_measurement pose_point_proto = packet.pose_points(i);
      coop::virtual_pose3_point3_t pose_point_lcm;
      proto2lcm(pose_point_proto, pose_point_lcm);
      pose_point_lcm.id = incoming_id;
      pose_point_lcm_vec.push_back(pose_point_lcm);
    }

    // part 3 the acknowledgments
    for(int i = 0; i < packet.acks_size(); i++){
      acks[packet.acks(i).id()] = (packet.acks(i).bit() == 1);
    }

    // part 4  the global measurements
    if(packet.has_global()){
      global_measurement global_proto = packet.global();
      global_lcm = global;
      proto2lcm(global_proto,global_lcm);
      global_lcm.id = incoming_id;
    }

    // part 5 relative range measurements included in the packet
    for(int i = 0; i<packet.relative_ranges_size(); i++){
      relative_range_measurement relative_range_proto = packet.relative_ranges(i);
      coop::relative_range_t relative_range_lcm;
      proto2lcm(relative_range_proto,relative_range_lcm);
      relative_range_lcm.omega = 1/range_covariance;
      relative_range_lcm_vec.push_back(relative_range_lcm);
    }

    // part 6 calculate new relative range measurement
    double tof = reception.getRangingTime();
    double range = tof*m_sound_speed;
    coop::relative_range_t r;
    r.receive_t  = incoming_time;
    if(m_runtype == "simulation") // would be nice to have in simulation..
      r.send_t   = incoming_time;
    else
      r.send_t     = incoming_time - int64_t(tof*1000);
    r.send_id    = incoming_id; // what we just received from
    r.receive_id = id; // us
    r.omega = 1/range_covariance;
    r.mu = range;
    relative_range_lcm_vec.push_back(r);

    // part 7 build the lcm packet and send to Coop ISAM
    coop::acomms_packet_t to_send;
    to_send.source_id = incoming_id;
    to_send.num_poses = pose_pose_lcm_vec.size();
    to_send.virtual_pose_poses.resize(to_send.num_poses);
    for(int i=0;i<to_send.num_poses;i++){
      to_send.virtual_pose_poses[i]=pose_pose_lcm_vec[i];
    }
    to_send.num_landmarks = pose_point_lcm_vec.size();
    to_send.virtual_pose_points.resize(to_send.num_landmarks);
    for(int i=0 ;i<to_send.num_landmarks; i++){
      to_send.virtual_pose_points[i]=pose_point_lcm_vec[i];
    }
    to_send.num_ranges = relative_range_lcm_vec.size();
    to_send.ranges.resize(to_send.num_ranges);
    for(int i=0;i<to_send.num_ranges;i++){
      to_send.ranges[i]=relative_range_lcm_vec[i];
    }
    to_send.num_acks = N+1;
    to_send.acks.resize(N+1);
    for (int i=0; i<=N; i++){
      to_send.acks[i] = acks[i];
    }
    to_send.has_global = packet.has_global();
    if(to_send.has_global)
      to_send.global_pose = global_lcm;

    _lcm->publish("INCOMING_ACOMMS_PACKET",&to_send);


    // part 8 request a batch optimization step
    coop::command_t c;
    c.command = coop::command_list_t::BATCH; // do a batch optimization and also requests a trajectory be published
    _lcm->publish("COMMAND",&c);


  }
}



// End packet reception
// #######################################################

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool AcommsManager::OnConnectToServer()
{
   RegisterVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
 
bool AcommsManager::Iterate()
{
  AppCastingMOOSApp::Iterate();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AcommsManager::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  read_mission_file();

  _lcm = new lcm::LCM(provider.c_str());
  lcm_subscribe();

  // spawn the lcm_subscribe thread
  pthread_t lcm_subscribe_thread;
  if(pthread_create(&lcm_subscribe_thread,NULL,AcommsManager::lcm_handle,this)){
    cout << "Error creating LCM subscribe thread" << endl;
  }

  coop::command_t c;
  c.command = coop::command_list_t::SET_PACKET_TYPE;
  c.data    = packet_type;
  _lcm->publish("COMMAND",&c);


  RegisterVariables();
  Notify("ACOMMS_TRANSMIT_RATE",1.0); // TODO read this RATE in from mission file
  Notify("ACOMMS_TRANSMIT_DEST",0.0); // broadcast
  m_start_time = time_in_ms();
  return(true);

}

void AcommsManager::read_mission_file(){
  list<string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration("pAcommsManager", sParams)) {
    list<string>::iterator p;
    for(p=sParams.begin(); p!=sParams.end(); p++) {
      string original_line = *p;
      string param = stripBlankEnds(toupper(biteString(*p, '=')));
      string value = stripBlankEnds(*p);
      
      if(param == "VEHICLE_ID")
        id = atoi(value.c_str());
      else if(param == "NUM_AUVS")
	N = atoi(value.c_str());
      else if(param == "GPS_COVARIANCE"){
	Matrix2d GPS_covariance = read_matrix(value);
	Matrix2d GPS_information = GPS_covariance.inverse();
	for (int i=0; i<2; i++)
	  for (int j=0; j<2; j++)
	    global.pos.omega[i][j] = GPS_information(i,j);
      }
      else if(param == "COMPASS_COVARIANCE"){
	Matrix3d compass_covariance = read_matrix(value);
	Matrix3d compass_information = compass_covariance.inverse();
	for (int i=0; i<3; i++){
	  for (int j=0; j<3; j++){
	    global.attitude.omega[i][j] = compass_information(i,j);
	    pose_pose.pose.omega[i+3][j+3] = compass_information(i,j);
	  }
	}
      }
      else if(param == "DEPTH_COVARIANCE"){
	double depth_covariance = atof(value.c_str());
	global.depth.omega = 1/depth_covariance;
	pose_pose.pose.omega[2][2] = 1/depth_covariance;
      }
      else if(param == "RANGE_COVARIANCE")
	range_covariance = atof(value.c_str());
      else if(param == "BACKGROUND_MODE")
	background_mode = (tolower(value) == "true");
      else if(param == "LCM_PROVIDER")
	provider = "udpm://239.255.76.56:" + value;
      else if(param == "RUNTYPE")
	m_runtype = value;
      else if(param == "SPEED_OF_SOUND")
	m_sound_speed = atof(value.c_str());
      else if(param == "PACKET_TYPE")
	packet_type = atof(value.c_str());		     
      else 
	reportUnhandledConfigWarning(original_line);
    }
  }
}
  

//---------------------------------------------------------
// Procedure: RegisterVariables

void AcommsManager::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SCHEDULER_TRANSMIT","*", 0);
  Register("ACOMMS_RECEIVED","*",0); 
  Register("ACOMMS_DRIVER_STATUS","*",0);
  Register("DEPLOY","*",0);
}



//-----------------------------------------------------
// LCM stuff:
void * AcommsManager::lcm_handle(void* data){
  while(0==((AcommsManager *) data)->_lcm->handle());
}

void AcommsManager::lcm_subscribe(){
  _lcm->subscribe(string("OUTGOING_ACOMMS_PACKET"),
		  &AcommsManager::on_acomms_packet,
		  this);
}


//#################################################################3
// Begin packet transmission

// procedure: on_acomms_packet
// description: receive the acomms packet from lcm, chunk it up, convert it to protobufs and send to modem.
void AcommsManager::on_acomms_packet(const lcm::ReceiveBuffer* rbuf,
				     const string& channel,
				     const coop::acomms_packet_t* msg){


  packet_master packet;
  packet.set_src_id(id);

  //step 1 process the pose_pose_measurements
  for (int i=0; i<msg->num_poses; i++){
    virtual_pose3d_pose3d_measurement* pose_pose_proto = packet.add_pose_poses(); 
    lcm2proto(msg->virtual_pose_poses[i], pose_pose_proto);
  }

  // step 2 process the pose_landmark measurements
  for (int i=0; i< msg->num_landmarks; i++){
    virtual_pose3d_point3d_measurement* pose_point_proto = packet.add_pose_points();
    lcm2proto(msg->virtual_pose_points[i], pose_point_proto);
  }

  // step 3 process ack bits to send
  for (int i=0; i< msg->num_acks; i++){
    ack* ack_proto = packet.add_acks();
    ack_proto->set_id(i);
    ack_proto->set_bit(msg->acks[i]);
  }

  // step 4 ranges
  for (int i=0; i<msg->num_ranges; i++){
    relative_range_measurement* range_proto = packet.add_relative_ranges();
    lcm2proto(msg->ranges[i], range_proto);
  }

  // step 5 global
  if(msg->has_global){
    global_measurement global_proto;
    lcm2proto(msg->global_pose, &global_proto);
    packet.mutable_global()->CopyFrom(global_proto); 
  }
  
  // do the encoding and posting
  string bytes;
  codec.encode(&bytes,packet);
  Notify("ACOMMS_TRANSMIT_DATA_BINARY",(void*) bytes.data(),bytes.size());
  reportEvent("Transmitting" + packet.DebugString());
  outgoing_count++;

}

// End packet transmission
// #############################################################


bool AcommsManager::buildReport(){
  m_msgs << "Number of acomms packets received: " << incoming_count << endl;
  m_msgs << "Number of acomms transmissions   : " << outgoing_count << endl;
  m_msgs << "Driver status: " << int(m_driver_status) << endl;
}
