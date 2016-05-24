#include <CoopIsam.h>
#include <lcm_utils.h> // using for the time_in_ms() function

#define DEBUG 1

CoopIsam::CoopIsam(int N, int id, string provider):_N(N),_id(id){
  _lcm = new lcm::LCM(provider.c_str());
  _coop_fg = new FactorGraph(N,id,true, provider.c_str());
  _packet_manager = new PacketManager(N,id);
  lcm_subscribe();
}

CoopIsam::~CoopIsam(){
  delete _lcm;
  delete _coop_fg;
  delete _packet_manager;
}

void CoopIsam::run(){
  while(0==_lcm->handle());
}


void CoopIsam::lcm_subscribe(){

  const string odom_channel="ODOM_XY_GLOBAL_YPRZ";
  _lcm->subscribe(odom_channel,&CoopIsam::on_odom_xy_global_yprz,this);
  const string sidescan_detections_channel="SIDESCAN_DETECTIONS";
  _lcm->subscribe(sidescan_detections_channel,
		&CoopIsam::on_sidescan_detections,
		this);
   const string range_channel="RANGE";
  _lcm->subscribe(range_channel,
		  &CoopIsam::on_range,
		  this);
  const string global_channel="GLOBAL";
  _lcm->subscribe(global_channel,
		  &CoopIsam::on_global,
		  this);
  const string command_channel="COMMAND";
  _lcm->subscribe(command_channel,
		  &CoopIsam::on_command,
		  this);
  const string acomms_in_channel="INCOMING_ACOMMS_PACKET";
  _lcm->subscribe(acomms_in_channel,
		  &CoopIsam::on_acomms_in,
		  this);
}

void CoopIsam::lcm_publish_pose(){
  pose3_t p;
  if(!(_coop_fg->get_current_pose(_id,p)))
    return;
  p.id = _id;
  p.t = _coop_fg->get_current_time(_id);
  _lcm->publish("CURRENT_POSE",&p);
}

void CoopIsam::lcm_publish_trajectory(){
  trajectory_t t;
  if(!(_coop_fg->get_trajectory(_id,t)))
    return;
  _lcm->publish("TRAJECTORY",&t);
}

void CoopIsam::on_odom_xy_global_yprz(const lcm::ReceiveBuffer* rbuf,
				      const string& channel,
				      const coop::odom_xy_global_yprz_t* msg){
  if(!_coop_fg->add_odom_xy_global_yprz(*msg))
    request_prior();
}

void CoopIsam::on_sidescan_detections(const lcm::ReceiveBuffer* rbuf,
				      const string& channel,
				      const coop::sidescan_detections_t* msg){
  // cycle through them and add them all individually
  for (int i=0; i<msg->n;i++){
    coop::sidescan_detection_t s = msg->detections[i];
    _coop_fg->add_sidescan_detection(s);
  }

}

void CoopIsam::on_range(const lcm::ReceiveBuffer* rbuf,
			const string& channel,
			const coop::relative_range_t* msg){
  process_range(msg);
}

void CoopIsam::process_range(const coop::relative_range_t* msg){
  if(DEBUG) cout << "processing range measurement from " << msg->send_id << "("<<msg->send_t<<") to " << msg->receive_id << "("<<msg->receive_t<<") " << endl;
 
  _coop_fg->add_relative_range(*msg); 
}

void CoopIsam::on_global(const lcm::ReceiveBuffer* rbuf,
			 const string& channel,
			 const coop::global_t* msg){
  process_global(msg);
}

void CoopIsam::process_global(const coop::global_t* msg){
  if(msg->id == _id && _coop_fg->num_poses(_id)==0){ // this is the first one.
    _packet_manager->initialize_CPs(msg->t);
    _packet_manager->add_global(*msg);
  }
  if (msg->id > _N){
    cout << "Warning received measurement for id out of range : " << msg->id << endl;
  }
  // TODO should make sure we haven't received it in a previous packet, in which case we are adding it twice.
  _coop_fg->add_global(*msg);
}

void CoopIsam::on_acomms_in(const lcm::ReceiveBuffer* rbuf,
			    const string& channel,
			    const coop::acomms_packet_t* msg){
  if (DEBUG) cout << "Received acomms packet." << endl;

  // step 1 update the packet manager with the received ack bit etc
  _packet_manager->packet_received(msg->source_id,msg->acks[_id]); // msg->id is id of sender. msg->acks[_id] is the ack bit for us (did robot msg->id receive our last transmission?)

  // step 2 start sending stuff to coop factor graph. Note that the order matters here, the global had better go first and the ranges last.

  if (msg->has_global){
    const global_t global = msg->global_pose;
    process_global(&global);
  }
  for (int i=0; i<msg->num_poses;i++){
    if (DEBUG) cout << "adding virtual pose pose measurement factor for vehicle " << msg->source_id << endl;
    _coop_fg->add_virtual_pose_pose(msg->virtual_pose_poses[i]);
  }
  for(int i=0; i<msg->num_landmarks; i++){
    if (DEBUG) cout << "adding virtual point pose measurement factor for vehicle " << msg->source_id << endl;
    _coop_fg->add_virtual_pose_point(msg->virtual_pose_points[i]);
  }

  for (int i=0; i<msg->num_ranges; i++){
    const relative_range_t range = msg->ranges[i];
    process_range(&range);
  }

  if (DEBUG) cout << "done adding in acomms packet" << endl;
}

void CoopIsam::on_command(const lcm::ReceiveBuffer* rbuf,
			  const string& channel,
			  const coop::command_t* msg){

  switch(msg->command){
  case coop::command_list_t::UPDATE:
    {
      //      _coop_fg->print();
      _coop_fg->update();
      _coop_fg->update_viewer();
      lcm_publish_pose();
      break;
    }
  case coop::command_list_t::BATCH:
    {
      if (DEBUG) cout << "isam_coop start batch optimization" << endl;
      _coop_fg->batch();
      //_coop_fg->print();
      if (DEBUG) cout << "isam_coop end batch optimization" << endl;
      _coop_fg->update_viewer();
      lcm_publish_pose();
      lcm_publish_trajectory();
      break;
    }
  case coop::command_list_t::POST_TRAJECTORY:
    {
      lcm_publish_trajectory();
      break;
    }
  case coop::command_list_t::GENERATE_PACKET:
    {
      cout << "Received Generate Packet command at time: " << time_in_ms() <<  endl;
      int64_t t1 = time_in_ms();
      coop::acomms_packet_t generated_packet;
      if(_packet_manager->generate(_coop_fg,generated_packet))
	_lcm->publish("OUTGOING_ACOMMS_PACKET", &generated_packet);
      cout << "Finished Processing packet request at time:  " << time_in_ms() << endl;
      int64_t t2 = time_in_ms();
      cout << "Processing time in ms: " << t2 - t1 << endl;
      break;
    }
  case coop::command_list_t::SET_PACKET_TYPE:
    {
      Packet::PacketType p = static_cast<Packet::PacketType>(msg->data);
      _packet_manager->set_packet_type(p);
      cout << "Setting packet type to: " << p << endl;
      break;
    }
  default:
    {
      cout << "Warning: Unknown command received" << endl;
      break;
    }
  }
}

void CoopIsam::request_prior(){
  coop::command_t c;
  c.command = coop::command_list_t::REQUEST_PRIOR;
  _lcm->publish("COMMAND",&c);
}
