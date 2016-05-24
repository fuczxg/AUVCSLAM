#include <iostream>
#include <dccl.h>
#include "protobufs/factors.pb.h"
#include "protobufs/packet.pb.h"

using namespace std;

int main(){
  std::string encoded_bytes;
  dccl::Codec codec;
  codec.load<packet_PSK1>();
  
  packet_master packet;
  packet.set_src_id(1);

  //step 1 process the pose_pose_measurement
  virtual_pose3d_pose3d_measurement proto_p; 
  proto_p.set_mu_x(5);
  proto_p.set_mu_y(-5);
  proto_p.set_mu_z(0.2);
  proto_p.set_mu_psi(0.02);
  proto_p.set_omega_xx(20);
  proto_p.set_omega_xy(0.9);
  proto_p.set_omega_yy(10);
  proto_p.set_t1(10001);
  proto_p.set_t2(10002);
  packet.mutable_pose_pose()->CopyFrom(proto_p);


  // step 3 process ack bits to send
  for (int i=0; i< 2; i++){
    ack* ack_proto = packet.add_acks();
    ack_proto->set_id(1);
    ack_proto->set_bit(0);
  }

  if(1){
    global_measurement global_proto;
    global_proto.set_x(10);
    global_proto.set_y(12);
    global_proto.set_z(1);
    global_proto.set_psi(0.2);
    global_proto.set_theta(0.23);
    global_proto.set_phi(0.32);
    global_proto.set_t(1010101);

    packet.mutable_global()->CopyFrom(global_proto);
    
  }
  
  // do the encoding and posting
  string bytes;
  codec.encode(&bytes,packet);
  cout << "Encoded: " << endl;
  cout << bytes.size() << endl;
  cout << packet.DebugString() << endl;

  packet_PSK1 packet_in;
  codec.decode(bytes,&packet_in);
  cout << "Decoded: " << endl;
  cout << packet_in.DebugString() << endl;

}
