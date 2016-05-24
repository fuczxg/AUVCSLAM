#include <CoopIsam.h>
#include <iostream>
#include <string>

void usage(){
  std::cout << "Usage: CoopIsam -n <num_robots> -i <robot_id> -l <lcm-port>" << std::endl;
};


int main(int argc, char* argv[])
{
  if (argc != 7)
    usage();

  int N,id;
  std::string provider;
  for (int i=1; i<argc; i+=2){
    string argi=argv[i];
    if(argi=="-n")
      N = atoi(argv[i+1]);
    else if(argi=="-i")
      id = atoi(argv[i+1]);
    else if(argi=="-l")
      provider = "udpm://239.255.76.56:" + string(argv[i+1]) + "?ttl=1";
    else
      usage();
  }
  
  std::cout << "Launching Coop Isam with N = " << N << " id = " << id << std::endl;

  CoopIsam _CoopIsam(N,id,provider);
  _CoopIsam.run();
  return 0;

}
