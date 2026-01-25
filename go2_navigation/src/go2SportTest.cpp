#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>

// ./install/go2_smoke/lib/go2_smoke/go2SportTest enp0s31f6
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();


  sport_client.Sit();
  sleep(3);
  sport_client.RiseSit();
  sleep(3);

  return 0;
}