// Standalone pseudo-terminal test: no ROS graph and no motor device.
#include "port_handler_linux.h"
#include <pty.h>
#include <unistd.h>
#include <cassert>
#include <cstdio>
int main() {
  int master, slave; char name[128];
  assert(openpty(&master, &slave, name, nullptr, nullptr) == 0);
  close(slave);
  dynamixel::PortHandlerLinux port(name);
  assert(port.openPort());
  uint8_t byte=0;
  assert(port.readPort(&byte, 1) == 0);
  const uint8_t sent=42;
  assert(write(master, &sent, 1) == 1);
  int count=0;
  for(int i=0;i<100 && count==0;++i) {count=port.readPort(&byte,1);usleep(1000);}
  assert(count==1 && byte==sent);
  port.closePort();
  assert(port.readPort(&byte, 1) == -1);
  close(master);
  puts("3 Linux port cases passed");
}
