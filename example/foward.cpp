#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.161", 8082)
  {
    udp.InitCmdData(cmd);
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
  int motiontime = 0;
  float dt = 0.002; // 控制循环的时间间隔
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime += 2;
  udp.GetRecv(state);

  // 设置机器人行走的初始命令
  cmd.mode = 2;             // 行走模式
  cmd.gaitType = 1;         // 步态类型
  cmd.speedLevel = 0;       // 速度级别
  cmd.footRaiseHeight = 0;  // 抬脚高度
  cmd.bodyHeight = 0;       // 身体高度
  cmd.euler[0] = 0;         // 身体姿态：俯仰角
  cmd.euler[1] = 0;         // 身体姿态：横滚角
  cmd.euler[2] = 0;         // 身体姿态：偏航角
  cmd.velocity[0] = 0.1f;   // 机器人前进速度（单位：米/秒）
  cmd.velocity[1] = 0.0f;   // 机器人左右移动速度（单位：米/秒）
  cmd.yawSpeed = 0.0f;      // 机器人旋转速度（单位：弧度/秒）
  cmd.reserve = 0;

  // 根据运动时间判断何时停止
  if (motiontime >= 2500) // 持续时间 2500*dt 秒 -> 0.25 米（基于 0.1 米/秒）
  {
    cmd.mode = 0;         // 停止行走
    cmd.velocity[0] = 0;  // 停止前进
  }

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (custom.motiontime < 2500)
  {
    usleep(2000); // 2ms 延迟
  }

  // 程序结束时，停止循环并结束程序
  loop_udpSend.shutdown();
  loop_udpRecv.shutdown();
  loop_control.shutdown();

  std::cout << "Program finished. Robot walked 0.25 meters." << std::endl;

  return 0;
}
