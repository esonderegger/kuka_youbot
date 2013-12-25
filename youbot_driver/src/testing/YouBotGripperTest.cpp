#include <youbot_driver/testing/YouBotGripperTest.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>

using namespace youbot;

YouBotGripperTest::YouBotGripperTest() :
    dof(5)
{
  char* location = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (location == NULL)
    throw std::runtime_error(
        "YouBotGripperTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");
  EthercatMaster::getInstance("youbot-ethercat.cfg", location, true);

}

YouBotGripperTest::~YouBotGripperTest()
{

}

void YouBotGripperTest::setUp()
{
  Logger::logginLevel = trace;
  updateCycle = 2000;

}

void YouBotGripperTest::tearDown()
{
  //	EthercatMaster::destroy();
}

void YouBotGripperTest::youBotGripperTest()
{
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("YouBotArmTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  LOG(info) << __func__ << "\n";
  YouBotManipulator myArm("youbot-manipulator", configLocation);

  GripperDataTrace myTrace(myArm.getArmGripper().getGripperBar2(), __func__, true);

  TargetPositionReached bar1TargetReched;
  TargetPositionReached bar2TargetReched;
  bool targetReachedBar1 = false;
  bool targetReachedBar2 = false;

  myArm.calibrateGripper(true);
  myTrace.startTrace("Load", "");

  //open gripper
  myArm.getArmGripper().open();

  for (int i = 0; i < 40; i++)
  {
    myArm.getArmGripper().getGripperBar1().getConfigurationParameter(bar1TargetReched);
    bar1TargetReched.getParameter(targetReachedBar1);
    myArm.getArmGripper().getGripperBar2().getConfigurationParameter(bar2TargetReched);
    bar2TargetReched.getParameter(targetReachedBar2);
    myTrace.updateTrace((double)targetReachedBar1);
    if (targetReachedBar1 && targetReachedBar2)
    {
      break;
    }
  }
  targetReachedBar1 = false;
  targetReachedBar2 = false;

  //close gripper
  myArm.getArmGripper().close();

  for (int i = 0; i < 40; i++)
  {
    myArm.getArmGripper().getGripperBar1().getConfigurationParameter(bar1TargetReched);
    bar1TargetReched.getParameter(targetReachedBar1);
    myArm.getArmGripper().getGripperBar2().getConfigurationParameter(bar2TargetReched);
    bar2TargetReched.getParameter(targetReachedBar2);
    myTrace.updateTrace((double)targetReachedBar2);
    if (targetReachedBar1 && targetReachedBar2)
    {
      break;
    }
  }

  myTrace.stopTrace();
  myTrace.plotTrace();

}
