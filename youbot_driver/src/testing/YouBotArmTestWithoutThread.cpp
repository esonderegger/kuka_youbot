#include <youbot_driver/testing/YouBotArmTestWithoutThread.hpp>
#include <stdlib.h>
#include <stdexcept>

using namespace youbot;

YouBotArmTestWithoutThread::YouBotArmTestWithoutThread() :
    dof(5)
{

}

YouBotArmTestWithoutThread::~YouBotArmTestWithoutThread()
{

}

void YouBotArmTestWithoutThread::setUp()
{
  char* location = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (location == NULL)
    throw std::runtime_error(
        "YouBotArmTestWithoutThread.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  Logger::logginLevel = trace;
  updateCycle = 2000;
  ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", location, false);
  if (ethercatMaster->isThreadActive())
  {
    LOG(error) << "Thread Active";
    EthercatMaster::destroy();
    ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", location, false);
  }

}

void YouBotArmTestWithoutThread::tearDown()
{
  EthercatMaster::destroy();
}

void YouBotArmTestWithoutThread::youBotArmTest()
{
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("YouBotArmTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  LOG(info) << __func__ << "\n";
  YouBotManipulator myArm("youbot-manipulator", configLocation);
  myArm.doJointCommutation();
  myArm.calibrateManipulator();

  std::stringstream jointNameStream;
  boost::ptr_vector<DataTrace> myTrace;
  std::vector<JointAngleSetpoint> upstretchedpose;
  std::vector<JointAngleSetpoint> foldedpose;
  JointAngleSetpoint desiredJointAngle;

  desiredJointAngle.angle = 2.56244 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.04883 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -2.43523 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.73184 * radian;
  upstretchedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 1.5 * radian;
  upstretchedpose.push_back(desiredJointAngle);

  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = -0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.11 * radian;
  foldedpose.push_back(desiredJointAngle);
  desiredJointAngle.angle = 0.2 * radian;
  foldedpose.push_back(desiredJointAngle);

  for (int i = 1; i <= dof; i++)
  {
    jointNameStream << "Joint_" << i << "_" << __func__;
    myTrace.push_back(new DataTrace(myArm.getArmJoint(i), jointNameStream.str(), true));
    jointNameStream.str("");
  }

  for (int i = 0; i < dof; i++)
  {
    myTrace[i].startTrace();
  }

  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myArm.getArmJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myArm.getArmJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myArm.getArmJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myArm.getArmJoint(4).setConfigurationParameter(clearTimeoutFlag);
  myArm.getArmJoint(5).setConfigurationParameter(clearTimeoutFlag);

  if (!ethercatMaster->isThreadActive())
  {
    ethercatMaster->sendProcessData();
    ethercatMaster->receiveProcessData();
  }

  // 1 sec no movement
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 1000;
  while (myTrace[0].getTimeDurationMilliSec() < overallTime)
  {
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    for (int i = 0; i < dof; i++)
    {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }

  // move to upstretched position
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 5000;
  myArm.setJointData(upstretchedpose);
  while (myTrace[0].getTimeDurationMilliSec() < overallTime)
  {
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    for (int i = 0; i < dof; i++)
    {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }

  // move to folded position
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 5000;
  myArm.setJointData(foldedpose);
  while (myTrace[0].getTimeDurationMilliSec() < overallTime)
  {
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    for (int i = 0; i < dof; i++)
    {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }

  // 1 sec no movement
  startTime = myTrace[0].getTimeDurationMilliSec();
  overallTime = startTime + 1000;
  while (myTrace[0].getTimeDurationMilliSec() < overallTime)
  {
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    for (int i = 0; i < dof; i++)
    {
      myTrace[i].updateTrace();
    }
    SLEEP_MICROSEC(updateCycle);
  }

  for (int i = 0; i < dof; i++)
  {
    myTrace[i].stopTrace();
    myTrace[i].plotTrace();
  }

}
