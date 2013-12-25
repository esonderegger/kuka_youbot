#include <youbot_driver/testing/YouBotBaseTestWithoutThread.hpp>
#include <stdlib.h>
#include <stdexcept>

using namespace youbot;

YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread()
{

}

YouBotBaseTestWithoutThread::~YouBotBaseTestWithoutThread()
{
}

void YouBotBaseTestWithoutThread::setUp()
{
  char* location = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (location == NULL)
    throw std::runtime_error(
        "YouBotBaseTestWithoutThread.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  Logger::logginLevel = trace;
  ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", location, false);
  if (ethercatMaster->isThreadActive())
  {
    LOG(error) << "Thread Active";
    EthercatMaster::destroy();
    ethercatMaster = &EthercatMaster::getInstance("youbot-ethercat.cfg", location, false);
  }

  jointNO = 4;
  stepStartTime = 1000;
  durationNull = 1000;
  overallTime = 0;
  startTime = 0;
  updateCycle = 2000;
  setAngle.angle = 0 * radian;
  setVel.angularVelocity = 0 * radian_per_second;
  currentSetpoint.current = 0 * ampere;
}

void YouBotBaseTestWithoutThread::tearDown()
{

  EthercatMaster::destroy();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_PositionMode()
{
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("YouBotArmTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  LOG(info) << __func__ << "\n";
  YouBotBase myBase("youbot-base", configLocation);
  myBase.doJointCommutation();
  DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
  myBase.getBaseJoint(jointNO).setEncoderToZero();
  if (!ethercatMaster->isThreadActive())
  {
    ethercatMaster->sendProcessData();
    ethercatMaster->receiveProcessData();
  }
  myTrace.startTrace();

  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);

  startTime = myTrace.getTimeDurationMilliSec();
  overallTime = startTime + durationNull + stepStartTime + durationNull;

  while (myTrace.getTimeDurationMilliSec() < overallTime)
  {
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull)
    {
      setAngle.angle = 0 * radian;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
        && myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime)
    {
      setAngle.angle = 2 * radian;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime)
    {
      setAngle.angle = 0 * radian;
    }

    myBase.getBaseJoint(jointNO).setData(setAngle);
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    myTrace.updateTrace(setAngle);

    SLEEP_MICROSEC(updateCycle);
  }
  myTrace.stopTrace();
  myTrace.plotTrace();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_VelocityMode()
{
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("YouBotArmTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  LOG(info) << __func__ << "\n";
  YouBotBase myBase("youbot-base", configLocation);
  myBase.doJointCommutation();
  DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
  myBase.getBaseJoint(jointNO).setEncoderToZero();
  if (!ethercatMaster->isThreadActive())
  {
    ethercatMaster->sendProcessData();
    ethercatMaster->receiveProcessData();
  }
  myTrace.startTrace();

  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);

  startTime = myTrace.getTimeDurationMilliSec();
  overallTime = startTime + durationNull + stepStartTime + durationNull;

  while (myTrace.getTimeDurationMilliSec() < overallTime)
  {
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull)
    {
      setVel.angularVelocity = 0 * radian_per_second;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
        && myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime)
    {
      setVel.angularVelocity = 2 * radian_per_second;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime)
    {
      setVel.angularVelocity = 0 * radian_per_second;
    }

    myBase.getBaseJoint(jointNO).setData(setVel);
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    myTrace.updateTrace(setVel);

    SLEEP_MICROSEC(updateCycle);
  }
  myTrace.stopTrace();
  myTrace.plotTrace();
}

void YouBotBaseTestWithoutThread::YouBotBaseTestWithoutThread_CurrentMode()
{
  char* configLocation = getenv("YOUBOT_CONFIG_FOLDER_LOCATION");
  if (configLocation == NULL)
    throw std::runtime_error("YouBotArmTest.cpp: Could not find environment variable YOUBOT_CONFIG_FOLDER_LOCATION");

  LOG(info) << __func__ << "\n";
  YouBotBase myBase("youbot-base", configLocation);
  myBase.doJointCommutation();
  DataTrace myTrace(myBase.getBaseJoint(jointNO), __func__, true);
  myBase.getBaseJoint(jointNO).setEncoderToZero();
  if (!ethercatMaster->isThreadActive())
  {
    ethercatMaster->sendProcessData();
    ethercatMaster->receiveProcessData();
  }
  myTrace.startTrace();

  ClearMotorControllerTimeoutFlag clearTimeoutFlag;
  myBase.getBaseJoint(1).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(2).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(3).setConfigurationParameter(clearTimeoutFlag);
  myBase.getBaseJoint(4).setConfigurationParameter(clearTimeoutFlag);

  startTime = myTrace.getTimeDurationMilliSec();
  overallTime = startTime + durationNull + stepStartTime + durationNull;

  while (myTrace.getTimeDurationMilliSec() < overallTime)
  {
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull)
    {
      currentSetpoint.current = 0 * ampere;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull
        && myTrace.getTimeDurationMilliSec() < startTime + durationNull + stepStartTime)
    {
      currentSetpoint.current = 0.5 * ampere;
    }
    if (myTrace.getTimeDurationMilliSec() > startTime + durationNull + stepStartTime)
    {
      currentSetpoint.current = 0 * ampere;
    }

    myBase.getBaseJoint(jointNO).setData(currentSetpoint);
    if (!ethercatMaster->isThreadActive())
    {
      ethercatMaster->sendProcessData();
      ethercatMaster->receiveProcessData();
    }
    myTrace.updateTrace(currentSetpoint);

    SLEEP_MICROSEC(updateCycle);
  }
  myTrace.stopTrace();
  myTrace.plotTrace();
}
