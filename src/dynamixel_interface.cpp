/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000)
//

#include "dynamixel_interface.h"

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

DxlInterface::DxlInterface(int mod, int baud, char* port) : model(mod), baudrate(baud), portname(port) {
  if (mod == 430) {
    protocol = 2.0;
    addrTorqueEnable = 64;
    addrOperatingMode = 11;
    addrGoalPosition = 116;
    addrGoalCurrent = 102;
    addrPresentPosition = 132;

  } else if (mod == 540) {
    protocol = 2.0;
    addrTorqueEnable = 64;
    addrOperatingMode = 11;
    addrGoalPosition = 116;
    addrGoalCurrent = 102;
    addrPresentPosition = 132;

  } else {
    printf("Construction failed. Currently only the models XM-430 and XM-540 are supported. \n");
    return;
  }

  portHandler = dynamixel::PortHandler::getPortHandler(portname);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol);

  startPosition = END_POSITION;

  dxl_comm_result = COMM_TX_FAIL;
  dxl_error = 0;
  dxl_present_position = 0;
}

bool DxlInterface::initialize() {

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return false;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(baudrate))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return false;
  }

  // Set start position
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, addrPresentPosition, (uint32_t*)&dxl_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    startPosition = dxl_present_position;
  }

  return true;
}

bool DxlInterface::enableTorque() {

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, addrTorqueEnable, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  return true;

}

bool DxlInterface::disableTorque() {

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, addrTorqueEnable, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  return true;

}

bool DxlInterface::setModeCurrent() {

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, addrOperatingMode, CURRENT_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  else
  {
    printf("Current control mode successfully engaged \n");
    return true;
  }


}


bool DxlInterface::setModePosition() {

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, addrOperatingMode, POSITION_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  else
  {
    printf("Position control mode successfully engaged \n");
    return true;
  }


}

bool DxlInterface::setModeExtendedPosition() {

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, addrOperatingMode, EXTENDED_POSITION_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  else
  {
    printf("Extended position control mode successfully engaged \n");
    return true;
  }


}
bool DxlInterface::setGoalCurrent(short int goalCurrent) {

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, addrGoalCurrent, goalCurrent, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }
  return true;

}

bool DxlInterface::setGoalPosition(short int goalPos) {

  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, addrGoalPosition, goalPos, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }

  do
  {
    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, addrPresentPosition, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, goalPos, dxl_present_position);

  }while((abs(goalPos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  return true;

}

bool DxlInterface::resetPosition() {

  //dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, addrPresentPosition, (uint32_t*)&dxl_present_position, &dxl_error);
  int goalPosition = startPosition;
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, addrGoalPosition, goalPosition, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
    return false;
  }

  do
  {
    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, addrPresentPosition, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, goalPosition, dxl_present_position);

  }while((abs(goalPosition - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  return true;

}

bool DxlInterface::close() {

  disableTorque();
  portHandler->closePort();

  return true;

}

int dxl_interface_demo() {
  DxlInterface dxl_interface (430, BAUDRATE, DEVICENAME);
  
  short int dxl_goal_current[2] = {-DXL_GOAL_CURRENT, DXL_GOAL_CURRENT};
  int index = 0;

  dxl_interface.initialize();

  dxl_interface.setModeCurrent();
  dxl_interface.enableTorque();

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;
 
    // Write goal current
    dxl_interface.setGoalCurrent(dxl_goal_current[index]);   

    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }
  // Reset goal current
  dxl_interface.setGoalCurrent(0);


  // Disable Dynamixel Torque
  dxl_interface.disableTorque();

  // Switch to extended position control
  dxl_interface.setModeExtendedPosition();

  // Enable Dynamixel Torque
  dxl_interface.enableTorque();

  // Reset Position
  dxl_interface.resetPosition();

  // Close port
  dxl_interface.close();

  return 0;
  

}
/*
int main() {

  return dxl_interface_demo();

}
*/

/*
int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
  short int dxl_goal_current[2] = {-DXL_GOAL_CURRENT, DXL_GOAL_CURRENT};

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Change Operating Mode to Current Control

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XM430_OPERATING_MODE, CURRENT_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    printf("Current control mode successfully engaged \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
 
    // Write goal current
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM430_GOAL_CURRENT, dxl_goal_current[index], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    

    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
  }
  // Reset goal current
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM430_GOAL_CURRENT, 0, &dxl_error);


  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  // Switch to position control
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XM430_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    printf("Position control mode successfully engaged \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  // Write goal position
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, END_POSITION, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

  do
  {
    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, END_POSITION, dxl_present_position);

  }while((abs(END_POSITION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

  return 0;
}
*/
