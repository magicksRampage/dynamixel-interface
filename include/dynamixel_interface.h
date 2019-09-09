#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_XM430_OPERATING_MODE	11
#define ADDR_XM430_GOAL_CURRENT		102
#define ADDR_PRO_TORQUE_ENABLE          64//562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116//596
#define ADDR_PRO_PRESENT_POSITION       132//611

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define CURRENT_CONTROL			0		    // Value for current control mode
#define POSITION_CONTROL		3		    // Value for position control mode
#define EXTENDED_POSITION_CONTROL	4		    // Value for extended position control mode
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define DXL_GOAL_CURRENT		1000		// Goal curent
#define END_POSITION			0

#define ESC_ASCII_VALUE                 0x1b

int getch();
int kbhit(void);

int dxl_interface_demo();

class DxlInterface {
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  
  int model;
  int baudrate;
  char* portname;
  float protocol;

  int startPosition;

  int addrTorqueEnable;
  int addrOperatingMode;
  int addrGoalPosition;
  int addrGoalCurrent;
  int addrPresentPosition;

  int dxl_comm_result;             // Communication result
  uint8_t dxl_error;                          // Dynamixel error
  int32_t dxl_present_position;               // Present position
public:
  DxlInterface (int,int,char*);

  bool initialize();
  bool enableTorque();
  bool disableTorque();
  bool setModeCurrent();
  bool setModePosition();
  bool setModeExtendedPosition();
  bool setGoalCurrent(short int);
  bool setGoalPosition(short int);
  bool resetPosition();
  bool close();
};
