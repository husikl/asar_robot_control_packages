#ifndef _FORCEPS_STATUS_CODE_H_
#define _FORCEPS_STATUS_CODE_H_

// Function return status

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#define EXIT_PREEMPTED 2
#define EXIT_ABORTED 3

// Object number
#define FORCEPS0 0
#define FORCEPS1 1

enum STATUS
{
  STOPPED = 0,
  MOVING
};

// Robot Status
enum
{
  F_UNINITIALIZED = 0,
  F_CONNECTED,
  F_READY,
  F_POSITION_SLAVE,
  F_TORQUE_SLAVE,
};

// ROBOT Actions
// enum
// {
//   F_CONNECT = 0,
//   F_DISCONNECT,
//   F_CALIBRATE,
//   F_POSITION_CONTROL,
//   F_TORQUE_CONTROL,
// };

// FORCEPS Modes
enum
{
  MODE_MANUAL = 0,
  MODE_AUTO,
};

#endif
