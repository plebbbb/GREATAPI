//****************Universal settings****************

#define rotateVoltCap 10000 // voltage cap while rotating
#define moveRotVoltCap 6000 // voltage cap for rotation while moving
#define moveVoltCap 10000 // voltage cap for straight line movement

//******************For Tank drive******************

//set PID values for maintaining angle while translating.
#define kPAngle 18000
#define kIAngle 5000
#define kDAngle 160000

// Different PID value for rotate command. kP should be greater.
#define kPRotate 25000
#define kIRotate 5000
#define kDRotate 160000

//*******************For X-drive*******************