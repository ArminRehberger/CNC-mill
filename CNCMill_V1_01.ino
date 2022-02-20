/*
""" ############################################################################################ """
""" ############################################################################################ """
""" CNC mill with Teensy4.0 """
""" V1_01, 2022-02-20, are """
Changes:
G68 and G69 implemented, Coordinate System Rotation
M98 and M99 implemented, Subprogram Call and Subprogram end
O implemented, Subprogram number
""" ############################################################################################ """
""" ############################################################################################ """

/*
// ###############################################################################################################
// ###############################################################################################################
MotorCommon[0]. = X-axis
MotorCommon[1]. = Y-axis
MotorCommon[2]. = Z-axis
*/

/*
// ###############################################################################################################
// ###############################################################################################################
##### Libary for I2C Raspberry <-> Teensy4.0:
https://github.com/Richard-Gemmell/teensy4_i2c
i2c_driver.h
i2c_driver_wire.h
Call functions with Wire1.x
Teensy slave address 8
Teensy Pin 16 SCL1 Raspberry SCL GPIO 27, Pin 13
Teensy Pin 17 SDA1 Raspberry SDA GPIO 17, Pin 11
Pull up resistors 2K2 Ohm

##### Libary for LiquidChristal, I2C:
https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c
LiquidCrystal_I2C.h
Required changes:
In file:  LiquidCrystal_I2C.H   --> #include <Wire.h> replaced to #include <i2c_driver_wire.h>
          LiquidCrystal_I2C.CPP --> #include <Wire.h> replaced to #include <i2c_driver_wire.h>
*/

/*
// ###############################################################################################################
// ###############################################################################################################
##### I2C Master --> Slave, Struct I2C recive, max. 32 Byte possible
Raspberry → Teensy, Byte 0..27, 28 Byte
Byte 0
  Bit 0 = Operate (0 - Stepper disabled 1 - Stepper enabled)
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = ActivateCommand
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Manual command Spindle on
  Bit 1 = Manual command Coolant on
  Bit 2 = Manual command Clamp on
  Bit 3 = Manual command Light on
  Bit 4 = Manual command X+
  Bit 5 = Manual command X-
  Bit 6 = Manual command Y+
  Bit 7 = Manual command Y-
Byte 3
  Bit 0 = Manual command Z+
  Bit 1 = Manual command Z-
  Bit 2 = Clear diagnostic counter I2C
  Bit 3 = Measure tool length
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5 Spindle speed in manual mode
Byte 6  G Code / M Code
Byte 7, 8, 9, 10 Wert 0 (with four decimal places)
Byte 11, 12, 13, 14 Value 1 (with four decimal places)
Byte 15, 16, 17, 18 Value 2 (with four decimal places)
Byte 19, 20, 21, 22 Value 3 (with four decimal places)
Byte 23, 24, 25, 26 Value 4 (with four decimal places)
Byte 27 Checksum (XOR Byte 0..26)

##### Slave --> I2C Master, Struct I2C send, max. 32 Byte possible
Teensy → Raspberry, Byte 0..22, 23 Byte
Byte 0
  Bit 0 = OperateOn
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = CommandDone
  Bit 1 = ReadyForCommand
  Bit 2 = ProgramDone
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Reference sensor X
  Bit 1 = Reference sensor Y
  Bit 2 = Reference sensor Z
  Bit 3 = Emergency stop button
  Bit 4 = Spindle on
  Bit 5 = Coolant on
  Bit 6 = Clamp on
  Bit 7 = Light on
Byte 3
  Bit 0 = Tool length sensor
  Bit 1 = Measure tool length done
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5, 6, 7 Actual position motor 0 (with four decimal places)
Byte 8, 9, 10, 11 Actual position motor 1 (with four decimal places)
Byte 12, 13, 14, 15 Actual position motor 2 (with four decimal places)
Byte 16, 17, 18, 19 Actual position motor 3 (with four decimal places)
Byte 20, 21 ActualStepProgram
Byte 22 Checksum (XOR Byte 0..21)
*/

// ###############################################################################################################
// ###############################################################################################################
// ##### Include
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include <LiquidCrystal_I2C.h> // For LCD display HD44780 2004 LCD, 4x20 characters

// ##### Create an IntervalTimer object 
IntervalTimer IntervalTimer0;

// ##### Configure LCD display, address 0x27 (39), 4x20 Zeichen
LiquidCrystal_I2C lcd(0x27,20,4);

// ##### Definitions
#define AmountOfMaxSteppermotors 4 // Must be 4
#define AmountOfSteppermotors 3 // Active stepermotors, must be 4 or less
#define SLAVE_ADDRESS 0x08 // I2C slave, communication to Raspberry PI,  address 8 (0x08)
#define AmountOfStepData 7500 // Arraysize StepData, 7500 = 84% of memory Teensy
#define DEBUGMODE false // Print diagnostic values to serial monitor
#define DEBUGMODESTEP true // Print step no. to serial monitor
char NameAxis[4][2] =  {{"X"}, {"Y"}, {"Z"}, {"-"}}; // Name of the axis

// Timer interrupt in us
// 250 = 250us
// 125
// 62.5
// 31.125
// 15.5625
#define TimeInterrupt 15.5625

// Hardware mechanic
const double StepsPerTurn = 400.0;
const double SpindlePitch = 3.0; // 3.0 mm
const double MM_Step = 0.0075; // mm per step
const double ToolLengthSensor = 32.2; // Switching point of tool length sensor in mm

// Speeds axis X,Y,Z
const double SpeedG00 = 2400.0; // Speed rapid movement in mm/min, 2400.0mm/min = 40mm/s
const double SpeedG74 = 1200.0; // Speed reference run in mm/min, 1200.0mm/min = 20mm/s
const double SpeedInching = 150.0; // Speed inching in mm/min, 150.0mm/min = 2.5mm/s
const double SpeedMeasureToolLength = 300.0; // Speed measure tool length in mm/min, 300.0mm/min = 5mm/s
const double SpeedMeasureToolLengthUpToReferenceSensor = 450.0; // Speed measure tool length in mm/min, 450.0mm/min = 7.5mm/s

// Speed spindle motor
#define SpeedSpindleMin 5000.0
#define SpeedSpindleMax 25000.0
// PWM spindle motor 0.1Khz-20Khz = 0-10V
#define SpindleSpeedMinHz 100.0   // 100.0Hz = 0.1Khz;
#define SpindleSpeedMaxHz 21420.0 // 20000.0Hz = 20Khz (15.5625us on, 31.125us off = 46.6875us period length = 21419Hz)

// Pi, DEG, RAD
const double Pi = 3.1415926535897932384626433832795;
const double Deg_To_Rad = 0.017453292519943295769236907684886; // 1/ 180/PI
const double Rad_To_Deg = 57.295779513082320876798154814105; // 180/PI

// Direction motor
// true = Output direction = 1, motor moves in positive direction
const bool Dir[4] = {false, true, true, true}; // X, Y, Z, NA

// Reference sensors, light switching
// true = Input reference sensor = 1 when reference position reached
const bool ReferenceSensorLightSwitching[4] = {true, true, true, true}; // X, Y, Z, NA

// Direction Reference sensor
// true = Reference sensor is mounted in pos. direction
const bool DirPos[4] = {true, true, true, true}; // X, Y, Z, NA

// Delaytimes in cycles
// Counter variables are unsigned int. The Due stores a 4 byte (32-bit) value, ranging from 0 to 4,294,967,295 (2^32)
// TimeInterrupt = 31.125
// 32128 * 31.125us = 999984us = 999.9ms
// 16064 * 31.125us = 499992us = 499.9ms
// 6425 * 31.125us = 199978us = 199.9ms
// 3212 * 31.125us = 99973.5us = 99.9ms
// 1606 * 31.125us = 50030.1us = 50.0ms
// 803 * 31.125us = 24993.3us = 24.9ms
// 401 * 31.125us = 12481.1us = 12.4ms
// 32 * 31.125us = 996us = 0.996ms

// TimeInterrupt = 15.5625
// 128512* 15.5625us = 1999968us = 1999.9ms
// 64256 * 15.5625us = 999984us = 999.9ms
// 32128 * 15.5625us = 499992us = 499.9ms
// 12850 * 15.5625us = 199978us = 199.9ms
// 3212 * 15.5625us = 49986us = 49.9ms
// 64 * 15.5625us = 996us = 0.996ms

#define WaitEnableStepAfterEnableOn 64256 // 1000ms Enable step after enable on
#define WaitEnableOffAfterOperateOff 64256 // 1000ms Enable off after operate off
#define WaitEnableOnAfterOperateOff 64256 // 1000ms Enable on after operate off
#define WaitMoveCommand 64 // 1ms Wait each move command before first step (to set direction output first)
#define WaitPositioning 3212 // 50ms Wait between each step run reference sensor free / go to reference sensor
#define WaitForSpindleOnAfterImmediateStop 128512 // 2000ms Wait for spindle on after immediate stop

// Define steps ramp, depend on speed
// Actual Speed >=200  and <= 2000, Steps Ramp = 0..100
// Actual Speed < 200, Steps Ramp = 0
// Actual Speed > 2000, Steps Ramp = 100
// Y = m*x+b
#define YMXB_MinInput 200.0
#define YMXB_MaxInput 2000.0
#define YMXB_MinResult 0.0
#define YMXB_MaxResult 100.0

// Switch / case step no. motor
#define StepNoAction 0
#define StepPositioningMain 10
#define StepLinearMovement 20
#define StepCircularMovement 30
#define StepDwellTime 40
#define StepInching 50
#define StepMeasureToolLength 60
#define StepHandshakeCommandDone 100

// Switch / case step no. positioning
#define StepPositioningRunReferenceSensorFreeFast 5
#define StepPositioningFindReferenceSensor 10
#define StepPositioningRunReferenceSensorFree 20
#define StepPositioningGotoReferenceSensor 30

// Switch / case step no. measure tool length
#define StepMeasureToolLengthRunSensorFreeFast 5
#define StepMeasureToolLengthFindSensor 10
#define StepMeasureToolLengthRunSensorFree 20
#define StepMeasureToolLengthGotoSensor 30
#define StepMeasureToolLengthUpFast 40

// G-Code / M-Code
#define CodeG00 100   // 100  G00 Eilgang
#define CodeG01 1     // 1    G01 Geradeninterpolation
#define CodeG02 2     // 2    G02 Kreisinterpolation, im Uhrzeigersinn
#define CodeG03 3     // 3    G03 Kreisinterpolation, gegen Uhrzeigersinn
#define CodeG04 4     // 4    G04 Verweilzeit in Sekunden
#define CodeG52 52    // 52   G52 Koordinatensystem Verschiebung
#define CodeG54 54    // 54   G54 Nullpunktverschiebung auf Werkstücknullpunkt oder Programmnullpunkt
#define CodeG68 68    // 68   G68 Coordinate rotation
#define CodeG69 69    // 69   G69 Cancel coordinate rotation
#define CodeG74 74    // 74   G74 Referenzfahrt über Referenzfühler anfahren
#define CodeG90 90    // 90   G90 Absolutmaßprogrammierung
#define CodeG91 91    // 91   G91 Kettenmaßprogrammierung
#define CodeM02 202   // 202  M02 Programmende
#define CodeM03 203   // 203  M03 Spindle on, clockwise
#define CodeM05 205   // 205  M05 Spindle off
#define CodeM08 208   // 208  M08 Coolant on
#define CodeM09 209   // 209  M09 Coolant off
#define CodeM10 210   // 210  M10 Clamp on
#define CodeM11 211   // 211  M11 Clamp off
#define CodeM35 235   // 235  M35 Light on
#define CodeM36 236   // 236  M36 Light off
#define CodeM30 230   // 230  M30 Programmende mit Ruecksprung auf Programmanfang
#define CodeInching 250 // 250 Inching
#define CodeMeasureToolLength 251 // 251 Measure tool length
#define CodeLastStep 252  // 252 Last step in steptransfer
#define CodeM98 253   // 253 M98 Subprogram call
#define CodeM99 254   // 254 M99 Subprogram end
#define CodeO 255   // 255 O Subprogram number

// ##### Forward declaration functions
void SendDataI2C();
void ReciveDataI2C(int);
void Step(int);
long YMXB(double);
unsigned long SpeedMM_Min_ProgramScans(double);
void Positioning(int, bool, unsigned long, bool);

// ##### Global variables for interruptroutine
// A global variable must be defined as volatile, if it is used in interrupts
volatile int timer = 0;
volatile bool bReciveDataI2CIsActive = false;
volatile uint8_t PosOrder[4][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};
volatile uint8_t PosAmount[4] = {0, 0, 0, 0};
volatile int CounterChecksumErrorI2CRecive = 0;
volatile bool FirstCycleMovement = false;
volatile int IndexStepData = 0;
volatile int IndexActualStepNo = 0;
volatile int AmountOfStepsMain = 0;
volatile bool ToggleOutputPWMSpindleSpeed = false;
volatile double dCounterOutputPWMSpindleSpeed = 0;
volatile double dActualHzPWMSpindleSpeed = SpindleSpeedMinHz;
volatile bool ReciveSubprogram = false;
volatile int IndexSubStepData = 0;
volatile int IndexSubStepNo = 0;

// ##### Testvariables
volatile int INTTestValue[10];
volatile long LongTestValue[10];
volatile double DoubleTestValue[10];

// Struct I2C receive
// Raspberry → Teensy, Byte 0..27, 28 Byte
typedef struct
{
  bool Operate;             // Byte 0, bit 0
  bool Reserve0_1;          // Bit 1
  bool Reserve0_2;          // Bit 2
  bool Reserve0_3;          // Bit 3
  bool Reserve0_4;          // Bit 4
  bool Reserve0_5;          // Bit 5
  bool Reserve0_6;          // Bit 6
  bool Reserve0_7;          // Bit 7
  bool ActivateCommand;     // Byte 1, bit 0
  bool Reserve1_1;           // Bit 1
  bool Reserve1_2;          // Bit 2
  bool Reserve1_3;          // Bit 3
  bool Reserve1_4;          // Bit 4
  bool Reserve1_5;          // Bit 5
  bool Reserve1_6;          // Bit 6
  bool Reserve1_7;          // Bit 7
  bool SpindleOn;           // Byte 2, bit 0
  bool CoolantOn;           // Bit 1
  bool ClampOn;             // Bit 2
  bool LightOn;             // Bit 3
  bool XPlus;               // Bit 4
  bool XMinus;              // Bit 5
  bool YPlus;               // Bit 6
  bool YMinus;              // Bit 7
  bool ZPlus;               // Byte 3, bit 0
  bool ZMinus;              // Bit 1
  bool ClearDiagnosticCounterI2C; // Bit 2
  bool MeasureToolLength;   // Bit 3
  bool Reserve3_4;          // Bit 4
  bool Reserve3_5;          // Bit 5
  bool Reserve3_6;          // Bit 6
  bool Reserve3_7;          // Bit 7
  uint16_t SpindleSpeedManualMode; // Byte 4 + 5
  uint8_t Code;             // Byte 6
  double dValue[5];         // Byte 7 + 8 + 9 + 10
                            // Byte 11 + 12 + 13 + 14
                            // Byte 15 + 16 + 17 + 18
                            // Byte 19 + 20 + 21 + 22
                            // Byte 23 + 24 + 25 + 26
  uint8_t Checksum;         // Byte 27
} MotorStructRecive;
volatile MotorStructRecive MotorRecive = {false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          0, 0, 0, 0, 0, 0, 0, 0};
volatile MotorStructRecive MotorReciveCopy = {false, false, false, false, false, false, false, false,
                                              false, false, false, false, false, false, false, false,
                                              false, false, false, false, false, false, false, false,
                                              false, false, false, false, false, false, false, false,
                                              0, 0, 0, 0, 0, 0, 0, 0};

// Struct I2C send
// Teensy → Raspberry, Byte 0..22, 23 Byte
typedef struct
{
  bool OperateOn;             // Byte 0, bit 0
  bool Reserve0_1;            // Bit 1
  bool Reserve0_2;            // Bit 2
  bool Reserve0_3;            // Bit 3
  bool Reserve0_4;            // Bit 4
  bool Reserve0_5;            // Bit 5
  bool Reserve0_6;            // Bit 6
  bool Reserve0_7;            // Bit 7
  bool CommandDone;           // Byte 1, bit 0
  bool ReadyForCommand;       // Bit 1
  bool ProgramDone;           // Bit 2
  bool Reserve1_3;            // Bit 3
  bool Reserve1_4;            // Bit 4
  bool Reserve1_5;            // Bit 5
  bool Reserve1_6;            // Bit 6
  bool Reserve1_7;            // Bit 7
  bool ReferenceSensorX;      // Byte 2, bit 0
  bool ReferenceSensorY;      // Bit 1
  bool ReferenceSensorZ;      // Bit 2
  bool EmergencyStopButton;   // Bit 3
  bool SpindleOn;             // Bit 4
  bool CoolantOn;             // Bit 5
  bool ClampOn;               // Bit 6
  bool LightOn;               // Bit 7
  bool ToolLengthSensor;      // Byte 3, bit 0
  bool MeasureToolLengthDone; // Bit 1
  bool Reserve3_2;            // Bit 2
  bool Reserve3_3;            // Bit 3
  bool Reserve3_4;            // Bit 4
  bool Reserve3_5;            // Bit 5
  bool Reserve3_6;            // Bit 6
  bool Reserve3_7;            // Bit 7
  double ActPosMotor[4];      // Byte 4 + 5 + 6 + 7
                              // Byte 8 + 9 + 10 + 11
                              // Byte 12 + 13 + 14 + 15
                              // Byte 16 + 17 + 18 + 19
  uint16_t ActualStepProgram; // Byte 20 + 21
  uint8_t Checksum;           // Byte 22
} MotorStructSend;
volatile MotorStructSend MotorSend = {false, false, false, false, false, false, false, false,
                                      false, false, false, false, false, false, false, false,
                                      false, false, false, false, false, false, false, false,
                                      false, false, false, false, false, false, false, false,
                                      0, 0, 0, 0, 0, 0};
volatile MotorStructSend MotorSendCopy = {false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          false, false, false, false, false, false, false, false,
                                          0, 0, 0, 0, 0, 0};

// Struct Step data
typedef struct
{
  uint8_t Code;
  double dValue[5];
} StructStepData;
volatile StructStepData StepData[AmountOfStepData];
volatile StructStepData ActualStepData;
volatile StructStepData *SubProgramData; // Memory allocation in RAM2 for subprogram

// Struct Subprogram, provided for max. 100 subroutines with a nesting depth of 8
// Step data for the subroutines are stored in the array "SubProgramData"
#define MaxSubprograms 100
#define MaxNestingSubprogram 8
typedef struct
{
  bool IsActive; // Subprogram is active
  int IndexActualStepNo; // Actual step no. of the active subprogram number in the Array SubProgramData[7500]
  int IndexActiveSubprogram; // Index of the actual subprogram no. 0..MaxSubprograms
  int ActiveProgramNo; // Program no. of the subprogram, parameter P (M98 P1000 L5)
  int AdressPointerFirstStep[MaxSubprograms]; // Loaded during program transfer. Adress pointer in the Array SubProgramData[7500] where the subprogram starts (nesting)
  int ProgramNo[MaxSubprograms]; // Loaded during program transfer. Program no. of the subprogram, parameter P (M98 P1000 L5)
  int CounterNumberOfRepeats[MaxNestingSubprogram]; // Number of repeats, parameter L (M98 P1000 L5)
  int AdressPointerStepBack[MaxNestingSubprogram]; // Adress pointer in the Array SubProgramData[7500] step back when subprogram ends (nesting)
  int ActualRepeats[MaxNestingSubprogram]; // Actual repeats of the subprogram
  int AdressPointerFirstStepRepeats[MaxNestingSubprogram];
} StructSubprogramData;
volatile StructSubprogramData Subprogram;

// Struct Motor common
typedef struct
{
  bool StepsPositiveIsActive;
  bool StepsNegativeIsActive;
  bool RotationPositiveIsActive;
  bool RotationNegativeIsActive;
  bool RotationPositiveWasActive;
  bool RotationNegativeWasActive;
  bool PositioningIsActive; // benoetigt
  bool PositioningDone; // benoetigt
  bool ReferenceSensor; // benoetigt
  bool DirectionPositivePositioning; // benoetigt
  bool DelaytimeMoveCommand; // benoetigt
  bool ResetStep;
  uint8_t DigitalOutputMotor; // benoetigt
  uint8_t StepNoPositioning; // benoetigt
  unsigned long CounterDelaytimeMoveCommand; // 0 to 4,294,967,295 (2^32)                 benötigt
  unsigned long ValueDelaytimeMoveCommand; // 0 to 4,294,967,295 (2^32)                   benötigt
  unsigned long lCounterRamp; // benoetigt
  double dAbsolutePosMotor; // benoetigt
  double dRequiredPosAbsolute;
  double dAbsolutePosFromMachineZeroPoint;
} MotorStructCommon;
volatile MotorStructCommon MotorCommon[AmountOfMaxSteppermotors] = {
  {false, false, false, false, false, false, false, false, false, false, false, false, 0b00000000, 0, 0, 0, 0, 0.0, 0.0, 0.0},
  {false, false, false, false, false, false, false, false, false, false, false, false, 0b00000000, 0, 0, 0, 0, 0.0, 0.0, 0.0},
  {false, false, false, false, false, false, false, false, false, false, false, false, 0b00000000, 0, 0, 0, 0, 0.0, 0.0, 0.0},
  {false, false, false, false, false, false, false, false, false, false, false, false, 0b00000000, 0, 0, 0, 0, 0.0, 0.0, 0.0}
};

// Struct Motor all
typedef struct
{
  bool EmergencyStop;
  bool Enable;
  bool EnableStep;
  bool AbsoluteIsActive; // Default = true
  bool RelativeIsActive; // Default = false
  bool PositioningIsActive;
  bool Startramp;
  bool Linear;
  bool Stopramp;
  bool MovementWithoutRamp;
  bool DoStep;
  bool DoStepInchingX;
  bool DoStepInchingY;
  bool DoStepInchingZ;
  bool G02IsActive;
  bool G03IsActive;
  bool SpindleOn;         
  bool CoolantOn;
  bool ClampOn;
  bool LightOn;
  bool ToolLengthSensor;
  bool CommandActive;
  bool bInchingStarted;
  bool bInchingXPlusActive;
  bool bInchingXMinusActive;
  bool bInchingYPlusActive;
  bool bInchingYMinusActive;
  bool bInchingZPlusActive;
  bool bInchingZMinusActive;
  bool MeasureToolLengthStarted;
  bool MeasureToolLengthDone;
  bool SpindleWasOnStopMovement;
  bool RotationG68IsActive;
  uint8_t StepNo;
  uint8_t StepNoEnable;
  uint8_t StepPositioningStepOrder;
  uint8_t StepNoCommMaster;
  uint8_t StepNoInchingX;
  uint8_t StepNoInchingY;
  uint8_t StepNoInchingZ;
  uint8_t StepNoMeasureToolLength;
  uint8_t StepNoStopMovement;
  uint8_t StepNoStopSpindle;
  unsigned long CounterEnableStepAfterEnableOn; // 0 to 4,294,967,295 (2^32)
  unsigned long CounterEnableOffAfterOperateOff; // 0 to 4,294,967,295 (2^32)
  unsigned long CounterEnableOnAfterOperateOff; // 0 to 4,294,967,295 (2^32)
  double dStartpositionX;
  double dStartpositionY;
  double dStartpositionZ;
  double dEndpositionX;
  double dEndpositionY;
  double dEndpositionZ;
  double dDistanceX;
  double dDistanceY;
  double dDistanceZ;
  double dDistanceAbsX;
  double dDistanceAbsY;
  double dDistanceAbsZ;
  double dAmountStepsX;
  double dAmountStepsY;
  double dAmountStepsZ;
  double dAmountStepsAbsX;
  double dAmountStepsAbsY;
  double dAmountStepsAbsZ;
  double dAmountStepsMax;
  double dDistanceAbsPerStepX;
  double dDistanceAbsPerStepY;
  double dDistanceAbsPerStepZ;
  double dDistancePerStepX;
  double dDistancePerStepY;
  double dDistancePerStepZ;
  double dRequiredPosX;
  double dRequiredPosY;
  double dRequiredPosZ;
  double dInternalDistancecounterX;
  double dInternalDistancecounterY;
  double dInternalDistancecounterZ;
  double dRadius;
  double CenterCircleX;
  double CenterCircleY;
  double dStartAngle;
  double dEndAngle;
  double dDistanceAngle;
  double dAngleStep;
  double dInternalAnglecounter;
  double dActualAngle;
  double dDistanceAngleRamp;
  double dStartrampEndAngle;
  double dStoprampStartAngle;
  double dLastSpeed;
  double dActualSpeed;
  double dMaxStepRamp;
  double dRequiredSpindleSpeed;
  double RotationG68CX;
  double RotationG68CY;
  double RotationG68Angle;
  double LastRequiredPositionX;
  double LastRequiredPositionY;
  unsigned long lAmountSteps; // 0 to 4,294,967,295 (2^32)
  unsigned long lStepcounter; // 0 to 4,294,967,295 (2^32)
  unsigned long lAmountStepsRamp; // 0 to 4,294,967,295 (2^32)
  unsigned long lCounterRamp; // 0 to 4,294,967,295 (2^32)
  unsigned long lCounterStepsStopMovement; // 0 to 4,294,967,295 (2^32)
  unsigned long lCounterRampInchingX; // 0 to 4,294,967,295 (2^32)
  unsigned long lCounterRampInchingY; // 0 to 4,294,967,295 (2^32)
  unsigned long lCounterRampInchingZ; // 0 to 4,294,967,295 (2^32)
  unsigned long lActualValueRamp; // 0 to 4,294,967,295 (2^32)
  unsigned long lMaxStepRamp; // 0 to 4,294,967,295 (2^32)
  unsigned long lSpeedInProgramscansPerStep; // 0 to 4,294,967,295 (2^32)
  unsigned long lDwellTimeCycles; // 0 to 4,294,967,295 (2^32)
  unsigned long lDwellTimeCounter; // 0 to 4,294,967,295 (2^32)
} MotorStructAll;
volatile MotorStructAll MotorAll = {false, false, false, true, false, false, false, false,
                                    false, false, false, false, false, false, false, false,
                                    false, false, false, false, false, false, false, false,
                                    false, false, false, false, false, false, false, false,
                                    false,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Struct Debugvalues
typedef struct
{
  int StepsInStartramp;
  int StepsInLinear;
  int StepsInStopramp;
} StructDebugValues;
volatile StructDebugValues DebugValues = {0, 0, 0};

// ##### Inputs / Outputs Teensy
// Pin 13 = LED_BUILTIN
/*
Pin Used for
13  LED buildin
14  Ref. 1 Y-Axis (Input)
15  Ref. 2 Z-Axis (Input)
16  SCL1 Raspberry
17  SDA1 Raspberry
18  SDA0 Liquid Christal
19  SCL0 Liquid Christal
20  Ref. 0 X-Axis (Input)
21  Emergency stop (Input Pullup)
22  PWM Spindle Speed (Output)
23  Tool length sensor (Input Pullup)
3.3V Output 250mA
GND 0V Power supply
Vin 5V Power supply

12  Enable 0 (Output)
11  Step 0 (Output)
10  Dir 0 (Output)
9   Enable 1 (Output)
8   Step 1 (Output)
7   Dir 1 (Output)
6   Enable 2 (Output)
5   Step 2 (Output)
4   Dir 2 (Output)
3   Spindle on (Output)
2   Coolant on (Output)
1   Clamp on (Output)
0   Light on (Output)
GND 0V (Not connected)
*/

#define DigitalInputMotor1ReferenceSensor 14
#define DigitalInputMotor2ReferenceSensor 15
#define DigitalInputMotor0ReferenceSensor 20
#define DigitalInputEmergencyStop 21
#define DigitalOutputPWMSpindleSpeed 22
#define DigitalInputToolLengthSensor 23

#define DigitalOutputMotor0Enable 12
#define DigitalOutputMotor0Step 11
#define DigitalOutputMotor0Dir 10
#define DigitalOutputMotor1Enable 9
#define DigitalOutputMotor1Step 8
#define DigitalOutputMotor1Dir 7
#define DigitalOutputMotor2Enable 6
#define DigitalOutputMotor2Step 5
#define DigitalOutputMotor2Dir 4
#define DigitalOutputSpindleOn 3
#define DigitalOutputCoolantOn 2
#define DigitalOutputClampOn 1
#define DigitalOutputLightOn 0

// ##### Function (event) send data to I2C master
/*
Teensy → Raspberry, Byte 0..22, 23 Byte
Byte 0
  Bit 0 = OperateOn
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = CommandDone
  Bit 1 = ReadyForCommand
  Bit 2 = ProgramDone
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Reference sensor X
  Bit 1 = Reference sensor Y
  Bit 2 = Reference sensor Z
  Bit 3 = Emergency stop button
  Bit 4 = Spindle on
  Bit 5 = Coolant on
  Bit 6 = Clamp on
  Bit 7 = Light on
Byte 3
  Bit 0 = Tool length sensor
  Bit 1 = Measure tool length done
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5, 6, 7 Actual position motor 0 (with four decimal places)
Byte 8, 9, 10, 11 Actual position motor 1 (with four decimal places)
Byte 12, 13, 14, 15 Actual position motor 2 (with four decimal places)
Byte 16, 17, 18, 19 Actual position motor 3 (with four decimal places)
Byte 20, 21 ActualStepProgram
Byte 22 Checksum (XOR Byte 0..21)
*/

void SendDataI2C()
{

  // ##### Local variables
  int id;
  unsigned char t[23] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Empty array where to put the data going to the master
  long lTmp;

  // ##### Write byte 0 bit data
  id = 0;
  // Byte 0
  t[id] = 0; // Clear send byte 0
  if (MotorSendCopy.OperateOn == true)
  {
    t[id] |= 1 << 0; // Set bit 0
  }
  if (MotorSendCopy.Reserve0_1 == true)
  {
    t[id] |= 1 << 1; // Set bit 1
  }
  if (MotorSendCopy.Reserve0_2 == true)
  {
    t[id] |= 1 << 2; // Set bit 2
  }
  if (MotorSendCopy.Reserve0_3 == true)
  {
    t[id] |= 1 << 3; // Set bit 3
  }
  if (MotorSendCopy.Reserve0_4 == true)
  {
    t[id] |= 1 << 4; // Set bit 4
  }
  if (MotorSendCopy.Reserve0_5 == true)
  {
    t[id] |= 1 << 5; // Set bit 5
  }
  if (MotorSendCopy.Reserve0_6 == true)
  {
    t[id] |= 1 << 6; // Set bit 6
  }
  if (MotorSendCopy.Reserve0_7 == true)
  {
    t[id] |= 1 << 7; // Set bit 7
  }

  // ##### Write byte 1 bit data
  // increment byte no.
  id++;
  // Byte 1
  t[id] = 0; // Clear send byte 1
  if (MotorSendCopy.CommandDone == true)
  {
    t[id] |= 1 << 0; // Set bit 0
  }
  if (MotorSendCopy.ReadyForCommand == true)
  {
    t[id] |= 1 << 1; // Set bit 1
  }
  if (MotorSendCopy.ProgramDone == true)
  {
    t[id] |= 1 << 2; // Set bit 2
  }
  if (MotorSendCopy.Reserve1_3 == true)
  {
    t[id] |= 1 << 3; // Set bit 3
  }
  if (MotorSendCopy.Reserve1_4 == true)
  {
    t[id] |= 1 << 4; // Set bit 4
  }
  if (MotorSendCopy.Reserve1_5 == true)
  {
    t[id] |= 1 << 5; // Set bit 5
  }
  if (MotorSendCopy.Reserve1_6 == true)
  {
    t[id] |= 1 << 6; // Set bit 6
  }
  if (MotorSendCopy.Reserve1_7 == true)
  {
    t[id] |= 1 << 7; // Set bit 7
  }

  // ##### Write byte 2 bit data
  // increment byte no.
  id++;
  // Byte 2
  t[id] = 0; // Clear send byte 2
  if (MotorSendCopy.ReferenceSensorX == true)
  {
    t[id] |= 1 << 0; // Set bit 0
  }
  if (MotorSendCopy.ReferenceSensorY == true)
  {
    t[id] |= 1 << 1; // Set bit 1
  }
  if (MotorSendCopy.ReferenceSensorZ == true)
  {
    t[id] |= 1 << 2; // Set bit 2
  }
  if (MotorSendCopy.EmergencyStopButton == true)
  {
    t[id] |= 1 << 3; // Set bit 3
  }
  if (MotorSendCopy.SpindleOn == true)
  {
    t[id] |= 1 << 4; // Set bit 4
  }
  if (MotorSendCopy.CoolantOn == true)
  {
    t[id] |= 1 << 5; // Set bit 5
  }
  if (MotorSendCopy.ClampOn == true)
  {
    t[id] |= 1 << 6; // Set bit 6
  }
  if (MotorSendCopy.LightOn == true)
  {
    t[id] |= 1 << 7; // Set bit 7
  }

  // ##### Write byte 3 bit data
  // increment byte no.
  id++;
  // Byte 3
  t[id] = 0; // Clear send byte 3
  if (MotorSendCopy.ToolLengthSensor == true)
  {
    t[id] |= 1 << 0; // Set bit 0
  }
  if (MotorSendCopy.MeasureToolLengthDone == true)
  {
    t[id] |= 1 << 1; // Set bit 1
  }
  if (MotorSendCopy.Reserve3_2 == true)
  {
    t[id] |= 1 << 2; // Set bit 2
  }
  if (MotorSendCopy.Reserve3_3 == true)
  {
    t[id] |= 1 << 3; // Set bit 3
  }
  if (MotorSendCopy.Reserve3_4 == true)
  {
    t[id] |= 1 << 4; // Set bit 4
  }
  if (MotorSendCopy.Reserve3_5 == true)
  {
    t[id] |= 1 << 5; // Set bit 5
  }
  if (MotorSendCopy.Reserve3_6 == true)
  {
    t[id] |= 1 << 6; // Set bit 6
  }
  if (MotorSendCopy.Reserve3_7 == true)
  {
    t[id] |= 1 << 7; // Set bit 7
  }

  // Byte 4, 5, 6, 7   Actual position motor 0 (mit vier Nachkommastellen)
  // increment byte no.
  id++;
  lTmp = long(MotorSendCopy.ActPosMotor[0] * 10000.0); // explizite Typumwandlung von double in long
  t[id] = char((lTmp & 0xff000000) >> 24); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x00ff0000) >> 16); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x0000ff00) >> 8); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char(lTmp & 0x000000ff); // Bitwise AND has lower priority than >>, therefore () around &

  // Byte 8, 9, 10, 11 Actual position motor 1 (mit vier Nachkommastellen)
  // increment byte no.
  id++;
  lTmp = long(MotorSendCopy.ActPosMotor[1] * 10000.0); // explizite Typumwandlung von double in long
  t[id] = char((lTmp & 0xff000000) >> 24); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x00ff0000) >> 16); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x0000ff00) >> 8); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char(lTmp & 0x000000ff); // Bitwise AND has lower priority than >>, therefore () around &

  // Byte 12, 13, 14, 15 Actual position motor 2 (mit vier Nachkommastellen)
  // increment byte no.
  id++;
  lTmp = long(MotorSendCopy.ActPosMotor[2] * 10000.0); // explizite Typumwandlung von double in long
  t[id] = char((lTmp & 0xff000000) >> 24); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x00ff0000) >> 16); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x0000ff00) >> 8); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char(lTmp & 0x000000ff); // Bitwise AND has lower priority than >>, therefore () around &

  // Byte 16, 17, 18, 19 Actual position motor 3 (mit vier Nachkommastellen)
  // increment byte no.
  id++;
  lTmp = long(MotorSendCopy.ActPosMotor[3] * 10000.0); // explizite Typumwandlung von double in long
  t[id] = char((lTmp & 0xff000000) >> 24); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x00ff0000) >> 16); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char((lTmp & 0x0000ff00) >> 8); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char(lTmp & 0x000000ff); // Bitwise AND has lower priority than >>, therefore () around &

  // Byte 20, 21 ActualStepProgram
  // increment byte no.
  id++;
  t[id] = char((MotorSendCopy.ActualStepProgram & 0xff00) >> 8); // Bitwise AND has lower priority than >>, therefore () around &
  id++;
  t[id] = char(MotorSendCopy.ActualStepProgram & 0x00ff); // Bitwise AND has lower priority than >>, therefore () around &

  // Byte 22 Checksum (XOR Byte 0..21)
  // increment byte no.
  id++;
  MotorSend.Checksum = t[0] xor t[1] xor t[2] xor t[3] xor t[4] xor t[5] xor t[6] xor t[7] xor t[8] xor t[9] xor t[10]
                       xor t[11] xor t[12] xor t[13] xor t[14] xor t[15] xor t[16] xor t[17] xor t[18] xor t[19] xor t[20] xor t[21];
  t[id] = MotorSend.Checksum;

  // ##### I2C send data. Respond with message of 23 bytes as expected by master
  Wire1.write(t, 23);
}

// ##### Function (event) receive data from I2C master
/*
Raspberry → Teensy, Byte 0..27, 28 Byte
Byte 0
  Bit 0 = Operate (0 - Stepper disabled 1 - Stepper enabled)
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 1
  Bit 0 = ActivateCommand
  Bit 1 = Reserve 1
  Bit 2 = Reserve 2
  Bit 3 = Reserve 3
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 2
  Bit 0 = Manual command Spindle on
  Bit 1 = Manual command Coolant on
  Bit 2 = Manual command Clamp on
  Bit 3 = Manual command Light on
  Bit 4 = Manual command X+
  Bit 5 = Manual command X-
  Bit 6 = Manual command Y+
  Bit 7 = Manual command Y-
Byte 3
  Bit 0 = Manual command Z+
  Bit 1 = Manual command Z-
  Bit 2 = Clear diagnostic counter I2C
  Bit 3 = Measure tool length
  Bit 4 = Reserve 4
  Bit 5 = Reserve 5
  Bit 6 = Reserve 6
  Bit 7 = Reserve 7
Byte 4, 5 Spindle speed in manual mode
Byte 6  G Code / M Code
Byte 7, 8, 9, 10 Wert 0 (with four decimal places)
Byte 11, 12, 13, 14 Value 1 (with four decimal places)
Byte 15, 16, 17, 18 Value 2 (with four decimal places)
Byte 19, 20, 21, 22 Value 3 (with four decimal places)
Byte 23, 24, 25, 26 Value 4 (with four decimal places)
Byte 27 Checksum (XOR Byte 0..26)
*/

void ReciveDataI2C(int byteCount)
{

  // ##### Set tag receive data I2C is active
  bReciveDataI2CIsActive = true;

  // ##### Local variables
  int i, id;
  long tmp0, tmp1, tmp2, tmp3;
  unsigned char t[29] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Empty array where to put the data coming from the master (Byte 0..27 + 1 for slave address = 29 byte)
  unsigned char Checksum[1] = {0};
  bool ChecksumPassed;

  // ##### I2C receive data. Expect 29 bytes or 1 byte from master
  // If there is a send request from master, this function is also called with 1 byte from master
  // Byte 0 = Slave address, in our case 8 (#define SLAVE_ADDRESS 0x08)
  // Byte 1..28 = Recive data

  // ##### Copy receive data only, when 29 bytes from master
  if (byteCount == 29) // byteCount = 29 or 1
  {

    // ##### Copy recieved data
    for (i = 0; i < byteCount; i++)
    {
      t[i] = Wire1.read();
    }

    // ##### Checksum (XOR Byte 0..26 -> Byte 1..27)
    // Attention: Starts with byte 1 !!! (Byte 0 = Slave address)
    Checksum[0] = t[1] xor t[2] xor t[3] xor t[4] xor t[5] xor t[6] xor t[7] xor t[8] xor t[9] xor t[10]
                  xor t[11] xor t[12] xor t[13] xor t[14] xor t[15] xor t[16] xor t[17] xor t[18] xor t[19] xor t[20]
                  xor t[21] xor t[22] xor t[23] xor t[24] xor t[25] xor t[26] xor t[27];

    ChecksumPassed = true;
    if (Checksum[0] != t[28])
    {
      ChecksumPassed = false;
      CounterChecksumErrorI2CRecive +=1;
    }

    // ##### Read byte data
    if (ChecksumPassed == true)
    {
      id = 1; // Attention: Starts with byte 1 !!! (Byte 0 = Slave address)
      // Byte 0, Bit data
      if ((t[id] & 0b00000001) != 0 )
      {
        MotorReciveCopy.Operate = true; // Set bit 0
      }
      else
      {
        MotorReciveCopy.Operate = false; // Clear bit 0
      }

      if ((t[id] & 0b00000010) != 0 )
      {
        MotorReciveCopy.Reserve0_1 = true; // Set bit 1
      }
      else
      {
        MotorReciveCopy.Reserve0_1 = false; // Clear bit 1
      }

      if ((t[id] & 0b00000100) != 0 )
      {
        MotorReciveCopy.Reserve0_2 = true; // Set bit 2
      }
      else
      {
        MotorReciveCopy.Reserve0_2 = false; // Clear bit 2
      }

      if ((t[id] & 0b00001000) != 0 )
      {
        MotorReciveCopy.Reserve0_3 = true; // Set bit 3
      }
      else
      {
        MotorReciveCopy.Reserve0_3 = false; // Clear bit 3
      }

      if ((t[id] & 0b00010000) != 0 )
      {
        MotorReciveCopy.Reserve0_4 = true; // Set bit 4
      }
      else
      {
        MotorReciveCopy.Reserve0_4 = false; // Clear bit 4
      }

      if ((t[id] & 0b00100000) != 0 )
      {
        MotorReciveCopy.Reserve0_5 = true; // Set bit 5
      }
      else
      {
        MotorReciveCopy.Reserve0_5 = false; // Clear bit 5
      }

      if ((t[id] & 0b01000000) != 0 )
      {
        MotorReciveCopy.Reserve0_6 = true; // Set bit 6
      }
      else
      {
        MotorReciveCopy.Reserve0_6 = false; // Clear bit 6
      }

      if ((t[id] & 0b10000000) != 0 )
      {
        MotorReciveCopy.Reserve0_7 = true; // Set bit 7
      }
      else
      {
        MotorReciveCopy.Reserve0_7 = false; // Clear bit 7
      }

      // Byte 1, Bit data
      // increment byte no.
      id++;
      if ((t[id] & 0b00000001) != 0 )
      {
        MotorReciveCopy.ActivateCommand = true; // Set bit 0
      }
      else
      {
        MotorReciveCopy.ActivateCommand = false; // Clear bit 0
      }

      if ((t[id] & 0b00000010) != 0 )
      {
        MotorReciveCopy.Reserve1_1 = true; // Set bit 1
      }
      else
      {
        MotorReciveCopy.Reserve1_1 = false; // Clear bit 1
      }

      if ((t[id] & 0b00000100) != 0 )
      {
        MotorReciveCopy.Reserve1_2 = true; // Set bit 2
      }
      else
      {
        MotorReciveCopy.Reserve1_2 = false; // Clear bit 2
      }

      if ((t[id] & 0b00001000) != 0 )
      {
        MotorReciveCopy.Reserve1_3 = true; // Set bit 3
      }
      else
      {
        MotorReciveCopy.Reserve1_3 = false; // Clear bit 3
      }

      if ((t[id] & 0b00010000) != 0 )
      {
        MotorReciveCopy.Reserve1_4 = true; // Set bit 4
      }
      else
      {
        MotorReciveCopy.Reserve1_4 = false; // Clear bit 4
      }

      if ((t[id] & 0b00100000) != 0 )
      {
        MotorReciveCopy.Reserve1_5 = true; // Set bit 5
      }
      else
      {
        MotorReciveCopy.Reserve1_5 = false; // Clear bit 5
      }

      if ((t[id] & 0b01000000) != 0 )
      {
        MotorReciveCopy.Reserve1_6 = true; // Set bit 6
      }
      else
      {
        MotorReciveCopy.Reserve1_6 = false; // Clear bit 6
      }

      if ((t[id] & 0b10000000) != 0 )
      {
        MotorReciveCopy.Reserve1_7 = true; // Set bit 7
      }
      else
      {
        MotorReciveCopy.Reserve1_7 = false; // Clear bit 7
      }

      // Byte 2, Bit data
      // increment byte no.
      id++;
      if ((t[id] & 0b00000001) != 0 )
      {
        MotorReciveCopy.SpindleOn = true; // Set bit 0
      }
      else
      {
        MotorReciveCopy.SpindleOn = false; // Clear bit 0
      }

      if ((t[id] & 0b00000010) != 0 )
      {
        MotorReciveCopy.CoolantOn = true; // Set bit 1
      }
      else
      {
        MotorReciveCopy.CoolantOn = false; // Clear bit 1
      }

      if ((t[id] & 0b00000100) != 0 )
      {
        MotorReciveCopy.ClampOn = true; // Set bit 2
      }
      else
      {
        MotorReciveCopy.ClampOn = false; // Clear bit 2
      }

      if ((t[id] & 0b00001000) != 0 )
      {
        MotorReciveCopy.LightOn = true; // Set bit 3
      }
      else
      {
        MotorReciveCopy.LightOn = false; // Clear bit 3
      }

      if ((t[id] & 0b00010000) != 0 )
      {
        MotorReciveCopy.XPlus = true; // Set bit 4
      }
      else
      {
        MotorReciveCopy.XPlus = false; // Clear bit 4
      }

      if ((t[id] & 0b00100000) != 0 )
      {
        MotorReciveCopy.XMinus = true; // Set bit 5
      }
      else
      {
        MotorReciveCopy.XMinus = false; // Clear bit 5
      }

      if ((t[id] & 0b01000000) != 0 )
      {
        MotorReciveCopy.YPlus = true; // Set bit 6
      }
      else
      {
        MotorReciveCopy.YPlus = false; // Clear bit 6
      }

      if ((t[id] & 0b10000000) != 0 )
      {
        MotorReciveCopy.YMinus = true; // Set bit 7
      }
      else
      {
        MotorReciveCopy.YMinus = false; // Clear bit 7
      }

      // Byte 3, Bit data
      // increment byte no.
      id++;
      if ((t[id] & 0b00000001) != 0 )
      {
        MotorReciveCopy.ZPlus = true; // Set bit 0
      }
      else
      {
        MotorReciveCopy.ZPlus = false; // Clear bit 0
      }

      if ((t[id] & 0b00000010) != 0 )
      {
        MotorReciveCopy.ZMinus = true; // Set bit 1
      }
      else
      {
        MotorReciveCopy.ZMinus = false; // Clear bit 1
      }

      if ((t[id] & 0b00000100) != 0 )
      {
        MotorReciveCopy.ClearDiagnosticCounterI2C = true; // Set bit 2
      }
      else
      {
        MotorReciveCopy.ClearDiagnosticCounterI2C = false; // Clear bit 2
      }

      if ((t[id] & 0b00001000) != 0 )
      {
        MotorReciveCopy.MeasureToolLength = true; // Set bit 3
      }
      else
      {
        MotorReciveCopy.MeasureToolLength = false; // Clear bit 3
      }

      if ((t[id] & 0b00010000) != 0 )
      {
        MotorReciveCopy.Reserve3_4 = true; // Set bit 4
      }
      else
      {
        MotorReciveCopy.Reserve3_4 = false; // Clear bit 4
      }

      if ((t[id] & 0b00100000) != 0 )
      {
        MotorReciveCopy.Reserve3_5 = true; // Set bit 5
      }
      else
      {
        MotorReciveCopy.Reserve3_5 = false; // Clear bit 5
      }

      if ((t[id] & 0b01000000) != 0 )
      {
        MotorReciveCopy.Reserve3_6 = true; // Set bit 6
      }
      else
      {
        MotorReciveCopy.Reserve3_6 = false; // Clear bit 6
      }

      if ((t[id] & 0b10000000) != 0 )
      {
        MotorReciveCopy.Reserve3_7 = true; // Set bit 7
      }
      else
      {
        MotorReciveCopy.Reserve3_7 = false; // Clear bit 7
      }

      // Byte 4, 5 Spindle speed in manual mode
      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
     
      // MSB (High byte 1)
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 8;

      tmp0 = tmp0 | tmp1;
      MotorReciveCopy.SpindleSpeedManualMode = tmp0;

      // Byte 4, G Code / M Code
      // increment byte no.
      id++;
      MotorReciveCopy.Code = t[id];

      // Byte 5, 6, 7, 8 Wert 0 (mit vier Nachkommastellen)
      // MSB (High byte 3), Byte 2, Byte 1, LSB (Low byte 0), Wertebereich -2,147,483,648 to +2,147,483,647
      // MSB (High byte 3)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
      tmp0 = tmp0 << 24;

      // Byte 2
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 16;

      // Byte 1
      // increment byte no.
      id++;
      tmp2 = 0;
      tmp2 = long(t[id]);
      tmp2 = tmp2 << 8;

      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp3 = 0;
      tmp3 = long(t[id]);

      tmp0 = tmp0 | tmp1 | tmp2 | tmp3;
      MotorReciveCopy.dValue[0] = double(tmp0) / 10000.0; // explizite Typumwandlung von long in double

      // Byte 9, 10, 11, 12 Wert 1 (mit vier Nachkommastellen)
      // MSB (High byte 3), Byte 2, Byte 1, LSB (Low byte 0), Wertebereich -2,147,483,648 to +2,147,483,647
      // MSB (High byte 3)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
      tmp0 = tmp0 << 24;

      // Byte 2
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 16;

      // Byte 1
      // increment byte no.
      id++;
      tmp2 = 0;
      tmp2 = long(t[id]);
      tmp2 = tmp2 << 8;

      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp3 = 0;
      tmp3 = long(t[id]);

      tmp0 = tmp0 | tmp1 | tmp2 | tmp3;
      MotorReciveCopy.dValue[1] = double(tmp0) / 10000.0; // explizite Typumwandlung von long in double
            
      // Byte 13, 14, 15, 16 Wert 2 (mit vier Nachkommastellen)
      // MSB (High byte 3), Byte 2, Byte 1, LSB (Low byte 0), Wertebereich -2,147,483,648 to +2,147,483,647
      // MSB (High byte 3)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
      tmp0 = tmp0 << 24;

      // Byte 2
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 16;

      // Byte 1
      // increment byte no.
      id++;
      tmp2 = 0;
      tmp2 = long(t[id]);
      tmp2 = tmp2 << 8;

      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp3 = 0;
      tmp3 = long(t[id]);

      tmp0 = tmp0 | tmp1 | tmp2 | tmp3;
      MotorReciveCopy.dValue[2] = double(tmp0) / 10000.0; // explizite Typumwandlung von long in double
            
      // Byte 17, 18, 19, 20 Wert 3 (mit vier Nachkommastellen)
      // MSB (High byte 3), Byte 2, Byte 1, LSB (Low byte 0), Wertebereich -2,147,483,648 to +2,147,483,647
      // MSB (High byte 3)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
      tmp0 = tmp0 << 24;

      // Byte 2
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 16;

      // Byte 1
      // increment byte no.
      id++;
      tmp2 = 0;
      tmp2 = long(t[id]);
      tmp2 = tmp2 << 8;

      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp3 = 0;
      tmp3 = long(t[id]);

      tmp0 = tmp0 | tmp1 | tmp2 | tmp3;
      MotorReciveCopy.dValue[3] = double(tmp0) / 10000.0; // explizite Typumwandlung von long in double
      
      // Byte 21, 22, 23, 24 Wert 4 (mit vier Nachkommastellen)
      // MSB (High byte 3), Byte 2, Byte 1, LSB (Low byte 0), Wertebereich -2,147,483,648 to +2,147,483,647
      // MSB (High byte 3)
      // increment byte no.
      id++;
      tmp0 = 0;
      tmp0 = long(t[id]);
      tmp0 = tmp0 << 24;

      // Byte 2
      // increment byte no.
      id++;
      tmp1 = 0;
      tmp1 = long(t[id]);
      tmp1 = tmp1 << 16;

      // Byte 1
      // increment byte no.
      id++;
      tmp2 = 0;
      tmp2 = long(t[id]);
      tmp2 = tmp2 << 8;

      // LSB (Low byte 0)
      // increment byte no.
      id++;
      tmp3 = 0;
      tmp3 = long(t[id]);

      tmp0 = tmp0 | tmp1 | tmp2 | tmp3;
      MotorReciveCopy.dValue[4] = double(tmp0) / 10000.0; // explizite Typumwandlung von long in double

      // Byte 27 Checksum
      // increment byte no.
      id++;
      MotorReciveCopy.Checksum = t[id];

    } // if... Checksum passed

  } // if... Receive data only, when 29 bytes from master

  // ##### Reset tag receive data I2C is active
  bReciveDataI2CIsActive = false;
}

// ##### Function Motor step
// Input variable i = motor number
inline void Step(int i)
{
  if ((MotorCommon[i].DigitalOutputMotor & 0b00000001) != 0 )
  {
    MotorCommon[i].DigitalOutputMotor = 0b00000000;
  }
  else
  {
    MotorCommon[i].DigitalOutputMotor = 0b00000001;
  }
  return;
}

// ##### Function y=m*x+b
inline long YMXB(double Input)
{
  long ResultYMXB = 0; // Return value
  /*
  // Y = m*x+b
  #define YMXB_MinInput 200.0
  #define YMXB_MaxInput 2000.0
  #define YMXB_MinResult 0.0
  #define YMXB_MaxResult 100.0
  */
  ResultYMXB = long((Input-YMXB_MinInput)*((YMXB_MaxResult-YMXB_MinResult)/(YMXB_MaxInput-YMXB_MinInput))+YMXB_MinResult);

  if(ResultYMXB < YMXB_MinResult)
  {
    ResultYMXB = (long)YMXB_MinResult;
  }
  if(ResultYMXB > YMXB_MaxResult)
  {
    ResultYMXB = (long)YMXB_MaxResult;
  }

  return(ResultYMXB);
}

// ##### Function calculate speed from mm/min -> ProgramScans
// e.g.
// 2400mm/min :60 = 40mm/s
// 40mm/s :3 = 13,3 Turns/s (SpindlePitch 3,0mm)
// 13,3 Turns/s = 13,3 Turns/1000ms = 1000ms/13,3 Turns = 75 ms/Turn
// 75ms/Turn / 400 = 0,1875 ms/Step (Steps per turn motor = 400)
// 0,1875 ms/Step * 1000 = 187,5 us/Step
// 187,5 us/Step : 15,5625 = 12,048 ProgramScans
/*
#define TimeInterrupt 15.5625
const double StepsPerTurn = 400.0;
const double SpindlePitch = 3.0;
*/
inline unsigned long SpeedMM_Min_ProgramScans(double MM_Min)
{
  unsigned long ProgramScans = 0; // Return value
  double Umdr_s;
  double Ms_Umdr;
  double Ms_Step;
  double Us_Step;
  double fProgramscans;

  Umdr_s = MM_Min / (60.0 * SpindlePitch);
  Ms_Umdr = 1000.0 / Umdr_s;
  Ms_Step = Ms_Umdr / StepsPerTurn;
  Us_Step = Ms_Step * 1000.0;
  fProgramscans = Us_Step / TimeInterrupt;
  ProgramScans = (unsigned long)fProgramscans;

  return (ProgramScans);
}

// ##### Function Positioning
// Input variable i = motor number
void Positioning(int i, bool Direction, unsigned long lSpeed, bool ReleaseMovement)
{

  // ##### Stop movement if release movement is gone
  switch (MotorAll.StepNoStopMovement)
  {
    // No release movement
    case 0:
      if(ReleaseMovement == false)
      {
        MotorAll.EnableStep = false; // Switch enable step off and wait for next release
        MotorAll.StepNoStopMovement = 1;
        // Switch spindle off
        if(MotorAll.SpindleOn == true)
        {
          MotorAll.SpindleOn = false;
          MotorAll.SpindleWasOnStopMovement = true;
        }
      }
      break;   

    // Wait for next release
    case 1:
      if(ReleaseMovement == true)
      {
        MotorAll.StepNoStopMovement = 2;
        // Start delaytime with 1ms
        MotorCommon[0].DelaytimeMoveCommand  = false;
        MotorCommon[0].CounterDelaytimeMoveCommand = 0;
        MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;
    
        // Switch spindle on, set delaytime to 2000ms
        if(MotorAll.SpindleWasOnStopMovement == true)
        {
          MotorAll.SpindleOn = true;
          MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
        }
    
      }
      break; 

    // Start movement again
    case 2:
      if (MotorCommon[0].DelaytimeMoveCommand  == true)
      {
        MotorAll.StepNoStopMovement = 0;
      }
      break; 

    default:
      break;
  }

  // ##### Release positioning
  if(MotorAll.StepNoStopMovement == 0)
  {

    // ##### Step sequence positioning
    switch (MotorCommon[i].StepNoPositioning)
    {
  
      case StepPositioningRunReferenceSensorFreeFast:
          // Set bit for direction positioning
          if (Direction == true)
          {
            MotorCommon[i].DirectionPositivePositioning = false;
          }
          else
          {
            MotorCommon[i].DirectionPositivePositioning = true;
          }
    
          // Is delaytime move command elapsed?
          if (MotorCommon[i].DelaytimeMoveCommand  == false)
          {
            break;
          }
  
          MotorCommon[i].lCounterRamp ++;
          if (MotorCommon[i].lCounterRamp >= lSpeed)
          {
            MotorCommon[i].lCounterRamp = 0;
    
            // Motor is at reference sensor, one step positive or negative
            if (MotorCommon[i].ReferenceSensor == true)
            {
              // Motor one step
              Step(i);
            }
            // Motor no more at reference sensor
            else
            {
              MotorCommon[i].StepNoPositioning = StepPositioningFindReferenceSensor;
  
              // Delaytime release next steps
              MotorCommon[i].DelaytimeMoveCommand = false;
              MotorCommon[i].CounterDelaytimeMoveCommand = 0;
              MotorCommon[i].ValueDelaytimeMoveCommand = WaitPositioning;
            }
          }
          break;
      
      case StepPositioningFindReferenceSensor:
  
          // Set bit for direction positioning
          if (Direction == true)
          {
            MotorCommon[i].DirectionPositivePositioning = true;
          }
          else
          {
            MotorCommon[i].DirectionPositivePositioning = false;
          }
    
          // Is delaytime move command elapsed?
          if (MotorCommon[i].DelaytimeMoveCommand  == false)
          {
            break;
          }
  
          MotorCommon[i].lCounterRamp ++;
          if (MotorCommon[i].lCounterRamp >= lSpeed)
          {
            MotorCommon[i].lCounterRamp = 0;
    
            // Reference motor not found, one step positive or negative
            if (MotorCommon[i].ReferenceSensor == false)
            {
              // Motor one step
              Step(i);
            }
            // Reference motor found
            else
            {
              MotorCommon[i].StepNoPositioning = StepPositioningRunReferenceSensorFree;
  
              // Delaytime release next steps
              MotorCommon[i].DelaytimeMoveCommand = false;
              MotorCommon[i].CounterDelaytimeMoveCommand = 0;
              MotorCommon[i].ValueDelaytimeMoveCommand = WaitPositioning;
            }
          }
          break;
    
        case StepPositioningRunReferenceSensorFree:
    
          // Set bit for direction positioning
          if (Direction == true)
          {
            MotorCommon[i].DirectionPositivePositioning = false;
          }
          else
          {
            MotorCommon[i].DirectionPositivePositioning = true;
          }
    
          // Is delaytime move command elapsed?
          if (MotorCommon[i].DelaytimeMoveCommand  == false)
          {
            break;
          }
    
          // Delaytime between each step
          MotorCommon[i].DelaytimeMoveCommand = false;
          MotorCommon[i].CounterDelaytimeMoveCommand = 0;
          MotorCommon[i].ValueDelaytimeMoveCommand = WaitPositioning;
    
          // Motor is at reference sensor, one step positive or negative
          if (MotorCommon[i].ReferenceSensor == true)
          {
            // Motor one step
            Step(i);
          }
          // Motor no more at reference sensor
          else
          {
            MotorCommon[i].StepNoPositioning = StepPositioningGotoReferenceSensor;
  
            // Delaytime release next steps
            MotorCommon[i].DelaytimeMoveCommand = false;
            MotorCommon[i].CounterDelaytimeMoveCommand = 0;
            MotorCommon[i].ValueDelaytimeMoveCommand = WaitPositioning;
  
          }
          break;
  
        case StepPositioningGotoReferenceSensor:
  
          // Set bit for direction positioning
          if (Direction == true)
          {
            MotorCommon[i].DirectionPositivePositioning = true;
          }
          else
          {
            MotorCommon[i].DirectionPositivePositioning = false;
          }
  
          // Is delaytime move command elapsed?
          if (MotorCommon[i].DelaytimeMoveCommand  == false)
          {
            break;
          }
    
          // Delaytime between each step
          MotorCommon[i].DelaytimeMoveCommand = false;
          MotorCommon[i].CounterDelaytimeMoveCommand = 0;
          MotorCommon[i].ValueDelaytimeMoveCommand = WaitPositioning;
  
          // Reference motor not found, one step positive or negative
          if (MotorCommon[i].ReferenceSensor == false)
          {
            // Motor one step
            Step(i);
          }
          // Reference motor found
          else
          {
            MotorCommon[i].PositioningDone = true;
            MotorCommon[i].dAbsolutePosMotor = 0.0;
            MotorCommon[i].dAbsolutePosFromMachineZeroPoint = 0.0;
            MotorCommon[i].dRequiredPosAbsolute = 0.0; // New required position (Startposition for next move command)
            MotorAll.RotationG68IsActive = false;
            MotorAll.RotationG68CX = 0.0;
            MotorAll.RotationG68CY = 0.0;
            MotorAll.RotationG68Angle = 0.0;
          }
          break;

      default:
      break;
    } // switch (MotorCommon[i].StepNoPositioning)
  } //   if(MotorAll.StepNoStopMovement == 0)
} // End positioning

// ##### Initialization setup()
void setup()
{

  // ##### Set values at startup
  MotorAll.StepNo = StepNoAction;
  MotorAll.dRequiredSpindleSpeed = SpeedSpindleMin; // Set spindle speed
  
  // ##### Define LED on board
  pinMode(LED_BUILTIN, OUTPUT); // LED on board

  // ##### Define digital input/output common
  pinMode(DigitalInputEmergencyStop, INPUT_PULLUP);
  pinMode(DigitalInputToolLengthSensor, INPUT_PULLUP);
  pinMode(DigitalOutputSpindleOn, OUTPUT);
  pinMode(DigitalOutputCoolantOn, OUTPUT);
  pinMode(DigitalOutputClampOn, OUTPUT);
  pinMode(DigitalOutputLightOn, OUTPUT);
  pinMode(DigitalOutputPWMSpindleSpeed, OUTPUT);

  // ##### Define digital input/output motor 0
  pinMode(DigitalOutputMotor0Enable, OUTPUT);
  pinMode(DigitalOutputMotor0Step, OUTPUT);
  pinMode(DigitalOutputMotor0Dir, OUTPUT);
  pinMode(DigitalInputMotor0ReferenceSensor, INPUT);

  // ##### Define digital input/output motor 1
  pinMode(DigitalOutputMotor1Enable, OUTPUT);
  pinMode(DigitalOutputMotor1Step, OUTPUT);
  pinMode(DigitalOutputMotor1Dir, OUTPUT);
  pinMode(DigitalInputMotor1ReferenceSensor, INPUT);

  // ##### Define digital input/output motor 2
  pinMode(DigitalOutputMotor2Enable, OUTPUT);
  pinMode(DigitalOutputMotor2Step, OUTPUT);
  pinMode(DigitalOutputMotor2Dir, OUTPUT);
  pinMode(DigitalInputMotor2ReferenceSensor, INPUT);

  // ##### Serieller Monitor
    if (DEBUGMODE == true || DEBUGMODESTEP == true)
    {
      Serial.begin(9600);
      Serial.println("Startup");
    }

  // ##### Start interval timer
  // Lower numbers are higher priority, with 0 the highest and 255 the lowest.
  // Default priority = 128
  // Most other interrupts default to 128
  IntervalTimer0.begin(handler, TimeInterrupt);
  IntervalTimer0.priority(100);

  // ##### Configure I2C (Slave for Raspberry)
  Wire1.begin(SLAVE_ADDRESS); // Join I2C bus with as slave
  Wire1.onRequest(SendDataI2C); // Register event send data to I2C master
  Wire1.onReceive(ReciveDataI2C); // Register event receive data from I2C master

  // ##### Initialize LCD display, 4x20 Zeichen
  lcd.init();
  lcd.backlight();

  // ##### Memory allocation in RAM2 for subprogram step data
  SubProgramData = (StructStepData*)malloc(7500 * sizeof(StructStepData));
} // End void setup()

// ##### Main program loop()
void loop()
{
  // ##### Local static variables
  static bool ledon = false;
  static int frequencyLED = 0;
  static uint8_t StepNoLEDOnBoard = 0;
  static uint8_t StepNoLCDActiveLine = 0;
 
  // ##### Local variables  
  char str[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int i;

  // ##### LCD display
  // Because of performance only one line per program scan
  switch (StepNoLCDActiveLine)
  {
    case 0:
      i = 0;
      memset(str, 0, 20);
      sprintf(str,"%s = %9.4f mm", NameAxis[i], MotorCommon[i].dAbsolutePosMotor); // Ausgabe Istposition 9-Stellig (4 Vorkomma, Komma, 4 Nachkomma)
      lcd.setCursor(0,i); // Text startet ab Zeichen 0 am Display
      lcd.print(str);
      StepNoLCDActiveLine ++;
      break;

    case 1:
      i = 1;
      memset(str, 0, 20);
      sprintf(str,"%s = %9.4f mm", NameAxis[i], MotorCommon[i].dAbsolutePosMotor); // Ausgabe Istposition 9-Stellig (4 Vorkomma, Komma, 4 Nachkomma)
      lcd.setCursor(0,i); // Text startet ab Zeichen 0 am Display
      lcd.print(str);
      StepNoLCDActiveLine ++;
      break;

    case 2:
      i = 2;
      memset(str, 0, 20);
      sprintf(str,"%s = %9.4f mm", NameAxis[i], MotorCommon[i].dAbsolutePosMotor); // Ausgabe Istposition 9-Stellig (4 Vorkomma, Komma, 4 Nachkomma)
      lcd.setCursor(0,i); // Text startet ab Zeichen 0 am Display
      lcd.print(str);
      StepNoLCDActiveLine ++;
      break;

    case 3:
      i = 3;
      memset(str, 0, 20);
      //sprintf(str,"Checksum %i", CounterChecksumErrorI2CRecive);
      sprintf(str,"Milling machine");
      lcd.setCursor(0,i); // Text startet ab Zeichen 0 am Display
      lcd.print(str);
      StepNoLCDActiveLine = 0;
      break;

    default:
      break;
  } // End switch (StepNoLCDActiveLine)

  // ##### LED on board
  // Counter timer is elapsed, is counting in interrupt handler
  switch (StepNoLEDOnBoard)
  {
    case 0:
      if (timer >= 8000)
      {
        timer = 0;
        ledon = ! ledon;
        digitalWrite(LED_BUILTIN, ledon);
        
        frequencyLED++;
        if (frequencyLED >= 6)
        {
          frequencyLED = 0;
          StepNoLEDOnBoard = 1;
        }
      }
      break;

    case 1:
      if (timer >= 2000)
      {
        timer = 0;
        ledon = ! ledon;
        digitalWrite(LED_BUILTIN, ledon);

        frequencyLED++;
        if (frequencyLED >= 6)
        {
          frequencyLED = 0;
          StepNoLEDOnBoard = 0;
        }
      }
      break;

    default:
      break;
  } // End switch (StepNoLEDOnBoard)
} // End void loop()

// ##### Interrupt handler
void handler()
{
  // #############################################################################################################
  // ##### Local variables
  // Not defined as global variable with "volatile" because this variables are not used as memory, they are written each scan.
  // If there in a variable needed with memory, it has to be declared as global variable with "volatile"
  int i, j, k;
  bool MotorsReleased = false;
  bool PositioningDone = false;
  bool DirPosIsActive = false;
  double dQuadrantangleStartpos = 0;
  double dQuadrantangleEndpos = 0;
  int iActiveQuadrant = 0;
  bool bNextStepCircular = false;
  bool bInchingButtonPressed = false;
  bool bReleaseMovementCommon = false;
  bool bInchingIsActive = false;
  double dTmp;

  // #############################################################################################################
  // ##### Copy I2C recive data. Volatile data must be copied value by value, no struct copy possible
  if(bReciveDataI2CIsActive == false)
  {
    MotorRecive.Operate = MotorReciveCopy.Operate;
    MotorRecive.Reserve0_1 = MotorReciveCopy.Reserve0_1;
    MotorRecive.Reserve0_2 = MotorReciveCopy.Reserve0_2;
    MotorRecive.Reserve0_3 = MotorReciveCopy.Reserve0_3;
    MotorRecive.Reserve0_4 = MotorReciveCopy.Reserve0_4;
    MotorRecive.Reserve0_5 = MotorReciveCopy.Reserve0_5;
    MotorRecive.Reserve0_6 = MotorReciveCopy.Reserve0_6;
    MotorRecive.Reserve0_7 = MotorReciveCopy.Reserve0_7;
    
    MotorRecive.ActivateCommand = MotorReciveCopy.ActivateCommand;
    MotorRecive.Reserve1_1 = MotorReciveCopy.Reserve1_1;
    MotorRecive.Reserve1_2 = MotorReciveCopy.Reserve1_2;
    MotorRecive.Reserve1_3 = MotorReciveCopy.Reserve1_3;
    MotorRecive.Reserve1_4 = MotorReciveCopy.Reserve1_4;
    MotorRecive.Reserve1_5 = MotorReciveCopy.Reserve1_5;
    MotorRecive.Reserve1_6 = MotorReciveCopy.Reserve1_6;
    MotorRecive.Reserve1_7 = MotorReciveCopy.Reserve1_7;
    
    MotorRecive.SpindleOn = MotorReciveCopy.SpindleOn;
    MotorRecive.CoolantOn = MotorReciveCopy.CoolantOn;
    MotorRecive.ClampOn = MotorReciveCopy.ClampOn;
    MotorRecive.LightOn = MotorReciveCopy.LightOn;
    MotorRecive.XPlus = MotorReciveCopy.XPlus;
    MotorRecive.XMinus = MotorReciveCopy.XMinus;
    MotorRecive.YPlus = MotorReciveCopy.YPlus;
    MotorRecive.YMinus = MotorReciveCopy.YMinus;

    MotorRecive.ZPlus = MotorReciveCopy.ZPlus;
    MotorRecive.ZMinus = MotorReciveCopy.ZMinus;
    MotorRecive.ClearDiagnosticCounterI2C = MotorReciveCopy.ClearDiagnosticCounterI2C;
    MotorRecive.MeasureToolLength = MotorReciveCopy.MeasureToolLength;
    MotorRecive.Reserve3_4 = MotorReciveCopy.Reserve3_4;
    MotorRecive.Reserve3_5 = MotorReciveCopy.Reserve3_5;
    MotorRecive.Reserve3_6 = MotorReciveCopy.Reserve3_6;
    MotorRecive.Reserve3_7 = MotorReciveCopy.Reserve3_7;

    MotorRecive.SpindleSpeedManualMode = MotorReciveCopy.SpindleSpeedManualMode;

    MotorRecive.Code = MotorReciveCopy.Code;
    MotorRecive.dValue[0] = MotorReciveCopy.dValue[0];
    MotorRecive.dValue[1] = MotorReciveCopy.dValue[1];
    MotorRecive.dValue[2] = MotorReciveCopy.dValue[2];
    MotorRecive.dValue[3] = MotorReciveCopy.dValue[3];
    MotorRecive.dValue[4] = MotorReciveCopy.dValue[4];
    MotorRecive.Checksum = MotorReciveCopy.Checksum;
  }

  // #############################################################################################################
  // ##### Increase timer, volatile variable
  timer++;

  // #############################################################################################################
  // ##### Clear counter checksum error I2C recive
  if (MotorRecive.ClearDiagnosticCounterI2C == true)
  {
    CounterChecksumErrorI2CRecive = 0;
  }

  // #############################################################################################################
  // ##### Communication to Raspberry, fill StepData array
  // Reset all values
  if (MotorRecive.Operate == true or MotorSend.ProgramDone == true)
  {
    MotorSend.ReadyForCommand = false;
    MotorSend.CommandDone = false;
    MotorAll.StepNoCommMaster = 0;
    IndexStepData = 0;
    IndexSubStepData = 0;
    IndexSubStepNo = 0;
    ReciveSubprogram = false;
  }

  // Reset Program done
  if (MotorRecive.Operate == false)
  {
    MotorSend.ProgramDone = false;
  }

  // Fill Step data array
  switch (MotorAll.StepNoCommMaster)
  {
    case 0: // Is Operate == false?
      if(MotorRecive.Operate == false)
      {
        IndexStepData = 0;
        IndexSubStepData = 0;
        IndexSubStepNo = 0;
        ReciveSubprogram = false;
        MotorAll.StepNoCommMaster = 1;
      }
      break;

    case 1: // Set ready for command
      MotorSend.ReadyForCommand = true;
      MotorSend.CommandDone = false;
      MotorAll.StepNoCommMaster = 2;
      break;

    case 2: // Is ActivateCommand == true?
      if(MotorRecive.ActivateCommand == true)
      {

        // Subprogram data active, first step of subprogram
        // M98 P1000 L5
        if(MotorRecive.Code == CodeO)
        {
          ReciveSubprogram = true;

          // Store first step of subprogram in array "SubProgramData"
          // Store program no
          // Store number of repeats
          Subprogram.AdressPointerFirstStep[IndexSubStepNo] = IndexSubStepData; // Loaded during program transfer. Adress pointer in the Array SubProgramData[7500] where the subprogram starts (nesting)
          Subprogram.ProgramNo[IndexSubStepNo] = int(MotorRecive.dValue[0]); // Loaded during program transfer. Program no. of the subprogram, parameter P (M98 P1000 L5)
          IndexSubStepNo += 1;
        }

        // Store subprogram data
        if(ReciveSubprogram == true)
        {
          SubProgramData[IndexSubStepData].Code = MotorRecive.Code;
          SubProgramData[IndexSubStepData].dValue[0] = MotorRecive.dValue[0];
          SubProgramData[IndexSubStepData].dValue[1] = MotorRecive.dValue[1];
          SubProgramData[IndexSubStepData].dValue[2] = MotorRecive.dValue[2];
          SubProgramData[IndexSubStepData].dValue[3] = MotorRecive.dValue[3];
          SubProgramData[IndexSubStepData].dValue[4] = MotorRecive.dValue[4];
          IndexSubStepData += 1;
        }

        // Store step data
        if(ReciveSubprogram == false)
        {
          StepData[IndexStepData].Code = MotorRecive.Code;
          StepData[IndexStepData].dValue[0] = MotorRecive.dValue[0];
          StepData[IndexStepData].dValue[1] = MotorRecive.dValue[1];
          StepData[IndexStepData].dValue[2] = MotorRecive.dValue[2];
          StepData[IndexStepData].dValue[3] = MotorRecive.dValue[3];
          StepData[IndexStepData].dValue[4] = MotorRecive.dValue[4];
          IndexStepData += 1;
        }
        
        // Subprogram data inactive, last step of subprogram
        if(MotorRecive.Code == CodeM99)
        {
          ReciveSubprogram = false;
        }

        // Set handshake values to Raspberry
        MotorSend.ReadyForCommand = false;
        MotorSend.CommandDone = true;
        MotorAll.StepNoCommMaster = 3;
        
        // Reset values for move command. After loading a new program start from the beginning.
        MotorAll.CommandActive = false;
        MotorAll.StepNo = StepNoAction;
        IndexActualStepNo = 0;
        MotorSend.ActualStepProgram = IndexActualStepNo;
        Subprogram.IndexActualStepNo = 0;
        Subprogram.IndexActiveSubprogram = 0;
        Subprogram.IsActive = false;
      }
      break;

    case 3: // Is ActivateCommand == false?
      if(MotorRecive.ActivateCommand == false)
      {
        MotorSend.CommandDone = false;
        MotorAll.StepNoCommMaster = 1;
        // If code = programend, then set IndexStepData to 0 (so that another program can be loaded or the program will be started)
        if(MotorRecive.Code == CodeLastStep)
        {
          AmountOfStepsMain = IndexStepData;
          IndexStepData = 0;
          IndexSubStepData = 0;
          IndexSubStepNo = 0;
          MotorAll.StepNoCommMaster = 0;
        }
      }
      break;

    default:
      break;    
  } // End switch (StepNoCommMaster) 

  // #############################################################################################################
  // ##### Read digital inputs
  // Read digital input reference sensor
  MotorCommon[0].ReferenceSensor = digitalReadFast(DigitalInputMotor0ReferenceSensor);
  MotorCommon[1].ReferenceSensor = digitalReadFast(DigitalInputMotor1ReferenceSensor);
  MotorCommon[2].ReferenceSensor = digitalReadFast(DigitalInputMotor2ReferenceSensor);

  // Read digital input emergency stop
  MotorAll.EmergencyStop = digitalReadFast(DigitalInputEmergencyStop);

  // Read digital input tool length sensor
  MotorAll.ToolLengthSensor = digitalReadFast(DigitalInputToolLengthSensor);

  // #############################################################################################################
  // ##### Release movement common
  bReleaseMovementCommon = false;
  if(MotorAll.EnableStep == true && MotorRecive.Operate == true && MotorAll.EmergencyStop == false)
  {
     bReleaseMovementCommon = true;
  }

  // #############################################################################################################
  // ##### Inching is active
  bInchingButtonPressed = false;
  if(MotorRecive.XPlus == true || MotorRecive.XMinus == true || MotorRecive.YPlus == true ||
     MotorRecive.YMinus == true || MotorRecive.ZPlus == true || MotorRecive.ZMinus == true)
  {
    bInchingButtonPressed = true;
  }
  
  bInchingIsActive = false;
  if(bInchingButtonPressed == true && bReleaseMovementCommon == true)
  {
    bInchingIsActive = true;
  }

  // #############################################################################################################
  // ##### Motor logic
  switch (MotorAll.StepNo)
  {

    // ##### Step no action, load next command ####################################################################
    case StepNoAction:

      // ##### Start inching
      if(bInchingIsActive == true && MotorAll.CommandActive == false)
      {
        ActualStepData.Code = CodeInching;
        MotorAll.bInchingStarted = true;
        MotorAll.CommandActive = true;
      }

      // ##### Start measure tool length
      if(MotorRecive.MeasureToolLength == true && MotorAll.MeasureToolLengthDone == false && MotorAll.MeasureToolLengthStarted == false
          && bReleaseMovementCommon == true && MotorAll.CommandActive == false)
      {
        ActualStepData.Code = CodeMeasureToolLength;
        MotorAll.MeasureToolLengthStarted = true;
        MotorAll.CommandActive = true;
      }
      // Start measure tool length, handshake to GUI
      if(MotorRecive.MeasureToolLength == false)
      {
        MotorAll.MeasureToolLengthDone = false;
      }

      // ##### Start G-Code / M-Code from step data array
      // If program is done or not enabled or measure tool length done, break
      // && Inching started == false / tool length started == false, because program has to go for one cycle in Switch G-Code / M-Code
      if((MotorSend.ProgramDone == true || bReleaseMovementCommon == false || MotorAll.MeasureToolLengthDone == true)
          && MotorAll.bInchingStarted == false && MotorAll.MeasureToolLengthStarted == false)
        break;

      // Load next command G-Code / M-Code from step data array
      if(MotorAll.CommandActive == false && bReleaseMovementCommon == true)
      {
        if(Subprogram.IsActive == true)
        {
          ActualStepData.Code = SubProgramData[Subprogram.IndexActualStepNo].Code;
          ActualStepData.dValue[0] = SubProgramData[Subprogram.IndexActualStepNo].dValue[0];
          ActualStepData.dValue[1] = SubProgramData[Subprogram.IndexActualStepNo].dValue[1];
          ActualStepData.dValue[2] = SubProgramData[Subprogram.IndexActualStepNo].dValue[2];
          ActualStepData.dValue[3] = SubProgramData[Subprogram.IndexActualStepNo].dValue[3];
          ActualStepData.dValue[4] = SubProgramData[Subprogram.IndexActualStepNo].dValue[4];
          MotorSend.ActualStepProgram = AmountOfStepsMain + Subprogram.IndexActualStepNo -1;
          // Print step no. subprogram
          if(DEBUGMODESTEP == true)
          {
            Serial.print("SubprogramStep  ");
            Serial.println(Subprogram.IndexActualStepNo, DEC);            
          }
          // Increse index
          Subprogram.IndexActualStepNo += 1;
        }
        else
        {
          ActualStepData.Code = StepData[IndexActualStepNo].Code;
          ActualStepData.dValue[0] = StepData[IndexActualStepNo].dValue[0];
          ActualStepData.dValue[1] = StepData[IndexActualStepNo].dValue[1];
          ActualStepData.dValue[2] = StepData[IndexActualStepNo].dValue[2];
          ActualStepData.dValue[3] = StepData[IndexActualStepNo].dValue[3];
          ActualStepData.dValue[4] = StepData[IndexActualStepNo].dValue[4];
          MotorSend.ActualStepProgram = IndexActualStepNo;
          // Print step no. main
          if(DEBUGMODESTEP == true)
          {
            Serial.print("MainprogramStep  ");
            Serial.println(IndexActualStepNo, DEC);           
          }
          // Increse index
          IndexActualStepNo += 1;
        }
        MotorAll.CommandActive = true;
      }

      // #############################################################################################################
      // ##### Switch G-Code / M-Code
      switch (ActualStepData.Code)
      {
        // ##### Commando G00 / G01, prepare movement #########################################################################
        // ##### G00 Eilgang
        // ##### G01 Geradeninterpolation
        case CodeG00:
        case CodeG01:

          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            if(ActualStepData.Code == CodeG00)
            {
              Serial.println("##### Initialize G00");
            }
            else
            {
              Serial.println("##### Initialize G01");
            }
          }
          
          // ##### Execute command
          MotorAll.StepNo = StepLinearMovement;

          // ##### Transfer speed
          if(ActualStepData.Code == CodeG00)
          {
            MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(SpeedG00); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
            MotorAll.dActualSpeed = SpeedG00;
          }

          if(ActualStepData.Code == CodeG01)
          {
            if(ActualStepData.dValue[3] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(ActualStepData.dValue[3]); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
              MotorAll.dLastSpeed = ActualStepData.dValue[3];
              MotorAll.dActualSpeed = ActualStepData.dValue[3];
            }
            else
            {
              MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(MotorAll.dLastSpeed); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
              MotorAll.dActualSpeed = MotorAll.dLastSpeed;
            }
          }

          // ##### Is coordinate rotation G68 active?
          // Store last required positions X and Y
          if(ActualStepData.dValue[0] != 9999.9999)
          {
            MotorAll.LastRequiredPositionX = ActualStepData.dValue[0];
          }
          if(ActualStepData.dValue[1] != 9999.9999)
          {
            MotorAll.LastRequiredPositionY = ActualStepData.dValue[1];
          }
          
          if(MotorAll.RotationG68IsActive == true)
          {
            // x,y = current x,y
            // cx, cy = rotation center
            // Angle = angle to rotate point
            // nx, ny = new coordinates x,y

            // Radians =(Pi/-180)*Angle
            // Cos = cos(Radians)
            // Sin = sin(Radians)
            // nx = (Cos * (x - cx)) + (Sin * (y - cy)) + cx; 
            // ny = (Cos * (y - cy)) - (Sin * (x - cx)) + cy;

            double x; // current X
            double y; // current Y
            double cx; // rotation center X
            double cy; // rotation center Y
            double Angle; // angle to rotate point
            double Radians;
            double Cos;
            double Sin;
            double nx; // new coordinate X
            double ny; // new coordinate Y

            // Either X or Y movement must be active, otherwise just movement in Z direction
            if(ActualStepData.dValue[0] != 9999.9999 || ActualStepData.dValue[1] != 9999.9999)
            {
              // Input values
              if(ActualStepData.dValue[0] != 9999.9999) // 9999.9999 = Value not valid
              {
                x = ActualStepData.dValue[0]; // Transfer required position X
              }
              else
              {
                x = MotorAll.LastRequiredPositionX; // No movement in direction X, use last required position
              }
 
              if(ActualStepData.dValue[1] != 9999.9999) // 9999.9999 = Value not valid
              {
                y = ActualStepData.dValue[1]; // Transfer required position Y
              }
              else
              {
                y = MotorAll.LastRequiredPositionY; // No movement in direction Y, use last required position
              }
              cx = MotorAll.RotationG68CX;
              cy = MotorAll.RotationG68CY;
              Angle = MotorAll.RotationG68Angle;
  
              // Calculate values
              Radians =(Pi/-180.0)*Angle;
              Cos = cos(Radians);
              Sin = sin(Radians);
              nx = (Cos * (x - cx)) + (Sin * (y - cy)) + cx;
              ny = (Cos * (y - cy)) - (Sin * (x - cx)) + cy;
  
              // Output values
              ActualStepData.dValue[0] = nx; // nx, new required position X
              ActualStepData.dValue[1] = ny; // ny, new required position Y
            }
          }

          // ##### Startpositions (actual position)
          MotorAll.dStartpositionX = MotorCommon[0].dAbsolutePosMotor;
          MotorAll.dStartpositionY = MotorCommon[1].dAbsolutePosMotor;
          MotorAll.dStartpositionZ = MotorCommon[2].dAbsolutePosMotor;

          // ##### Endpositions
          // Absolute movement is active
          if(MotorAll.AbsoluteIsActive == true)
          {
            // Endposition X
            if(ActualStepData.dValue[0] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionX = ActualStepData.dValue[0];
              MotorCommon[0].dRequiredPosAbsolute = ActualStepData.dValue[0]; // New required position
            }
            else
            {
              MotorAll.dEndpositionX = MotorAll.dStartpositionX; // No movement in direction X
              // Leave old required position
            }
            // Endposition Y
            if(ActualStepData.dValue[1] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionY = ActualStepData.dValue[1];
              MotorCommon[1].dRequiredPosAbsolute = ActualStepData.dValue[1]; // New required position
            }
            else
            {
              MotorAll.dEndpositionY = MotorAll.dStartpositionY; // No movement in direction Y
              // Leave old required position
            }
            // Endposition Z
            if(ActualStepData.dValue[2] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionZ = ActualStepData.dValue[2];
              MotorCommon[2].dRequiredPosAbsolute = ActualStepData.dValue[2]; // New required position
            }
            else
            {
              MotorAll.dEndpositionZ = MotorAll.dStartpositionZ; // No movement in direction Z
              // Leave old required position
            }
          }

          // ##### Endpositions
          // Relative movement is active
          if(MotorAll.RelativeIsActive == true)
          {
            // Endposition X
            if(ActualStepData.dValue[0] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionX += ActualStepData.dValue[0];
              MotorCommon[0].dRequiredPosAbsolute += ActualStepData.dValue[0]; // New required position
            }
            else
            {
              MotorAll.dEndpositionX = MotorAll.dStartpositionX; // No movement in direction X
              // Leave old required position
            }
            // Endposition Y
            if(ActualStepData.dValue[1] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionY += ActualStepData.dValue[1];
              MotorCommon[1].dRequiredPosAbsolute += ActualStepData.dValue[1]; // New required position
            }
            else
            {
              MotorAll.dEndpositionY = MotorAll.dStartpositionY; // No movement in direction Y
              // Leave old required position
            }
            // Endposition Z
            if(ActualStepData.dValue[2] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorAll.dEndpositionZ += ActualStepData.dValue[2];
              MotorCommon[2].dRequiredPosAbsolute += ActualStepData.dValue[2]; // New required position
            }
            else
            {
              MotorAll.dEndpositionZ = MotorAll.dStartpositionZ; // No movement in direction Z
              // Leave old required position
            }
          }

          // Distance
          MotorAll.dDistanceX = MotorAll.dEndpositionX - MotorAll.dStartpositionX; // Value could be negative
          MotorAll.dDistanceY = MotorAll.dEndpositionY - MotorAll.dStartpositionY; // Value could be negative
          MotorAll.dDistanceZ = MotorAll.dEndpositionZ - MotorAll.dStartpositionZ; // Value could be negative

          MotorAll.dDistanceAbsX = fabs(MotorAll.dDistanceX); // Positive value
          MotorAll.dDistanceAbsY = fabs(MotorAll.dDistanceY); // Positive value
          MotorAll.dDistanceAbsZ = fabs(MotorAll.dDistanceZ); // Positive value

          // Amount of steps X, Y, Z
          MotorAll.dAmountStepsX = MotorAll.dDistanceX / MM_Step; // Value could be negative
          MotorAll.dAmountStepsY = MotorAll.dDistanceY / MM_Step; // Value could be negative
          MotorAll.dAmountStepsZ = MotorAll.dDistanceZ / MM_Step; // Value could be negative

          MotorAll.dAmountStepsAbsX = fabs(MotorAll.dAmountStepsX); // Positive value
          MotorAll.dAmountStepsAbsY = fabs(MotorAll.dAmountStepsY); // Positive value
          MotorAll.dAmountStepsAbsZ = fabs(MotorAll.dAmountStepsZ); // Positive value

          // Max. step from steps X, Y, Z
          if(MotorAll.dAmountStepsAbsX >= MotorAll.dAmountStepsAbsY)
          {
            MotorAll.dAmountStepsMax = MotorAll.dAmountStepsAbsX;
          }
          else
          {
            MotorAll.dAmountStepsMax = MotorAll.dAmountStepsAbsY;
          }
          if(MotorAll.dAmountStepsMax < MotorAll.dAmountStepsAbsZ)
          {
            MotorAll.dAmountStepsMax = MotorAll.dAmountStepsAbsZ;
          }

          // Distance per step X, Y, Z
          MotorAll.dDistancePerStepX = MotorAll.dDistanceX / MotorAll.dAmountStepsMax; // Value could be negative
          MotorAll.dDistancePerStepY = MotorAll.dDistanceY / MotorAll.dAmountStepsMax; // Value could be negative
          MotorAll.dDistancePerStepZ = MotorAll.dDistanceZ / MotorAll.dAmountStepsMax; // Value could be negative

          MotorAll.dDistanceAbsPerStepX = fabs(MotorAll.dDistancePerStepX); // Positive value
          MotorAll.dDistanceAbsPerStepY = fabs(MotorAll.dDistancePerStepY); // Positive value
          MotorAll.dDistanceAbsPerStepZ = fabs(MotorAll.dDistancePerStepZ); // Positive value

          // Amount of steps (long)
          dTmp = fabs(MotorAll.dAmountStepsMax); // fabs returns the positive value of a double. The return value is also a double
          MotorAll.lAmountSteps = (long)dTmp; // Explizite Typumwandlung, Nachkommastelen abschneiden, from double to long

          // Calculate amount of steps ramp, depend on speed
          // Actual Speed >=200  and <= 2000, Steps Ramp = 0..100
          // Actual Speed < 200, Steps Ramp = 0
          // Actual Speed > 2000, Steps Ramp = 100
          // Y = m*x+b
          MotorAll.lAmountStepsRamp = YMXB(MotorAll.dActualSpeed);

          // Stepcounter, Ramp
          MotorAll.lStepcounter = 0;
          MotorAll.lCounterRamp = 0;
          MotorAll.lMaxStepRamp = MotorAll.lAmountSteps / 2;
          MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep + MotorAll.lAmountStepsRamp;
          MotorAll.Stopramp = false;
          MotorAll.DoStep = false;

          // MotorAll.lAmountStepsRamp == 0, Movement is without start - and stopramp
          if(MotorAll.lAmountStepsRamp == 0)
          {
            MotorAll.Startramp = false;
            MotorAll.Linear = true;
            MotorAll.MovementWithoutRamp = true;
          }
          else
          {
            MotorAll.Startramp = true;
            MotorAll.Linear = false;
            MotorAll.MovementWithoutRamp = false;           
          }

          // Switch direction bits
          MotorCommon[0].StepsPositiveIsActive = true;
          MotorCommon[1].StepsPositiveIsActive = true;
          MotorCommon[2].StepsPositiveIsActive = true;
          if(MotorAll.dDistanceX < 0.0)
          {
            MotorCommon[0].StepsPositiveIsActive = false;
          }
          
          if(MotorAll.dDistanceY < 0.0)
          {
            MotorCommon[1].StepsPositiveIsActive = false;
          }
          
          if(MotorAll.dDistanceZ < 0.0)
          {
            MotorCommon[2].StepsPositiveIsActive = false;
          }

          // Switch step bits off
          MotorCommon[0].DigitalOutputMotor = 0b00000000;
          MotorCommon[1].DigitalOutputMotor = 0b00000000;
          MotorCommon[2].DigitalOutputMotor = 0b00000000;
          MotorCommon[3].DigitalOutputMotor = 0b00000000;
          MotorCommon[0].ResetStep = false;
          MotorCommon[1].ResetStep = false;
          MotorCommon[2].ResetStep = false;
          MotorCommon[3].ResetStep = false;

          // Reset internal distancecounter
          MotorAll.dInternalDistancecounterX = 0;
          MotorAll.dInternalDistancecounterY = 0;
          MotorAll.dInternalDistancecounterZ = 0;

          // Delaytime release next steps
          MotorCommon[0].DelaytimeMoveCommand = false;
          MotorCommon[0].CounterDelaytimeMoveCommand = 0;
          MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;

          // Reset tags for stop movement if release movement is gone
          MotorAll.StepNoStopMovement = 0;
          MotorAll.lCounterStepsStopMovement = 0;
          MotorAll.SpindleWasOnStopMovement = false;
          MotorAll.StepNoStopSpindle = 0;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;

          break; // case G00 / G01 Geradeninterpolation

        // ##### Commando G02 / G03, prepare movement #########################################################################
        // ##### G02 Kreisinterpolation, im Uhrzeigersinn
        // ##### G03 Kreisinterpolation, gegen Uhrzeigersinn
        case CodeG02:
        case CodeG03:
        
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            if(ActualStepData.Code == CodeG02)
            {
              Serial.println("##### Initialize G02");
            }
            else
            {
              Serial.println("##### Initialize G03");
            }
          }

          // ##### Execute command
          MotorAll.StepNo = StepCircularMovement;

          // ##### Transfer speed
          if(ActualStepData.dValue[4] != 9999.9999) // 9999.9999 = Value not valid
          {
            MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(ActualStepData.dValue[4]); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
            MotorAll.dLastSpeed = ActualStepData.dValue[4];
            MotorAll.dActualSpeed = ActualStepData.dValue[4];
          }
          else
          {
            MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(MotorAll.dLastSpeed); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
            MotorAll.dActualSpeed = MotorAll.dLastSpeed;
          }

          // ##### Is coordinate rotation G68 active?
          if(MotorAll.RotationG68IsActive == true)
          {
            // x,y = current x,y
            // cx, cy = rotation center
            // Angle = angle to rotate point
            // nx, ny = new coordinates x,y

            // Radians =(Pi/-180)*Angle
            // Cos = cos(Radians)
            // Sin = sin(Radians)
            // nx = (Cos * (x - cx)) + (Sin * (y - cy)) + cx; 
            // ny = (Cos * (y - cy)) - (Sin * (x - cx)) + cy;

            double x; // current X
            double y; // current Y
            double cx; // rotation center X
            double cy; // rotation center Y
            double Angle; // angle to rotate point
            double Radians;
            double Cos;
            double Sin;
            double nx; // new coordinate X
            double ny; // new coordinate Y
            double AbsolutePosI;
            double AbsolutePosJ;

            // Store absolute values I and J
            AbsolutePosI = ActualStepData.dValue[0] - ActualStepData.dValue[2]; // Save absolute pos. I = X - I
            AbsolutePosJ = ActualStepData.dValue[1] - ActualStepData.dValue[3]; // Save absolute pos. J = Y - J

            x = ActualStepData.dValue[0]; // Transfer required position X
            y = ActualStepData.dValue[1]; // Transfer required position Y

            cx = MotorAll.RotationG68CX;
            cy = MotorAll.RotationG68CY;
            Angle = MotorAll.RotationG68Angle;

            // Calculate values X and Y
            Radians =(Pi/-180.0)*Angle;
            Cos = cos(Radians);
            Sin = sin(Radians);
            nx = (Cos * (x - cx)) + (Sin * (y - cy)) + cx;
            ny = (Cos * (y - cy)) - (Sin * (x - cx)) + cy;

            // Output values
            ActualStepData.dValue[0] = nx; // nx, new required position X
            ActualStepData.dValue[1] = ny; // ny, new required position Y

            // Calculate values I and J
            x = AbsolutePosI; // Transfer absolute position I
            y = AbsolutePosJ; // Transfer absolute position J
            nx = (Cos * (x - cx)) + (Sin * (y - cy)) + cx;
            ny = (Cos * (y - cy)) - (Sin * (x - cx)) + cy;
            ActualStepData.dValue[2] = ActualStepData.dValue[0] - nx; // nx, new required position I
            ActualStepData.dValue[3] = ActualStepData.dValue[1] - ny; // ny, new required position J
          }

          // ##### Calculate radius
          // =square root((ABS(I)^2)+(ABS(J)^2)) // I=Kreismittelpunkt X relativ zur Startposition, J=Kreismittelpunkt Y relativ zur Startposition
          // sqrt sqrt(), Return value square root = double
          // square sq(), Return value square = double
          MotorAll.dRadius = sqrt( sq(fabs(ActualStepData.dValue[2])) + sq(fabs(ActualStepData.dValue[3])) );
          if(MotorAll.dRadius <= 0.0)
          {
            MotorAll.dRadius = 0.001;
          }

          // ##### Transfer startposition (last required position)
          MotorAll.dStartpositionX = MotorCommon[0].dRequiredPosAbsolute;
          MotorAll.dStartpositionY = MotorCommon[1].dRequiredPosAbsolute;

          // ##### Transfer endposition
          if(MotorAll.AbsoluteIsActive == true)
          {
            MotorAll.dEndpositionX = ActualStepData.dValue[0];
            MotorAll.dEndpositionY = ActualStepData.dValue[1];
            MotorCommon[0].dRequiredPosAbsolute = ActualStepData.dValue[0]; // New required position
            MotorCommon[1].dRequiredPosAbsolute = ActualStepData.dValue[1]; // New required position
          }
          if(MotorAll.RelativeIsActive == true)
          {
            MotorAll.dEndpositionX += ActualStepData.dValue[0];
            MotorAll.dEndpositionY += ActualStepData.dValue[1];
            MotorCommon[0].dRequiredPosAbsolute = ActualStepData.dValue[0]; // New required position
            MotorCommon[1].dRequiredPosAbsolute = ActualStepData.dValue[1]; // New required position
          }

          // ##### Calculate center of the circle
          // Center X = Startposition X + I
          // Center Y = Startposition Y + J
          MotorAll.CenterCircleX = MotorAll.dStartpositionX + ActualStepData.dValue[2];
          MotorAll.CenterCircleY = MotorAll.dStartpositionY + ActualStepData.dValue[3];

          // ##### Is G02 or G03 active
          if(ActualStepData.Code == CodeG02)
          {
            MotorAll.G02IsActive = true;
            MotorAll.G03IsActive = false;
          }
          else
          {
            MotorAll.G02IsActive = false;
            MotorAll.G03IsActive = true;
          }

          // ##### Quadrant startposition
          // I >= 0.0
          if(ActualStepData.dValue[2] >= 0.0)
          {
            // Quadrant 2 (90.0deg)  = J < 0
            // Quadrant 3 (180.0deg) = J >= 0
            // J < 0.0
            if(ActualStepData.dValue[3] < 0.0)
            {
              dQuadrantangleStartpos = 90.0; // Startposition in quadrant 2
            }
            // J >= 0.0
            else
            {
              dQuadrantangleStartpos = 180.0; // Startposition in quadrant 3
            }
          }
          // I < 0.0
          else
          {
            // Quadrant 1 (0.0deg)   = J <= 0
            // Quadrant 4 (270.0deg) = J > 0
            // J <= 0.0
            if(ActualStepData.dValue[3] <= 0.0)
            {
              dQuadrantangleStartpos = 0.0; // Startposition in quadrant 1
            }
            // J > 0.0
            else
            {
              dQuadrantangleStartpos = 270.0; // Startposition in quadrant 4
            }
          }

          // ##### Quadrant endposition
          // Quadrant 1 (0.0deg)   = End X > Kreismittel X UND End Y >= Kreismittel Y
          // Quadrant 2 (90.0deg)  = End X <= Kreismittel X UND End Y > Kreismittel Y
          // Quadrant 3 (180.0deg) = End X < Kreismittel X UND End Y <= Kreismittel Y
          // Quadrant 4 (270.0deg) = End X >= Kreismittel X UND End Y < Kreismittel Y
          if(ActualStepData.dValue[0] > MotorAll.CenterCircleX and ActualStepData.dValue[1] >= MotorAll.CenterCircleY)
          {
            dQuadrantangleEndpos = 0.0; // Endposition in quadrant 1
          }
          
          if(ActualStepData.dValue[0] <= MotorAll.CenterCircleX and ActualStepData.dValue[1] > MotorAll.CenterCircleY)
          {
            dQuadrantangleEndpos = 90.0; // Endposition in quadrant 2
          }

          if(ActualStepData.dValue[0] < MotorAll.CenterCircleX and ActualStepData.dValue[1] <= MotorAll.CenterCircleY)
          {
            dQuadrantangleEndpos = 180.0; // Endposition in quadrant 3
          }

          if(ActualStepData.dValue[0] >= MotorAll.CenterCircleX and ActualStepData.dValue[1] < MotorAll.CenterCircleY)
          {
            dQuadrantangleEndpos = 270.0; // Endposition in quadrant 4
          }

          // ##### Calculate angle startposition
          // Angle startpos in quadrant 1 or 3
          // =(ARCSIN(ABS(J)/Radius)*180/3,141592) +0.0 oder +180.0
          if(dQuadrantangleStartpos == 0.0 or dQuadrantangleStartpos == 180.0)
          {
            // ARCSIN, value must be in range -1.0 .. 1.0
            dTmp = fabs(ActualStepData.dValue[3] / MotorAll.dRadius);
            if(dTmp > 1.0)
            {
              dTmp = 1.0;
            }
            //MotorAll.dStartAngle = (asin((fabs(J) / MotorAll.dRadius)) *Rad_To_Deg) + dQuadrantangleStartpos;
            MotorAll.dStartAngle = (asin(dTmp) *Rad_To_Deg) + dQuadrantangleStartpos;
          }
          // Angle startpos in quadrant 2 or 4
          // =(ARCSIN(ABS(I)/Radius)*180/3,141592) +90.0 oder +270.0
          else
          {
            // ARCSIN, value must be in range -1.0 .. 1.0
            dTmp = fabs(ActualStepData.dValue[2] / MotorAll.dRadius);
            if(dTmp > 1.0)
            {
              dTmp = 1.0;
            }
            //MotorAll.dStartAngle = (asin((fabs(I) / MotorAll.dRadius)) *Rad_To_Deg) + dQuadrantangleStartpos;
            MotorAll.dStartAngle = (asin(dTmp) *Rad_To_Deg) + dQuadrantangleStartpos;
          }

          if(MotorAll.dStartAngle >= 360.0)
          {
            MotorAll.dStartAngle -= 360.0;
          }

          // ##### Calculate angle endposition
          // Angle endpos in quadrant 1 or 3
          // =(ARCSIN(ABS(KreismittelY-EndposY)/Radius)*180/3,141592) +0.0 oder +180.0
          if(dQuadrantangleEndpos == 0.0 or dQuadrantangleEndpos == 180.0)
          {
            // ARCSIN, value must be in range -1.0 .. 1.0
            dTmp = fabs((MotorAll.CenterCircleY - MotorAll.dEndpositionY) / MotorAll.dRadius);
            if(dTmp > 1.0)
            {
              dTmp = 1.0;
            }
            //MotorAll.dEndAngle = (asin((fabs(KreismittelY-EndposY) / MotorAll.dRadius)) *Rad_To_Deg) + dQuadrantangleEndpos;
            MotorAll.dEndAngle = (asin(dTmp) *Rad_To_Deg) + dQuadrantangleEndpos;
          }
          // Angle endpos in quadrant 2 or 4
          // =(ARCSIN(ABS(KreismittelX-EndposX)/Radius)*180/3,141592) +90.0 oder +270.0
          else
          {
            // ARCSIN, value must be in range -1.0 .. 1.0
            dTmp = fabs((MotorAll.CenterCircleX - MotorAll.dEndpositionX) / MotorAll.dRadius);
            if(dTmp > 1.0)
            {
              dTmp = 1.0;
            }
            //MotorAll.dEndAngle = (asin((fabs(KreismittelX-EndposX) / MotorAll.dRadius)) *Rad_To_Deg) + dQuadrantangleEndpos;
            MotorAll.dEndAngle = (asin(dTmp) *Rad_To_Deg) + dQuadrantangleEndpos;
          }

          if(MotorAll.dEndAngle >= 360.0)
          {
            MotorAll.dEndAngle -= 360.0;
          }

          // ##### Calculate angle distance
          if(MotorAll.G03IsActive == true)
          {
            MotorAll.dDistanceAngle = MotorAll.dEndAngle - MotorAll.dStartAngle;
          }
          else
          {
            MotorAll.dDistanceAngle = MotorAll.dStartAngle - MotorAll.dEndAngle;
          }
          
          if(MotorAll.dDistanceAngle <=0.0)
          {
            MotorAll.dDistanceAngle += 360.0;
          }

          // ##### Calculate angle step
          MotorAll.dAngleStep = asin(MM_Step / MotorAll.dRadius) *Rad_To_Deg;

          // Calculate amount of steps ramp, depend on speed
          // Actual Speed >=200  and <= 2000, Steps Ramp = 0..100
          // Actual Speed < 200, Steps Ramp = 0
          // Actual Speed > 2000, Steps Ramp = 100
          // Y = m*x+b
          MotorAll.lAmountStepsRamp = YMXB(MotorAll.dActualSpeed);

          // Stepcounter
          MotorAll.lStepcounter = 0;
          MotorAll.lCounterRamp = 0;

          // Distance ramp
          MotorAll.dMaxStepRamp = MotorAll.dDistanceAngle / 2.0;
          MotorAll.dDistanceAngleRamp = MotorAll.dAngleStep * (double)MotorAll.lAmountStepsRamp;
          MotorAll.dStartrampEndAngle = MotorAll.dDistanceAngleRamp;
          MotorAll.dStoprampStartAngle = MotorAll.dDistanceAngle - MotorAll.dDistanceAngleRamp;
          MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep + MotorAll.lAmountStepsRamp;
          MotorAll.Stopramp = false;
          MotorAll.DoStep = false;

          // MotorAll.lAmountStepsRamp == 0, Movement is without start - and stopramp
          if(MotorAll.lAmountStepsRamp == 0)
          {
            MotorAll.Startramp = false;
            MotorAll.Linear = true;
            MotorAll.MovementWithoutRamp = true;
          }
          else
          {
            MotorAll.Startramp = true;
            MotorAll.Linear = false;
            MotorAll.MovementWithoutRamp = false;           
          }

          // Switch direction bits
          MotorCommon[0].StepsPositiveIsActive = true;
          MotorCommon[1].StepsPositiveIsActive = true;
          if(dQuadrantangleStartpos == 0.0 or dQuadrantangleStartpos == 90.0)
          {
             MotorCommon[0].StepsPositiveIsActive = false;
          }

          if(dQuadrantangleStartpos == 90.0 or dQuadrantangleStartpos == 180.0)
          {
             MotorCommon[1].StepsPositiveIsActive = false;
          }

          // Switch step bits off
          MotorCommon[0].DigitalOutputMotor = 0b00000000;
          MotorCommon[1].DigitalOutputMotor = 0b00000000;
          MotorCommon[2].DigitalOutputMotor = 0b00000000;
          MotorCommon[3].DigitalOutputMotor = 0b00000000;
          MotorCommon[0].ResetStep = false;
          MotorCommon[1].ResetStep = false;
          MotorCommon[2].ResetStep = false;
          MotorCommon[3].ResetStep = false;

          // Reset internal anglecounter
          MotorAll.dInternalAnglecounter = 0.0;

          // Set actual angle to start angle
          MotorAll.dActualAngle = MotorAll.dStartAngle;

          // Reset internal distancecounter
          MotorAll.dInternalDistancecounterX = 0.0;
          MotorAll.dInternalDistancecounterY = 0.0;
          MotorAll.dInternalDistancecounterZ = 0.0;

          // Delaytime release next steps
          MotorCommon[0].DelaytimeMoveCommand = false;
          MotorCommon[0].CounterDelaytimeMoveCommand = 0;
          MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;

          // Reset tags for stop movement if release movement is gone
          MotorAll.StepNoStopMovement = 0;
          MotorAll.lCounterStepsStopMovement = 0;
          MotorAll.SpindleWasOnStopMovement = false;
          MotorAll.StepNoStopSpindle = 0;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;
 
          break; // case G02 / G03 Kreisinterpolation

        // ##### Command inching, prepare movement #########################################################################
        case CodeInching:

          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Initialize inching");
          }

          // ##### Execute command
          MotorAll.StepNo = StepInching;

          // ##### Transfer speed
          MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(SpeedInching); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
          MotorAll.dActualSpeed = SpeedInching;

          // Stepcounter, Ramp
          MotorAll.lCounterRampInchingX = 0;
          MotorAll.lCounterRampInchingY = 0;
          MotorAll.lCounterRampInchingZ = 0;
          MotorAll.DoStepInchingX = false;
          MotorAll.DoStepInchingY = false;
          MotorAll.DoStepInchingZ = false;

          // Clear step no. inching
          MotorAll.StepNoInchingX = 0;
          MotorAll.StepNoInchingY = 0;
          MotorAll.StepNoInchingZ = 0;

          // Clear inching direction active
          MotorAll.bInchingXPlusActive = false;
          MotorAll.bInchingXMinusActive = false;
          MotorAll.bInchingYPlusActive = false;
          MotorAll.bInchingYMinusActive = false;
          MotorAll.bInchingZPlusActive = false;
          MotorAll.bInchingZMinusActive = false;

          // Switch step bits off
          MotorCommon[0].DigitalOutputMotor = 0b00000000;
          MotorCommon[1].DigitalOutputMotor = 0b00000000;
          MotorCommon[2].DigitalOutputMotor = 0b00000000;
          MotorCommon[3].DigitalOutputMotor = 0b00000000;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;

          break; // CodeInching

        // ##### Command measure tool length, prepare movement #########################################################################
        case CodeMeasureToolLength:

          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Initialize measure tool length");
          }

          // ##### Execute command
          MotorAll.StepNo = StepMeasureToolLength;

          // ##### Transfer speed
          MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(SpeedMeasureToolLength); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
          MotorAll.dActualSpeed = SpeedMeasureToolLength;

          // Stepcounter
          MotorAll.lCounterRamp = 0;

          // Set step no. measure tool length
          MotorAll.StepNoMeasureToolLength = StepMeasureToolLengthRunSensorFreeFast;

          // Switch step bits off
          MotorCommon[0].DigitalOutputMotor = 0b00000000;
          MotorCommon[1].DigitalOutputMotor = 0b00000000;
          MotorCommon[2].DigitalOutputMotor = 0b00000000;
          MotorCommon[3].DigitalOutputMotor = 0b00000000;

          // Delaytime release next steps
          MotorCommon[2].DelaytimeMoveCommand = false;
          MotorCommon[2].CounterDelaytimeMoveCommand = 0;
          MotorCommon[2].ValueDelaytimeMoveCommand = WaitMoveCommand;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;

          break; // CodeMeasureToolLength
        
        // ##### G04 Verweilzeit in Sekunden #########################################################################
        case CodeG04:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Initialize G04");
            Serial.print("Dwelltime in s "); 
            Serial.println(ActualStepData.dValue[0], DEC);
          }

          // ##### Execute command
          MotorAll.StepNo = StepDwellTime;

          // ##### Calculate dwell time in cycles
          // Dwell time with 4 decimal places e.g. G04 P2.1234 -> ActualStepData.dValue[0] = 2.1234
          // #define TimeInterrupt 15.5625
          MotorAll.lDwellTimeCycles = long((ActualStepData.dValue[0] *1000000.0) / TimeInterrupt);
          MotorAll.lDwellTimeCounter = 0;

          // Reset tags for stop dwelltime if release movement is gone
          MotorAll.StepNoStopSpindle = 0;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;
          
          break;

        // ##### G52 Koordinatensystem Verschiebung ##############################
        case CodeG52:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G52");
          }
          
          // ##### Koordinatensystem Verschiebung
          for (i = 0; i < AmountOfSteppermotors; i++)
          {
            if(ActualStepData.dValue[i] != 9999.9999) // 9999.9999 = Value not valid
            {
              dTmp = ActualStepData.dValue[i] - MotorCommon[i].dAbsolutePosFromMachineZeroPoint;
              MotorCommon[i].dAbsolutePosMotor -= dTmp;
              MotorCommon[i].dAbsolutePosFromMachineZeroPoint += dTmp; // So viel wurde schon verschoben
            }
          }          

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### G54 Nullpunktverschiebung auf Werkstücknullpunkt oder Programmnullpunkt ##############################
        // Nullpunktverschiebung soll Absolut zum machine zero point sein
        case CodeG54:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G54");
          }

          // ##### Nullpunktverschiebung auf Werkstücknullpunkt oder Programmnullpunkt
          for (i = 0; i < AmountOfSteppermotors; i++)
          {
            if(ActualStepData.dValue[i] != 9999.9999) // 9999.9999 = Value not valid
            {
              MotorCommon[i].dAbsolutePosMotor -= ActualStepData.dValue[i];
            }
          }

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### G68 Coordinate rotation ############################################################################################
        case CodeG68:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G68");
          }

          // ##### Set coordinate rotation variables
          MotorAll.RotationG68IsActive = true;
          MotorAll.RotationG68CX = ActualStepData.dValue[0];
          MotorAll.RotationG68CY = ActualStepData.dValue[1];
          MotorAll.RotationG68Angle = ActualStepData.dValue[2];

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### G69 Cancel coordinate rotation ######################################################################################
        case CodeG69:        
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G69");
          }
          
          // ##### Clear coordinate rotation variables
          MotorAll.RotationG68IsActive = false;
          MotorAll.RotationG68CX = 0.0;
          MotorAll.RotationG68CY = 0.0;
          MotorAll.RotationG68Angle = 0.0;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### G74 positioning, prepare movement #########################################################################
        // ##### G74 Referenzfahrt über Referenzfühler anfahren
        case CodeG74:

          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Initialize G74");
          }

          // ##### Execute command
          MotorAll.StepNo = StepPositioningMain;
          
          // Clear arrays PosAmount[4] and PosOrder[4][4]
          for (i = 0; i < 4; i++)
          {
            PosAmount[i] = 0;
            for (j = 0; j < 4; j++)
            {
              PosOrder[i][j] = 0;
            }
          }
          /*
          Value 0 = Order X (1, 2 or 3)
          Value 1 = Order Y (1, 2 or 3)
          Value 2 = Order Z (1, 2 or 3)

          Value 0 = 1 -> X1 e.g. X and Y at the same time, then Z
          Value 1 = 1 -> Y1
          Value 2 = 2 -> Z2
          PosOrder 1200 PosAmount 2 1 0 0   PosOrder 0/0=1 0/1=2 0/2=0 0/3=0
                   3000                              1/0=3 1/1=0 1/2=0 1/3=0
                   0000                              2/0=0 2/1=0 2/2=0 2/3=0
                   0000                              3/0=0 3/1=0 3/2=0 3/3=0

          Value 0 = 1 -> X1 e.g. X, Y and Z at the same time
          Value 1 = 1 -> Y1
          Value 2 = 1 -> Z1
          PosOrder 1230 PosAmount 3 0 0 0
                   0000
                   0000
                   0000

          Value 0 = 1 -> X1 e.g. X first, then Y and then Z
          Value 1 = 2 -> Y2
          Value 2 = 3 -> Z3
          PosOrder 1000 PosAmount 1 1 1 0
                   2000
                   3000
                   0000
          */
          for (i = 0; i < 4; i++)
          {
            for (j = 0; j < 4; j++)
            {
              if(long(ActualStepData.dValue[j]) == long(i+1))
              {
                k = PosAmount[i];
                PosOrder[i][k] = j+1;
                PosAmount[i] ++;
              }
            }
          }
       
          MotorAll.StepPositioningStepOrder = 0;
          MotorAll.PositioningIsActive = true;
          MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(SpeedG74); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long

          for (i = 0; i < AmountOfSteppermotors; i++)
          {
            MotorCommon[i].PositioningDone = false;
            MotorCommon[i].PositioningIsActive = false;
            MotorCommon[i].lCounterRamp = 0;
            MotorCommon[i].dAbsolutePosMotor = 0.0;
            MotorCommon[i].dAbsolutePosFromMachineZeroPoint = 0.0;
            MotorCommon[i].dRequiredPosAbsolute = 0.0; // New required position (Startposition for next move command)
            MotorCommon[i].DigitalOutputMotor = 0b00000000; // Switch step bits off
            MotorCommon[i].StepNoPositioning = StepPositioningRunReferenceSensorFreeFast;
            MotorAll.RotationG68IsActive = false;
            MotorAll.RotationG68CX = 0.0;
            MotorAll.RotationG68CY = 0.0;
            MotorAll.RotationG68Angle = 0.0;

            // Delaytime release next steps
            MotorCommon[i].DelaytimeMoveCommand = false;
            MotorCommon[i].CounterDelaytimeMoveCommand = 0;
            MotorCommon[i].ValueDelaytimeMoveCommand = WaitMoveCommand;
          }

          // Reset tags for stop movement if release movement is gone
          MotorAll.StepNoStopMovement = 0;
          MotorAll.lCounterStepsStopMovement = 0;
          MotorAll.SpindleWasOnStopMovement = false;

          // Unlatch tag first cycle movement
          FirstCycleMovement = false;

          break; // case G74 Referenzfahrt über Referenzfühler anfahren

        // ##### G90 Absolutmaßprogrammierung #########################################################################
        case CodeG90:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G90");
          }

          // ##### Absolutmaßprogrammierung
          MotorAll.AbsoluteIsActive = true;
          MotorAll.RelativeIsActive = false;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### G91 Kettenmaßprogrammierung #########################################################################
        case CodeG91:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command G91");
          }

          // ##### Kettenmaßprogrammierung
          MotorAll.AbsoluteIsActive = false;
          MotorAll.RelativeIsActive = true;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### M03 Spindle on, clockwise #########################################################################
        case CodeM03:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M03");
          }

          // ##### Spindle on
          MotorAll.SpindleOn = true;

          // ##### Transfer speed
          MotorAll.dRequiredSpindleSpeed = ActualStepData.dValue[0];
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;   
          break;

        // ##### M05 Spindle off #########################################################################
        case CodeM05:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M05");
          }

          // ##### Spindle off
          MotorAll.SpindleOn = false;

          // ##### Transfer speed
          MotorAll.dRequiredSpindleSpeed = SpeedSpindleMin;

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;    
          break;

        // ##### M08 Coolant on #########################################################################
        case CodeM08:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M08");
          }

          // ##### Coolant on
          MotorAll.CoolantOn = true;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;   
          break;

        // ##### M09 Coolant off #########################################################################
        case CodeM09:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M09");
          }

          // ##### Coolant off
          MotorAll.CoolantOn = false;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;    
          break;

        // ##### M10 Clamp on #########################################################################
        case CodeM10:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M10");
          }

          // ##### Clamp on
          MotorAll.ClampOn = true;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;   
          break;

        // ##### M11 Clamp off #########################################################################
        case CodeM11:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M11");
          }

          // ##### Clamp off
          MotorAll.ClampOn = false;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;    
          break;

        // ##### M35 Light on #########################################################################
        case CodeM35:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M35");
          }

          // ##### Light on
          MotorAll.LightOn = true;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;   
          break;

        // ##### M36 Light off #########################################################################
        case CodeM36:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M36");
          }

          // ##### Light off
          MotorAll.LightOn = false;
          
          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;    
          break;

        // ##### M02 / M30 Programmende #########################################################################
        // ##### M02 Programmende
        // ##### M30 Programmende mit Ruecksprung auf Programmanfang
        // Just do nothing, there will be a case with "CodeLastStep"
        case CodeM02:
        case CodeM30:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            if(ActualStepData.Code == CodeM02)
            {
              Serial.println("##### Command M02");
            }
            else
            {
              Serial.println("##### Command M30");
            }
          }

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### M98 Subprogram call #########################################################################
        case CodeM98:
          // #####Values loded during program transfer:
          // Subprogram.AdressPointerFirstStep[i]; // Loaded during program transfer. Adress pointer in the Array SubProgramData[7500] where the subprogram starts (nesting)
          // Subprogram.ProgramNo[i]; // Loaded during program transfer. Program no. of the subprogram, parameter P (M98 P1000 L5)
          
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M98");
          }

          // ##### Increase index active subprogram no. for nesting, when a subprogram is active
          if(Subprogram.IsActive == true)
          {
            Subprogram.IndexActiveSubprogram +=1;
          }
          else
          {
            Subprogram.IndexActiveSubprogram = 0;
          }

          // ##### Set values
          Subprogram.IsActive = true;
          Subprogram.AdressPointerStepBack[Subprogram.IndexActiveSubprogram] = Subprogram.IndexActualStepNo; // Adress pointer in the Array SubProgramData[7500] step back when subprogram ends (nesting)
          Subprogram.ActiveProgramNo = int(ActualStepData.dValue[0]);  // Program no. of the subprogram, parameter P (M98 P1000 L5)
          Subprogram.CounterNumberOfRepeats[Subprogram.IndexActiveSubprogram] = int(ActualStepData.dValue[1]); // Number of repeats, parameter L (M98 P1000 L5)
          Subprogram.ActualRepeats[Subprogram.IndexActiveSubprogram] = 0;

          // ##### Load first step of the subprogram as actual step 
          // Find satrtadress in array "SubProgramData"
          for(i=0; i<MaxSubprograms; i++)
          {
            if(Subprogram.ActiveProgramNo == Subprogram.ProgramNo[i])
            {
              Subprogram.IndexActualStepNo = Subprogram.AdressPointerFirstStep[i]; // Load first step of the subprogram as actual step in the Array SubProgramData[7500]
              Subprogram.AdressPointerFirstStepRepeats[Subprogram.IndexActiveSubprogram] = Subprogram.IndexActualStepNo;
              break;
            }
          }

          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("M98 P / L");
            Serial.println(Subprogram.ActiveProgramNo, DEC);
            Serial.println(Subprogram.CounterNumberOfRepeats[Subprogram.IndexActiveSubprogram], DEC);
            Serial.print("M98 FirstStep ");
            Serial.println(Subprogram.IndexActualStepNo, DEC);
            Serial.print("M98 Index ");
            Serial.println(Subprogram.IndexActiveSubprogram, DEC);
          }

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### M99 Subprogram end #########################################################################
        case CodeM99:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command M99");
          }

          // ##### Counter repeats of subprogram
          Subprogram.ActualRepeats[Subprogram.IndexActiveSubprogram] +=1;

          // ##### Amount repeats reached
          if(Subprogram.ActualRepeats[Subprogram.IndexActiveSubprogram] >= Subprogram.CounterNumberOfRepeats[Subprogram.IndexActiveSubprogram])
          {
            Subprogram.ActualRepeats[Subprogram.IndexActiveSubprogram] = 0;

            // Index subprogram <= 0 amounts reached, end subprograms
            if(Subprogram.IndexActiveSubprogram <= 0)
            {
              Subprogram.IndexActiveSubprogram = 0;
              Subprogram.IsActive = false;
            }
            else
            {
              // Load step back, step of the calling subprogram as actual step 
              Subprogram.IndexActualStepNo = Subprogram.AdressPointerStepBack[Subprogram.IndexActiveSubprogram];
              // Decrese index active subprogram
              Subprogram.IndexActiveSubprogram -=1;
            }
          }
          // ##### Amount repeats not reached, start same subprogram again
          else
          {
            // Load first step of the same subprogram as actual step 
            Subprogram.IndexActualStepNo = Subprogram.AdressPointerFirstStepRepeats[Subprogram.IndexActiveSubprogram];
          }

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### Code O Subprogram number #########################################################################
        case CodeO:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command O");
          }

          // ##### Values loded during program transfer:
          // Subprogram.AdressPointerFirstStep[i]; // Loaded during program transfer. Adress pointer in the Array SubProgramData[7500] where the subprogram starts (nesting)
          // Subprogram.ProgramNo[i]; // Loaded during program transfer. Program no. of the subprogram, parameter P (M98 P1000 L5)

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### CodeLastStep #########################################################################
        // Last step to detect end of file
        case CodeLastStep:
          // ##### DebugMode, print values
          if (DEBUGMODE == true)
          {
            Serial.println("##### Command CodeLastStep");
          }

          // ##### Set values
          MotorAll.AbsoluteIsActive = true;
          MotorAll.RelativeIsActive = false;
          MotorSend.ProgramDone = true;
          IndexActualStepNo = 0;
          MotorSend.ActualStepProgram = IndexActualStepNo;
          Subprogram.IndexActualStepNo = 0;
          Subprogram.IndexActiveSubprogram = 0;
          Subprogram.IsActive = false;
          MotorAll.SpindleOn = false;
          MotorAll.CoolantOn = false;
          MotorAll.ClampOn = false;
          MotorAll.LightOn = false;

          // ##### Command done
          MotorAll.StepNo = StepHandshakeCommandDone;
          break;

        // ##### Default
        default:
          break;
      } // switch (ActualStepData.Code)
    
    break; // case StepNoAction:

    // ##### Positioning #############################################################################
    case StepPositioningMain:
    /*
    Value 0 = Order X (1, 2 or 3)
    Value 1 = Order Y (1, 2 or 3)
    Value 2 = Order Z (1, 2 or 3)
    
    Value 0 = 1 -> X1 e.g. X and Y at the same time, then Z
    Value 1 = 1 -> Y1
    Value 2 = 2 -> Z2
    PosOrder 1200 PosAmount 2 1 0 0   PosOrder 0/0=1 0/1=2 0/2=0 0/3=0
             3000                              1/0=3 1/1=0 1/2=0 1/3=0
             0000                              2/0=0 2/1=0 2/2=0 2/3=0
             0000                              3/0=0 3/1=0 3/2=0 3/3=0
    */

      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          Serial.print("Amount in pos. order 0  "); 
          Serial.println(PosAmount[0], DEC);
          Serial.print("Amount in pos. order 1  "); 
          Serial.println(PosAmount[1], DEC);
          Serial.print("Amount in pos. order 2  "); 
          Serial.println(PosAmount[2], DEC);
          Serial.print("Amount in pos. order 3  "); 
          Serial.println(PosAmount[3], DEC);

          //Serial.print("Motors in pos. order 0  "); 
          //Serial.println(PosOrder[0][0], DEC);
          //Serial.println(PosOrder[0][1], DEC);
          //Serial.println(PosOrder[0][2], DEC);
          //Serial.println(PosOrder[0][3], DEC);

          //Serial.print("Motors in pos. order 1  "); 
          //Serial.println(PosOrder[1][0], DEC);
          //Serial.println(PosOrder[1][1], DEC);
          //Serial.println(PosOrder[1][2], DEC);
          //Serial.println(PosOrder[1][3], DEC);

          //Serial.print("Motors in pos. order 2  "); 
          //Serial.println(PosOrder[2][0], DEC);
          //Serial.println(PosOrder[2][1], DEC);
          //Serial.println(PosOrder[2][2], DEC);
          //Serial.println(PosOrder[2][3], DEC);

          //Serial.print("Motors in pos. order 3  "); 
          //Serial.println(PosOrder[3][0], DEC);
          //Serial.println(PosOrder[3][1], DEC);
          //Serial.println(PosOrder[3][2], DEC);
          //Serial.println(PosOrder[3][3], DEC);
        }
      }

      switch (MotorAll.StepPositioningStepOrder)
      {
        // #### Position all the motors in first pos order
        case 0:
          if(PosAmount[0] >0)
          {
            PositioningDone = true;
            for (i = 0; i < PosAmount[0]; i++)
            {
              k = PosOrder[0][i] -1; // k = Motor number, 0..3
              if (MotorCommon[k].PositioningDone == false)
              {
                // Call function positioning, Motor no. / direction / speed / release movement
                Positioning(k, DirPos[k], MotorAll.lSpeedInProgramscansPerStep, bReleaseMovementCommon );
  
                // Check if positioning for this motor is done
                MotorCommon[k].PositioningIsActive = true;
                if (MotorCommon[k].PositioningDone == true)
                {
                  MotorCommon[k].PositioningIsActive = false;
                }
                
                // Check if positioning for this motor is still active
                if (MotorCommon[k].PositioningDone == false)
                {
                  PositioningDone = false;
                }
              }
            }

            // Al motors in this pos order are positioned
            if(PositioningDone == true)
            {
              MotorAll.StepPositioningStepOrder = 1;
              if(DEBUGMODE == true)
              {
                Serial.println("PosOrder 0 done");
              }
            }

          } // PosAmount > 0
          else
          {
            MotorAll.StepPositioningStepOrder = 1;
            if(DEBUGMODE == true)
            {
              Serial.println("No motors in PosOrder 0");
            }
          }
         
          break;

        // #### Position all the motors in second pos order
        case 1:
          if(PosAmount[1] >0)
          {
            PositioningDone = true;
            for (i = 0; i < PosAmount[1]; i++)
            {
              k = PosOrder[1][i] -1; // k = Motor number, 0..3
              if (MotorCommon[k].PositioningDone == false)
              {
                // Call function positioning, Motor no. / direction / speed / release movement
                Positioning(k, DirPos[k], MotorAll.lSpeedInProgramscansPerStep, bReleaseMovementCommon );
  
                // Check if positioning for this motor is done
                MotorCommon[k].PositioningIsActive = true;
                if (MotorCommon[k].PositioningDone == true)
                {
                  MotorCommon[k].PositioningIsActive = false;
                }
                
                // Check if positioning for this motor is still active
                if (MotorCommon[k].PositioningDone == false)
                {
                  PositioningDone = false;
                }
              }
            }
            // All motors in this positioning order are positioned
            if(PositioningDone == true)
            {
              MotorAll.StepPositioningStepOrder = 2;
              if(DEBUGMODE == true)
              {
                Serial.println("PosOrder 1 done");
              }
            }

          } // PosAmount > 0
          else
          {
            MotorAll.StepPositioningStepOrder = 2;
            if(DEBUGMODE == true)
            {
              Serial.println("No motors in PosOrder 1");
            }
          }
          
          break;

        // #### Position all the motors in third pos order
        case 2:
          if(PosAmount[2] >0)
          {
            PositioningDone = true;
            for (i = 0; i < PosAmount[2]; i++)
            {
              k = PosOrder[2][i] -1; // k = Motor number, 0..3
              if (MotorCommon[k].PositioningDone == false)
              {
                // Call function positioning, Motor no. / direction / speed / release movement
                Positioning(k, DirPos[k], MotorAll.lSpeedInProgramscansPerStep, bReleaseMovementCommon );
  
                // Check if positioning for this motor is done
                MotorCommon[k].PositioningIsActive = true;
                if (MotorCommon[k].PositioningDone == true)
                {
                  MotorCommon[k].PositioningIsActive = false;
                }
                
                // Check if positioning for this motor is still active
                if (MotorCommon[k].PositioningDone == false)
                {
                  PositioningDone = false;
                }
              }
            }
            // All motors in this positioning order are positioned
            if(PositioningDone == true)
            {
              MotorAll.StepPositioningStepOrder = 3;
              if(DEBUGMODE == true)
              {
                Serial.println("PosOrder 2 done");
              }
            }

          } // PosAmount > 0
          else
          {
            MotorAll.StepPositioningStepOrder = 3;
            if(DEBUGMODE == true)
            {
              Serial.println("No motors in PosOrder 2");
            }
          }
          
          break;

        // #### Position all the motors in fourth pos order
        case 3:
          if(PosAmount[3] >0)
          {
            PositioningDone = true;
            for (i = 0; i < PosAmount[3]; i++)
            {
              k = PosOrder[3][i] -1; // k = Motor number, 0..3
              if (MotorCommon[k].PositioningDone == false)
              {
                // Call function positioning, Motor no. / direction / speed / release movement
                Positioning(k, DirPos[k], MotorAll.lSpeedInProgramscansPerStep, bReleaseMovementCommon );
  
                // Check if positioning for this motor is done
                MotorCommon[k].PositioningIsActive = true;
                if (MotorCommon[k].PositioningDone == true)
                {
                  MotorCommon[k].PositioningIsActive = false;
                }
                
                // Check if positioning for this motor is still active
                if (MotorCommon[k].PositioningDone == false)
                {
                  PositioningDone = false;
                }
              }
            }
            // All motors in this positioning order are positioned, positioning is finished
            if(PositioningDone == true)
            {
              for (i = 0; i < AmountOfSteppermotors; i++)
              {
                MotorCommon[i].PositioningIsActive = false;
              }
              MotorAll.PositioningIsActive = false;
              MotorAll.StepNo = StepHandshakeCommandDone;
              if(DEBUGMODE == true)
              {
                Serial.println("PosOrder 3 done");
              }
            }

          } // PosAmount > 0
          else
          {
            for (i = 0; i < AmountOfSteppermotors; i++)
            {
              MotorCommon[i].PositioningIsActive = false;
            }
            MotorAll.PositioningIsActive = false;
            MotorAll.StepNo = StepHandshakeCommandDone;
            if(DEBUGMODE == true)
            {
              Serial.println("No motors in PosOrder 3");
            }
          }

          break;

        default:
          break;
      } // switch MotorAll.StepPositioningStepOrder

      break; // case StepPositioningMain:

    // ##### Linear movement #########################################################################
    case StepLinearMovement:

      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          // Clear debugvalues
          DebugValues.StepsInStartramp = 0;
          DebugValues.StepsInLinear = 0;
          DebugValues.StepsInStopramp = 0;

          // Print values
          //Serial.print("dActualSpeed "); 
          //Serial.println(MotorAll.dActualSpeed, DEC); // 600 (600.0mm/min)
          //Serial.print("lActualValueRamp "); 
          //Serial.println(MotorAll.lActualValueRamp, DEC); // 44 (Rampe 20 + Speed 24)
          Serial.print("lSpeedInProgramscansPerStep "); 
          Serial.println(MotorAll.lSpeedInProgramscansPerStep, DEC); // 24 (Bei speed 600.0mm/min)
          
          //Serial.print("lAmountSteps "); 
          //Serial.println(MotorAll.lAmountSteps, DEC); // ok 533 (bei 4.0mm Weg)
          //Serial.print("X ");
          //Serial.println(MotorAll.dDistanceAbsPerStepX, DEC); // ok 0.0075 (bei 4.0mm Weg)
          //Serial.print("Y ");
          //Serial.println(MotorAll.dDistanceAbsPerStepY, DEC); // ok 0.0075 (bei 4.0mm Weg)
          //Serial.print("Z ");
          //Serial.println(MotorAll.dDistanceAbsPerStepZ, DEC); // ok 0.0 (bei 0.0mm Weg)
        }
      }

      // ##### Stop movement if release movement is gone
      switch (MotorAll.StepNoStopMovement)
      {
        // No release movement, go to stopramp
        // When already in stopramp do nothing here.
        case 0:
          if(bReleaseMovementCommon == false && MotorAll.Stopramp == false && MotorAll.lCounterRamp == 0)
          {
            MotorAll.StepNoStopMovement = 1;
            // Go to stopramp
            MotorAll.Startramp = false;
            MotorAll.Linear = false;
            MotorAll.Stopramp = true;
            // Transfer values for stopramp
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
            MotorAll.lCounterStepsStopMovement = 0;

            // Stop without ramp and switch spindle off
            if(MotorAll.MovementWithoutRamp == true)
            {
              MotorAll.StepNoStopMovement = 2; // Wait for release
              // Switch spindle off
              if(MotorAll.SpindleOn == true)
              {
                MotorAll.SpindleOn = false;
                MotorAll.SpindleWasOnStopMovement = true;
              }
            }
          }
          break;    

        // Stopramp active, count steps in stopramp
        case 1:
          if(MotorAll.lCounterStepsStopMovement >= MotorAll.lAmountStepsRamp)
          {
            MotorAll.StepNoStopMovement = 2;
            MotorAll.EnableStep = false; // Switch enable step off and wait for next release
            // Switch spindle off
            if(MotorAll.SpindleOn == true)
            {
              MotorAll.SpindleOn = false;
              MotorAll.SpindleWasOnStopMovement = true;
            }
          }
          break; 

        // Wait for next release
        case 2:
          if(bReleaseMovementCommon == true)
          {
            MotorAll.StepNoStopMovement = 3;
            // Start delaytime with 1ms
            MotorCommon[0].DelaytimeMoveCommand  = false;
            MotorCommon[0].CounterDelaytimeMoveCommand = 0;
            MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;

            // Switch spindle on, set delaytime to 2000ms
            if(MotorAll.SpindleWasOnStopMovement == true)
            {
              MotorAll.SpindleOn = true;
              MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
            }

          }
          break; 

        // Start movement again
        case 3:
          if (MotorCommon[0].DelaytimeMoveCommand  == true)
          {
            MotorAll.StepNoStopMovement = 0;
            // Calculations to start movement G00/G01 again
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep + MotorAll.lAmountStepsRamp;
            MotorAll.lMaxStepRamp = ((MotorAll.lAmountSteps - MotorAll.lStepcounter) / 2) + MotorAll.lStepcounter; // Calculate max step startramp again
            // Start again with startramp or linear movement
            MotorAll.Stopramp = false;
            if(MotorAll.MovementWithoutRamp == true)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = true;
            }
            else
            {
              MotorAll.Startramp = true;
              MotorAll.Linear = false;
            }
          }
          break; 

        default:
          break;
      }

      // ##### Motor is stopped, wait for next release
      if(MotorAll.StepNoStopMovement == 2 || MotorAll.StepNoStopMovement == 3)
      {
        break;
      }

      // ##### Is delaytime move command elapsed (direction bit is set in prepare move command before)
      if (MotorCommon[0].DelaytimeMoveCommand  == false)
      {
        break;
      }

      // ##### Amount steps not reached
      if(MotorAll.lStepcounter <= MotorAll.lAmountSteps)
      {
        // ##### Startramp
        if(MotorAll.Startramp == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lActualValueRamp)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp -= 1;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            DebugValues.StepsInStartramp++;

            // Startramp done
            if(MotorAll.lActualValueRamp <= MotorAll.lSpeedInProgramscansPerStep)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = true;
              MotorAll.Stopramp = false;
              break;
            }

            // Startramp is to short
            if(MotorAll.lStepcounter >= MotorAll.lMaxStepRamp)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = false;
              MotorAll.Stopramp = true;
              MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
              break;
            }
          }
        }

        // ##### Linear
        if(MotorAll.Linear == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lSpeedInProgramscansPerStep)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            DebugValues.StepsInLinear++;

           // Linerar done
           if(MotorAll.lStepcounter >= (MotorAll.lAmountSteps - MotorAll.lAmountStepsRamp) and MotorAll.MovementWithoutRamp == false)
           {
            MotorAll.Startramp = false;
            MotorAll.Linear = false;
            MotorAll.Stopramp = true;
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
            break;   
           }
          }
        }

        // ##### Stopramp
        if(MotorAll.Stopramp == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lActualValueRamp)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp += 1;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            MotorAll.lCounterStepsStopMovement++;
            DebugValues.StepsInStopramp++;
          }
        }

        // ##### Reset output step
        for (i = 0; i < AmountOfSteppermotors; i++)
        {
          if(MotorCommon[i].ResetStep == true)
          {
            Step(i); // Step to reset output axis
            MotorCommon[i].ResetStep = false;
          }
        }

        // ##### Do Step
        // MotorAll.lStepcounter = 1...x
        if(MotorAll.DoStep == true)
        {
          MotorAll.DoStep = false;

          // Calculate required position
          MotorAll.dRequiredPosX = (double)MotorAll.lStepcounter * MotorAll.dDistanceAbsPerStepX; // dDistanceAbsPerStepX is always positive
          MotorAll.dRequiredPosY = (double)MotorAll.lStepcounter * MotorAll.dDistanceAbsPerStepY; // dDistanceAbsPerStepY is always positive
          MotorAll.dRequiredPosZ = (double)MotorAll.lStepcounter * MotorAll.dDistanceAbsPerStepZ; // dDistanceAbsPerStepZ is always positive

          // Step X
          if((MotorAll.dInternalDistancecounterX + MM_Step) <= MotorAll.dRequiredPosX)
          {
             Step(0); // Step X-axis
             MotorCommon[0].ResetStep = true ;
             MotorAll.dInternalDistancecounterX += MM_Step;
             if( MotorCommon[0].StepsPositiveIsActive == true)
             {
              MotorCommon[0].dAbsolutePosMotor += MM_Step;
             }
             else
             {
              MotorCommon[0].dAbsolutePosMotor -= MM_Step;
             }
          }
          
          // Step Y
          if((MotorAll.dInternalDistancecounterY + MM_Step) <= MotorAll.dRequiredPosY)
          {
             Step(1); // Step Y-axis
             MotorCommon[1].ResetStep = true ;
             MotorAll.dInternalDistancecounterY += MM_Step;
             if( MotorCommon[1].StepsPositiveIsActive == true)
             {
              MotorCommon[1].dAbsolutePosMotor += MM_Step;
             }
             else
             {
              MotorCommon[1].dAbsolutePosMotor -= MM_Step;
             }
          }

          // Step Z
          if((MotorAll.dInternalDistancecounterZ + MM_Step) <= MotorAll.dRequiredPosZ)
          {
             Step(2); // Step Z-axis
             MotorCommon[2].ResetStep = true ;
             MotorAll.dInternalDistancecounterZ += MM_Step;
             if( MotorCommon[2].StepsPositiveIsActive == true)
             {
              MotorCommon[2].dAbsolutePosMotor += MM_Step;
             }
             else
             {
              MotorCommon[2].dAbsolutePosMotor -= MM_Step;
             }
          }
        
        }
      } // if(MotorAll.lStepcounter <= MotorAll.lAmountSteps)

      // Amount steps reached, movement done
      else
      {

        // ##### End movement
        // ##### Or stop spindle, when movement was already in stopramp and release movement is gone
        switch (MotorAll.StepNoStopSpindle)
        {
          // Release movement is active, end movement
          case 0:
            if(bReleaseMovementCommon == true)
            {
              MotorAll.StepNo = StepHandshakeCommandDone;
      
              // DebugMode, print values
              if(DEBUGMODE == true)
              {
                Serial.print("Steps in Startramp ");
                Serial.println(DebugValues.StepsInStartramp, DEC);
                Serial.print("Steps in Linear ");
                Serial.println(DebugValues.StepsInLinear, DEC);
                Serial.print("Steps in Stopramp ");
                Serial.println(DebugValues.StepsInStopramp, DEC);
              }
            }
            // No release movement, stop spindle
            else
            {
              MotorAll.StepNoStopSpindle = 1;
  
              // Switch spindle off
              if(MotorAll.SpindleOn == true)
              {
                MotorAll.SpindleOn = false;
              }
              else // Spindle was not on, end movement
              {
                MotorAll.StepNoStopSpindle = 10;
              }
            }
            break; 

          // Wait for next release
          case 1:
            if(bReleaseMovementCommon == true)
            {
              MotorAll.StepNoStopSpindle = 2;
              // Start delaytime with 2000ms
              MotorCommon[0].DelaytimeMoveCommand  = false;
              MotorCommon[0].CounterDelaytimeMoveCommand = 0;
              MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
              // Switch spindle on
              MotorAll.SpindleOn = true;
            }
            break; 

          // Delaytime elapsed
          case 2:
            if (MotorCommon[0].DelaytimeMoveCommand  == true)
            {
              MotorAll.StepNoStopSpindle = 10;
            }
            break;

          // End movement
          case 10:
            MotorAll.StepNo = StepHandshakeCommandDone;
    
            // DebugMode, print values
            if(DEBUGMODE == true)
            {
              Serial.print("Steps in Startramp ");
              Serial.println(DebugValues.StepsInStartramp, DEC);
              Serial.print("Steps in Linear ");
              Serial.println(DebugValues.StepsInLinear, DEC);
              Serial.print("Steps in Stopramp ");
              Serial.println(DebugValues.StepsInStopramp, DEC);
            }
            break; 
            
          default:
            break;
        }

      }

      break; // case StepLinearMovement:

    // ##### Circular movement #######################################################################
    case StepCircularMovement:
    /*
    Wert 0 = X mit vier Nachkommastellen (Endposition X, Absolut oder Relativ)
    Wert 1 = Y mit vier Nachkommastellen (Endposition Y, Absolut oder Relativ)
    Wert 2 = I mit vier Nachkommastellen (Kreismittelpunkt X, immer Relativ)
    Wert 3 = J mit vier Nachkommastellen (Kreismittelpunkt Y, immer Relativ)
    Wert 4 = F mit vier Nachkommastellen (Vorschub in mm/min) (99999999 = ungültiger Wert)

    Example: G03 Kreisinterpolation, gegen Uhrzeigersinn
    Startpunkt: X43,301  Y25
    G03 X25,0  Y43,301  I-43,301  J25,0 F600
    
    MotorAll.dActualSpeed = 600
    MotorAll.lActualValueRamp = 44
    MotorAll.lSpeedInProgramscansPerStep = 24
    MotorAll.dStartAngle = 30.0
    MotorAll.dEndAngle = 60.0
    MotorAll.dRadius = 50.0
    MotorAll.dDistanceAngle = 30.0
    MotorAll.dAngleStep = 0.0085
    MotorAll.CenterCircleX = 0.0
    MotorAll.CenterCircleY = 0.0
    MotorAll.dInternalAnglecounter = 0, Startet immer bei 0, und zählt dann z.B. von 0..30
    MotorAll.dActualAngle = 30, Startet immer mit dStartAngle z.B. 30, und zählt dann z.B. von 30..60
    MotorAll.dStartrampEndAngle = 0.17 [z.B. 0.0085 * 20] MotorAll.dAngleStep * double(MotorAll.lAmountStepsRamp);
    MotorAll.dMaxStepRamp = 15, immer dDistanceAngle / 2
    MotorAll.dStoprampStartAngle = 29.83 [z.B. 30.0 - 0.17] MotorAll.dStoprampStartAngle = MotorAll.dDistanceAngle - MotorAll.dDistanceAngleRamp;
    */

      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          // Clear debugvalues
          DebugValues.StepsInStartramp = 0;
          DebugValues.StepsInLinear = 0;
          DebugValues.StepsInStopramp = 0;

          // Print values
          //Serial.print("dActualSpeed "); 
          //Serial.println(MotorAll.dActualSpeed, DEC); // 600 (600.0mm/min)
          //Serial.print("lActualValueRamp "); 
          //Serial.println(MotorAll.lActualValueRamp, DEC); // 44 (Rampe 20 + Speed 24)
          //Serial.print("lSpeedInProgramscansPerStep "); 
          //Serial.println(MotorAll.lSpeedInProgramscansPerStep, DEC); // 24 (Bei speed 600.0mm/min)

          Serial.print("Startangle "); 
          Serial.println(MotorAll.dStartAngle, DEC); // 30
          Serial.print("Endangle "); 
          Serial.println(MotorAll.dEndAngle, DEC); // 60
          Serial.print("Radius "); 
          Serial.println(MotorAll.dRadius, DEC); // 50

          Serial.print("Distance angle "); 
          Serial.println(MotorAll.dDistanceAngle, DEC); // 30
          //Serial.print("Angle step "); 
          //Serial.println(MotorAll.dAngleStep, DEC); // 0.0085

          //Serial.print("G02IsActive "); 
          //Serial.println(MotorAll.G02IsActive, DEC); // 0
          //Serial.print("G03IsActive "); 
          //Serial.println(MotorAll.G03IsActive, DEC); // 1

          //Serial.print("CenterCircleX "); 
          //Serial.println(MotorAll.CenterCircleX, DEC); // 0
          //Serial.print("CenterCircleY "); 
          //Serial.println(MotorAll.CenterCircleY, DEC); // 0

          //Serial.print("Internal anglecounter "); 
          //Serial.println(MotorAll.dInternalAnglecounter, DEC); // 0
          //Serial.print("Actual angle "); 
          //Serial.println(MotorAll.dActualAngle, DEC); // 30

          //Serial.print("Startramp end angle "); 
          //Serial.println(MotorAll.dStartrampEndAngle, DEC); // 0.17
          //Serial.print("MaxStepRamp "); 
          //Serial.println(MotorAll.dMaxStepRamp, DEC); // 15
          //Serial.print("Stopramp start angle "); 
          //Serial.println(MotorAll.dStoprampStartAngle, DEC); // 29.83
        }
      }

      // ##### Stop movement if release movement is gone
      switch (MotorAll.StepNoStopMovement)
      {
        // No release movement, go to stopramp
        // When already in stopramp do nothing here.
        case 0:
          if(bReleaseMovementCommon == false && MotorAll.Stopramp == false && MotorAll.lCounterRamp == 0)
          {
            MotorAll.StepNoStopMovement = 1;
            // Go to stopramp
            MotorAll.Startramp = false;
            MotorAll.Linear = false;
            MotorAll.Stopramp = true;
            // Transfer values for stopramp
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
            MotorAll.lCounterStepsStopMovement = 0;
            
            // Stop without ramp and switch spindle off
            if(MotorAll.MovementWithoutRamp == true)
            {
              MotorAll.StepNoStopMovement = 2; // Wait for release
              // Switch spindle off
              if(MotorAll.SpindleOn == true)
              {
                MotorAll.SpindleOn = false;
                MotorAll.SpindleWasOnStopMovement = true;
              }
            }
          }
          break;    

        // Stopramp active, count steps in stopramp
        case 1:
          if(MotorAll.lCounterStepsStopMovement >= MotorAll.lAmountStepsRamp)
          {
            MotorAll.StepNoStopMovement = 2;
            MotorAll.EnableStep = false; // Switch enable step off and wait for next release
            // Switch spindle off
            if(MotorAll.SpindleOn == true)
            {
              MotorAll.SpindleOn = false;
              MotorAll.SpindleWasOnStopMovement = true;
            }
          }
          break; 

        // Wait for next release
        case 2:
          if(bReleaseMovementCommon == true)
          {
            MotorAll.StepNoStopMovement = 3;
            // Start delaytime with 1ms
            MotorCommon[0].DelaytimeMoveCommand  = false;
            MotorCommon[0].CounterDelaytimeMoveCommand = 0;
            MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;

            // Switch spindle on, set delaytime to 2000ms
            if(MotorAll.SpindleWasOnStopMovement == true)
            {
              MotorAll.SpindleOn = true;
              MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
            }
            
          }
          break; 

        // Start movement again
        case 3:
          if (MotorCommon[0].DelaytimeMoveCommand  == true)
          {
            MotorAll.StepNoStopMovement = 0;
            // Calculations to start movement G02/G03 again
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep + MotorAll.lAmountStepsRamp;
            MotorAll.dMaxStepRamp = ((MotorAll.dDistanceAngle - MotorAll.dInternalAnglecounter) / 2.0) +  MotorAll.dInternalAnglecounter; // Calculate max step startramp again
            // Start again with startramp or linear movement
            MotorAll.Stopramp = false;
            if(MotorAll.MovementWithoutRamp == true)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = true;
            }
            else
            {
              MotorAll.Startramp = true;
              MotorAll.Linear = false;
            }
          }
          break; 

        default:
          break;
      }

      // ##### Motor is stopped, wait for next release
      if(MotorAll.StepNoStopMovement == 2 || MotorAll.StepNoStopMovement == 3)
      {
        break;
      }

      // ##### Is delaytime move command elapsed (direction bit is set in prepare move command before)
      if (MotorCommon[0].DelaytimeMoveCommand  == false)
      {
        break;
      }

      // ##### Distance angle not reached
      if(MotorAll.dInternalAnglecounter <= MotorAll.dDistanceAngle)
      {
        // ##### Startramp
        if(MotorAll.Startramp == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lActualValueRamp)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp -= 1;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            DebugValues.StepsInStartramp++;

            // Startramp done
            if(MotorAll.lActualValueRamp <= MotorAll.lSpeedInProgramscansPerStep)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = true;
              MotorAll.Stopramp = false;
              break;
            }

            // Startramp is to short
            if(MotorAll.dInternalAnglecounter >= MotorAll.dMaxStepRamp)
            {
              MotorAll.Startramp = false;
              MotorAll.Linear = false;
              MotorAll.Stopramp = true;
              MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
              break;
            }
          }
        }

        // ##### Linear
        if(MotorAll.Linear == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lSpeedInProgramscansPerStep)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            DebugValues.StepsInLinear++;

           // Linerar done
           if(MotorAll.dInternalAnglecounter >= MotorAll.dStoprampStartAngle and MotorAll.MovementWithoutRamp == false)
           {
            MotorAll.Startramp = false;
            MotorAll.Linear = false;
            MotorAll.Stopramp = true;
            MotorAll.lActualValueRamp = MotorAll.lSpeedInProgramscansPerStep;
            break;   
           }
          }
        }

        // ##### Stopramp
        if(MotorAll.Stopramp == true)
        {
          MotorAll.lCounterRamp ++;
          if (MotorAll.lCounterRamp >= MotorAll.lActualValueRamp)
          {
            MotorAll.lCounterRamp = 0;
            MotorAll.lActualValueRamp += 1;
            MotorAll.DoStep = true;
            MotorAll.lStepcounter++;
            MotorAll.lCounterStepsStopMovement++;
            DebugValues.StepsInStopramp++;
          }
        }

        // ##### Reset output step
        for (i = 0; i < AmountOfSteppermotors; i++)
        {
          if(MotorCommon[i].ResetStep == true)
          {
            Step(i); // Step to reset output axis
            MotorCommon[i].ResetStep = false;
          }
        }

        // ##### Do Step
        if(MotorAll.DoStep == true)
        {
          MotorAll.DoStep = false;

          // While...
          bNextStepCircular = false;
          while(bNextStepCircular == false)
          {
            // Anglecounter
            //              90
            //      180             0
            //             270
            // Fictive startpoint circle is 0deg
            // Therefore G03 (counterclockwise) count angle +
            // Therefore G02 (clockwise) count angle -
            if(MotorAll.G03IsActive == true)
            {
              MotorAll.dInternalAnglecounter += MotorAll.dAngleStep;
              MotorAll.dActualAngle += MotorAll.dAngleStep;
              if(MotorAll.dActualAngle >= 360.0)
              {
                MotorAll.dActualAngle -= 360.0;
              }
            }
            else
            {
              MotorAll.dInternalAnglecounter += MotorAll.dAngleStep;
              MotorAll.dActualAngle -= MotorAll.dAngleStep;
              if(MotorAll.dActualAngle < 0.0)
              {
                MotorAll.dActualAngle += 360.0;
              }             
            }

            // Which quadrant is active
            // Quadrant 1 0 - 90deg
            // Quadrant 2 90 - 180deg
            // Quadrant 3 180 - 270deg
            // Quadrant 4 270 - 360deg
            if(MotorAll.dActualAngle >=0.0 and MotorAll.dActualAngle < 90.0)
            {
              iActiveQuadrant = 1;
            }
            if(MotorAll.dActualAngle >=90.0 and MotorAll.dActualAngle < 180.0)
            {
              iActiveQuadrant = 2;
            }
            if(MotorAll.dActualAngle >=180.0 and MotorAll.dActualAngle < 270.0)
            {
              iActiveQuadrant = 3;
            }
            if(MotorAll.dActualAngle >=270.0 and MotorAll.dActualAngle <= 360.0)
            {
              iActiveQuadrant = 4;
            }

            // Calculate required position
            // X= COS(Bogenmaß) * Radius + KreismittelpunktX
            // Y= SIN(Bogenmaß) * Radius + KreismittelpunktY
            dTmp = MotorAll.dActualAngle * Deg_To_Rad;
            MotorAll.dRequiredPosX = (cos(dTmp) * MotorAll.dRadius) + MotorAll.CenterCircleX;
            MotorAll.dRequiredPosY = (sin(dTmp) * MotorAll.dRadius) + MotorAll.CenterCircleY;

            // Do step X-
            if(iActiveQuadrant == 1 or iActiveQuadrant == 2)
            {
              if((MotorCommon[0].dAbsolutePosMotor - MotorAll.dRequiredPosX >= MM_Step and MotorAll.G03IsActive == true)
              or(MotorAll.dRequiredPosX - MotorCommon[0].dAbsolutePosMotor >= MM_Step and MotorAll.G02IsActive == true))
              {
                Step(0); // Step X-axis
                MotorCommon[0].ResetStep = true ;
                bNextStepCircular = true;
                                
                if(MotorAll.G03IsActive == true)
                {
                  MotorCommon[0].dAbsolutePosMotor -= MM_Step;
                }
                else
                {
                  MotorCommon[0].dAbsolutePosMotor += MM_Step;
                }
              }
            }

            // Do step X+
            if(iActiveQuadrant == 3 or iActiveQuadrant == 4)
            {
              if((MotorAll.dRequiredPosX - MotorCommon[0].dAbsolutePosMotor >= MM_Step and MotorAll.G03IsActive == true)
              or(MotorCommon[0].dAbsolutePosMotor - MotorAll.dRequiredPosX >= MM_Step and MotorAll.G02IsActive == true))
              {
                Step(0); // Step X-axis
                MotorCommon[0].ResetStep = true ;
                bNextStepCircular = true;
                
                if(MotorAll.G03IsActive == true)
                {
                  MotorCommon[0].dAbsolutePosMotor += MM_Step;
                }
                else
                {
                  MotorCommon[0].dAbsolutePosMotor -= MM_Step;
                }
              }
            }
  
            // Do step Y+
            if(iActiveQuadrant == 1 or iActiveQuadrant == 4)
            {
              if((MotorAll.dRequiredPosY - MotorCommon[1].dAbsolutePosMotor >= MM_Step and MotorAll.G03IsActive == true)
              or(MotorCommon[1].dAbsolutePosMotor - MotorAll.dRequiredPosY >= MM_Step and MotorAll.G02IsActive == true))
              {
                Step(1); // Step Y-axis
                MotorCommon[1].ResetStep = true ;
                bNextStepCircular = true;

                if(MotorAll.G03IsActive == true)
                {
                  MotorCommon[1].dAbsolutePosMotor += MM_Step;
                }
                else
                {
                  MotorCommon[1].dAbsolutePosMotor -= MM_Step;
                }
              }
            }

            // Do step Y-
            if(iActiveQuadrant == 2 or iActiveQuadrant == 3)
            {
              if((MotorCommon[1].dAbsolutePosMotor - MotorAll.dRequiredPosY >= MM_Step and MotorAll.G03IsActive == true)
              or(MotorAll.dRequiredPosY - MotorCommon[1].dAbsolutePosMotor >= MM_Step and MotorAll.G02IsActive == true))
              {
                Step(1); // Step Y-axis
                MotorCommon[1].ResetStep = true ;
                bNextStepCircular = true;

                if(MotorAll.G03IsActive == true)
                {
                  MotorCommon[1].dAbsolutePosMotor -= MM_Step;
                }
                else
                {
                  MotorCommon[1].dAbsolutePosMotor += MM_Step;
                }
              }
            }

          } // while(bNextStepCircular == false)

          // Switch direction bits
          // Quadrant 1 0 - 90deg, G03: X- Y+
          // Quadrant 2 90 - 180deg, G03: X- Y-
          // Quadrant 3 180 - 270deg, G03: X+ Y-
          // Quadrant 4 270 - 360deg G03: X+ Y+
          if(iActiveQuadrant == 1)
          {
            if(MotorAll.G03IsActive == true)
            {
              MotorCommon[0].StepsPositiveIsActive = false;
              MotorCommon[1].StepsPositiveIsActive = true;
            }
            else
            {
              MotorCommon[0].StepsPositiveIsActive = true;
              MotorCommon[1].StepsPositiveIsActive = false;              
            }
          }
          
          if(iActiveQuadrant == 2)
          {
            if(MotorAll.G03IsActive == true)
            {
              MotorCommon[0].StepsPositiveIsActive = false;
              MotorCommon[1].StepsPositiveIsActive = false;
            }
            else
            {
              MotorCommon[0].StepsPositiveIsActive = true;
              MotorCommon[1].StepsPositiveIsActive = true;                        
            }
          }
          
          if(iActiveQuadrant == 3)
          {
            if(MotorAll.G03IsActive == true)
            {
              MotorCommon[0].StepsPositiveIsActive = true;
              MotorCommon[1].StepsPositiveIsActive = false;
            }
            else
            {
              MotorCommon[0].StepsPositiveIsActive = false;
              MotorCommon[1].StepsPositiveIsActive = true;                        
            }
          }
          
          if(iActiveQuadrant == 4)
          {
            if(MotorAll.G03IsActive == true)
            {
              MotorCommon[0].StepsPositiveIsActive = true;
              MotorCommon[1].StepsPositiveIsActive = true;
            }
            else
            {
              MotorCommon[0].StepsPositiveIsActive = false;
              MotorCommon[1].StepsPositiveIsActive = false;                          
            }
          }
        } // if(MotorAll.DoStep == true)
      } // if(MotorAll.dInternalAnglecounter <= MotorAll.dDistanceAngle)

      // Amount steps reached, movement done
      else
      {

        // ##### End movement
        // ##### Or stop spindle, when movement was already in stopramp and release movement is gone
        switch (MotorAll.StepNoStopSpindle)
        {
          // Release movement is active, end movement
          case 0:
            if(bReleaseMovementCommon == true)
            {
              MotorAll.StepNo = StepHandshakeCommandDone;
      
              // DebugMode, print values
              if(DEBUGMODE == true)
              {
                Serial.print("Steps in Startramp ");
                Serial.println(DebugValues.StepsInStartramp, DEC);
                Serial.print("Steps in Linear ");
                Serial.println(DebugValues.StepsInLinear, DEC);
                Serial.print("Steps in Stopramp ");
                Serial.println(DebugValues.StepsInStopramp, DEC);
              }
            }
            // No release movement, stop spindle
            else
            {
              MotorAll.StepNoStopSpindle = 1;
  
              // Switch spindle off
              if(MotorAll.SpindleOn == true)
              {
                MotorAll.SpindleOn = false;
              }
              else // Spindle was not on, end movement
              {
                MotorAll.StepNoStopSpindle = 10;
              }
            }
            break; 

          // Wait for next release
          case 1:
            if(bReleaseMovementCommon == true)
            {
              MotorAll.StepNoStopSpindle = 2;
              // Start delaytime with 2000ms
              MotorCommon[0].DelaytimeMoveCommand  = false;
              MotorCommon[0].CounterDelaytimeMoveCommand = 0;
              MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
              // Switch spindle on
              MotorAll.SpindleOn = true;
            }
            break; 

          // Delaytime elapsed
          case 2:
            if (MotorCommon[0].DelaytimeMoveCommand  == true)
            {
              MotorAll.StepNoStopSpindle = 10;
            }
            break;

          // End movement
          case 10:
            MotorAll.StepNo = StepHandshakeCommandDone;
    
            // DebugMode, print values
            if(DEBUGMODE == true)
            {
              Serial.print("Steps in Startramp ");
              Serial.println(DebugValues.StepsInStartramp, DEC);
              Serial.print("Steps in Linear ");
              Serial.println(DebugValues.StepsInLinear, DEC);
              Serial.print("Steps in Stopramp ");
              Serial.println(DebugValues.StepsInStopramp, DEC);
            }
            break; 
            
          default:
            break;
        }

      }

      break; // case StepCircularMovement:

    // ##### Inching ##################################################################
    case StepInching:

      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          Serial.println("Start inching");
        }
      }

      // ##### Inching X
      switch (MotorAll.StepNoInchingX)
      {
        // Wait for start
        case 0:
          if(MotorRecive.XPlus == true || MotorRecive.XMinus == true)
          {
            if(MotorRecive.XPlus == true)
            {
              MotorAll.bInchingXPlusActive = true;
            }
            if(MotorRecive.XMinus == true)
            {
              MotorAll.bInchingXMinusActive = true;
            }
            MotorAll.StepNoInchingX = 1;
          }
          break;
  
        // Switch direction bit
        case 1:
          MotorCommon[0].StepsPositiveIsActive = true;
          if(MotorRecive.XMinus == true)
          {
            MotorCommon[0].StepsPositiveIsActive = false;
          }
          MotorCommon[0].DelaytimeMoveCommand = false;
          MotorCommon[0].CounterDelaytimeMoveCommand = 0;
          MotorCommon[0].ValueDelaytimeMoveCommand = WaitMoveCommand;
          MotorAll.StepNoInchingX = 2;
          break;
  
        // Is delaytime move command elapsed
        case 2:
          if (MotorCommon[0].DelaytimeMoveCommand  == true)
          {
            MotorAll.StepNoInchingX = 3;
          }
          break;

        // Move
        case 3:
          if(bReleaseMovementCommon == true)
          {
            if((MotorRecive.XPlus == true && MotorAll.bInchingXPlusActive == true) || (MotorRecive.XMinus == true && MotorAll.bInchingXMinusActive == true))
            {
              MotorAll.lCounterRampInchingX ++;
              if (MotorAll.lCounterRampInchingX >= MotorAll.lSpeedInProgramscansPerStep)
              {
                MotorAll.lCounterRampInchingX = 0;
                MotorAll.DoStepInchingX = true;
              }
            }
            else // Direction change
            {
              MotorAll.bInchingXPlusActive = false;
              MotorAll.bInchingXMinusActive = false;
              MotorAll.StepNoInchingX = 0;
            }
          }
          else // Switch enable step off and wait for next release
          {
            MotorAll.EnableStep = false;
          }
          break;

        default:
          break;
        
      } // case MotorAll.StepNoInchingX

      // ##### Inching Y
      switch (MotorAll.StepNoInchingY)
      {
        // Wait for start
        case 0:
          if(MotorRecive.YPlus == true || MotorRecive.YMinus == true)
          {
            if(MotorRecive.YPlus == true)
            {
              MotorAll.bInchingYPlusActive = true;
            }
            if(MotorRecive.YMinus == true)
            {
              MotorAll.bInchingYMinusActive = true;
            }
            MotorAll.StepNoInchingY = 1;
          }
          break;
  
        // Switch direction bit
        case 1:
          MotorCommon[1].StepsPositiveIsActive = true;
          if(MotorRecive.YMinus == true)
          {
            MotorCommon[1].StepsPositiveIsActive = false;
          }
          MotorCommon[1].DelaytimeMoveCommand = false;
          MotorCommon[1].CounterDelaytimeMoveCommand = 0;
          MotorCommon[1].ValueDelaytimeMoveCommand = WaitMoveCommand;
          MotorAll.StepNoInchingY = 2;
          break;
  
        // Is delaytime move command elapsed
        case 2:
          if (MotorCommon[1].DelaytimeMoveCommand  == true)
          {
            MotorAll.StepNoInchingY = 3;
          }
          break;

        // Move
        case 3:
          if(bReleaseMovementCommon == true)
          {
            if((MotorRecive.YPlus == true && MotorAll.bInchingYPlusActive == true) || (MotorRecive.YMinus == true && MotorAll.bInchingYMinusActive == true))
            {
              MotorAll.lCounterRampInchingY ++;
              if (MotorAll.lCounterRampInchingY >= MotorAll.lSpeedInProgramscansPerStep)
              {
                MotorAll.lCounterRampInchingY = 0;
                MotorAll.DoStepInchingY = true;
              }
            }
            else // Direction change
            {
              MotorAll.bInchingYPlusActive = false;
              MotorAll.bInchingYMinusActive = false;
              MotorAll.StepNoInchingY = 0;
            }
          }
          else // Switch enable step off and wait for next release
          {
            MotorAll.EnableStep = false;
          }
          break;

        default:
          break;
        
      } // case MotorAll.StepNoInchingY

      // ##### Inching Z
      switch (MotorAll.StepNoInchingZ)
      {
        // Wait for start
        case 0:
          if(MotorRecive.ZPlus == true || MotorRecive.ZMinus == true)
          {
            if(MotorRecive.ZPlus == true)
            {
              MotorAll.bInchingZPlusActive = true;
            }
            if(MotorRecive.ZMinus == true)
            {
              MotorAll.bInchingZMinusActive = true;
            }
            MotorAll.StepNoInchingZ = 1;
          }
          break;
  
        // Switch direction bit
        case 1:
          MotorCommon[2].StepsPositiveIsActive = true;
          if(MotorRecive.ZMinus == true)
          {
            MotorCommon[2].StepsPositiveIsActive = false;
          }
          MotorCommon[2].DelaytimeMoveCommand = false;
          MotorCommon[2].CounterDelaytimeMoveCommand = 0;
          MotorCommon[2].ValueDelaytimeMoveCommand = WaitMoveCommand;
          MotorAll.StepNoInchingZ = 2;
          break;
  
        // Is delaytime move command elapsed
        case 2:
          if (MotorCommon[2].DelaytimeMoveCommand  == true)
          {
            MotorAll.StepNoInchingZ = 3;
          }
          break;

        // Move
        case 3:
          if(bReleaseMovementCommon == true)
          {
            if((MotorRecive.ZPlus == true && MotorAll.bInchingZPlusActive == true) || (MotorRecive.ZMinus == true && MotorAll.bInchingZMinusActive == true))
            {
              MotorAll.lCounterRampInchingZ ++;
              if (MotorAll.lCounterRampInchingZ >= MotorAll.lSpeedInProgramscansPerStep)
              {
                MotorAll.lCounterRampInchingZ = 0;
                MotorAll.DoStepInchingZ = true;
              }
            }
            else // Direction change
            {
              MotorAll.bInchingZPlusActive = false;
              MotorAll.bInchingZMinusActive = false;
              MotorAll.StepNoInchingZ = 0;
            }
          }
          else // Switch enable step off and wait for next release
          {
            MotorAll.EnableStep = false;
          }
          break;

        default:
          break;
        
      } // case MotorAll.StepNoInchingZ

      // ##### Reset output step X, Y, Z
      for (i = 0; i < AmountOfSteppermotors; i++)
      {
        if(MotorCommon[i].ResetStep == true)
        {
          Step(i); // Step to reset output axis
          MotorCommon[i].ResetStep = false;
        }
      }

      // ##### Do Step X
      if(MotorAll.DoStepInchingX == true)
      {
        MotorAll.DoStepInchingX = false;
  
        Step(0); // Step X-axis
        MotorCommon[0].ResetStep = true ;
        if( MotorCommon[0].StepsPositiveIsActive == true)
        {
          MotorCommon[0].dAbsolutePosMotor += MM_Step;
        }
        else
        {
          MotorCommon[0].dAbsolutePosMotor -= MM_Step;
        }
      }

      // ##### Do Step Y
      if(MotorAll.DoStepInchingY == true)
      {
        MotorAll.DoStepInchingY = false;
  
        Step(1); // Step Y-axis
        MotorCommon[1].ResetStep = true ;
        if( MotorCommon[1].StepsPositiveIsActive == true)
        {
          MotorCommon[1].dAbsolutePosMotor += MM_Step;
        }
        else
        {
          MotorCommon[1].dAbsolutePosMotor -= MM_Step;
        }
      }

      // ##### Do Step Z
      if(MotorAll.DoStepInchingZ == true)
      {
        MotorAll.DoStepInchingZ = false;
  
        Step(2); // Step Z-axis
        MotorCommon[2].ResetStep = true ;
        if( MotorCommon[2].StepsPositiveIsActive == true)
        {
          MotorCommon[2].dAbsolutePosMotor += MM_Step;
        }
        else
        {
          MotorCommon[2].dAbsolutePosMotor -= MM_Step;
        }
      }

      // ##### Inching done
      // Wait until MotorAll.EnableStep and MotorRecive.Operate == false
      if(bInchingIsActive == false && MotorAll.EnableStep == false && MotorRecive.Operate == false)
      {
        MotorAll.StepNo = StepHandshakeCommandDone;
        // DebugMode, print values
        if(DEBUGMODE == true)
        {
          Serial.println("InchingDone");
        }
      }

      break; // case StepInching:

    // ##### Measure tool length ##################################################################
    case StepMeasureToolLength:

      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          Serial.println("Start measure tool length");
        }
      }

      // ##### Measure tool length interrupted
      // Measure tool length interrupted by operator, e.g. button MeasureToolLength Off pressed
      if(MotorRecive.MeasureToolLength == false || bReleaseMovementCommon == false)
      {
        MotorAll.EnableStep = false;
        MotorAll.StepNo = StepHandshakeCommandDone;
        // DebugMode, print values
        if(DEBUGMODE == true)
        {
          Serial.println("MeasuringToolLengthInterrupted by operator");
        }
        break;
      }

      // ##### Reset output step Z
      if(MotorCommon[2].ResetStep == true)
      {
        Step(2); // Step to reset output axis
        MotorCommon[2].ResetStep = false;
      }

      // ##### Main case measure tool length sensor
      switch (MotorAll.StepNoMeasureToolLength)
      {

        // ##### Run sensor free
        case StepMeasureToolLengthRunSensorFreeFast:
            // Set direction bit positive (up)
            MotorCommon[2].StepsPositiveIsActive = true;
      
            // Is delaytime move command elapsed?
            if (MotorCommon[2].DelaytimeMoveCommand  == false)
            {
              break;
            }

            MotorAll.lCounterRamp ++;
            if (MotorAll.lCounterRamp >= MotorAll.lSpeedInProgramscansPerStep)
            {
              MotorAll.lCounterRamp = 0;
      
              // Motor is at sensor, one step positive
              if (MotorAll.ToolLengthSensor == true)
              {
                // Motor one step
                Step(2);
                MotorCommon[2].ResetStep = true ;
              }
              // Motor no more at sensor
              else
              {
                MotorAll.StepNoMeasureToolLength = StepMeasureToolLengthFindSensor;
    
                // Delaytime release next steps
                MotorCommon[2].DelaytimeMoveCommand = false;
                MotorCommon[2].CounterDelaytimeMoveCommand = 0;
                MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;
              }
            }
            break;

        // ##### Find sensor
        case StepMeasureToolLengthFindSensor:
            // Set direction bit negative (down)
            MotorCommon[2].StepsPositiveIsActive = false;
      
            // Is delaytime move command elapsed?
            if (MotorCommon[2].DelaytimeMoveCommand  == false)
            {
              break;
            }

            MotorAll.lCounterRamp ++;
            if (MotorAll.lCounterRamp >= MotorAll.lSpeedInProgramscansPerStep)
            {
              MotorAll.lCounterRamp = 0;
      
              // Motor is not at sensor, one step negative
              if (MotorAll.ToolLengthSensor == false)
              {
                // Motor one step
                Step(2);
                MotorCommon[2].ResetStep = true ;
              }
              // Motor at sensor
              else
              {
                MotorAll.StepNoMeasureToolLength = StepMeasureToolLengthRunSensorFree;
    
                // Delaytime release next steps
                MotorCommon[2].DelaytimeMoveCommand = false;
                MotorCommon[2].CounterDelaytimeMoveCommand = 0;
                MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;
              }
            }
            break;

        case StepMeasureToolLengthRunSensorFree:
            // Set direction bit positive (up)
            MotorCommon[2].StepsPositiveIsActive = true;
      
            // Is delaytime move command elapsed?
            if (MotorCommon[2].DelaytimeMoveCommand  == false)
            {
              break;
            }

            // Delaytime between each step
            MotorCommon[2].DelaytimeMoveCommand = false;
            MotorCommon[2].CounterDelaytimeMoveCommand = 0;
            MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;
      
            // Motor is at sensor, one step positive
            if (MotorAll.ToolLengthSensor == true)
            {
              // Motor one step
              Step(2);
              MotorCommon[2].ResetStep = true ;
            }
            // Motor no more at sensor
            else
            {
              MotorAll.StepNoMeasureToolLength = StepMeasureToolLengthGotoSensor;
  
              // Delaytime release next steps
              MotorCommon[2].DelaytimeMoveCommand = false;
              MotorCommon[2].CounterDelaytimeMoveCommand = 0;
              MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;
            }
            break;

        // ##### Goto sensor
        case StepMeasureToolLengthGotoSensor:
            // Set direction bit negative (down)
            MotorCommon[2].StepsPositiveIsActive = false;
      
            // Is delaytime move command elapsed?
            if (MotorCommon[2].DelaytimeMoveCommand  == false)
            {
              break;
            }

            // Delaytime between each step
            MotorCommon[2].DelaytimeMoveCommand = false;
            MotorCommon[2].CounterDelaytimeMoveCommand = 0;
            MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;
      
            // Motor is not at sensor, one step negative
            if (MotorAll.ToolLengthSensor == false)
            {
              // Motor one step
              Step(2);
              MotorCommon[2].ResetStep = true ;
            }
            // Motor at sensor
            else
            {
              MotorAll.StepNoMeasureToolLength = StepMeasureToolLengthUpFast;
              MotorAll.lCounterRamp = 0;
  
              // Delaytime release next steps
              MotorCommon[2].DelaytimeMoveCommand = false;
              MotorCommon[2].CounterDelaytimeMoveCommand = 0;
              MotorCommon[2].ValueDelaytimeMoveCommand = WaitPositioning;

              // Set Z-Axis position
              MotorCommon[2].dAbsolutePosMotor = ToolLengthSensor;
              MotorCommon[2].dAbsolutePosFromMachineZeroPoint = 0.0;
              MotorCommon[2].dRequiredPosAbsolute = 0.0; // New required position (Startposition for next move command)

              // Transfer speed for next step up to reference sensor
              MotorAll.lSpeedInProgramscansPerStep = SpeedMM_Min_ProgramScans(SpeedMeasureToolLengthUpToReferenceSensor); // Call function to calculate speed in ProgramScans from mm/min. In = double, Out = unsigned long
              MotorAll.dActualSpeed = SpeedMeasureToolLengthUpToReferenceSensor;
            }
            break;

        // ##### Go up to reference sensor Z
        case StepMeasureToolLengthUpFast:
            // Set direction bit positive (up)
            MotorCommon[2].StepsPositiveIsActive = true;
      
            // Is delaytime move command elapsed?
            if (MotorCommon[2].DelaytimeMoveCommand  == false)
            {
              break;
            }

            MotorAll.lCounterRamp ++;
            if (MotorAll.lCounterRamp >= MotorAll.lSpeedInProgramscansPerStep)
            {
              MotorAll.lCounterRamp = 0;

              // Motor is not at reference sensor, one step positive
              if (MotorCommon[2].ReferenceSensor == false)
              {
                // Motor one step
                Step(2);
                MotorCommon[2].ResetStep = true ;
                MotorCommon[2].dAbsolutePosMotor += MM_Step;
              }
              // Motor at reference sensor
              else
              {
                // ##### Measure tool length done
                MotorAll.MeasureToolLengthDone = true;
                MotorAll.EnableStep = false;
                MotorAll.StepNo = StepHandshakeCommandDone;
                // DebugMode, print values
                if(DEBUGMODE == true)
                {
                  Serial.println("MeasuringToolLengthDone");
                }
              }
            }
            break;

        default:
        break;
      } // switch (MotorAll.StepNoMeasureToolLength)

    break; // case StepMeasureToolLength:

    // ##### Dwell time ##################################################################
    case StepDwellTime:
      // ##### First cycle movement
      if(FirstCycleMovement == false)
      {
        FirstCycleMovement = true;

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          // Print values
          Serial.print("Dwelltime in cycles "); 
          Serial.println(MotorAll.lDwellTimeCycles, DEC);      
        }
      }

      switch (MotorAll.StepNoStopSpindle)
      {
        // Release movement is active, count dwelltime
        case 0:
          if(bReleaseMovementCommon == true)
          {
            // Counter dwelltime
            MotorAll.lDwellTimeCounter += 1;
      
            // Dwelltime elapsed
            if(MotorAll.lDwellTimeCounter >= MotorAll.lDwellTimeCycles)
            {
              MotorAll.StepNo = StepHandshakeCommandDone;
            }
          }

          // No release movement, reset counter dwelltime, stop spindle
          else
          {
            MotorAll.lDwellTimeCounter = 0; // Reset counter dwelltime
            
            // Switch spindle off
            if(MotorAll.SpindleOn == true)
            {
              MotorAll.StepNoStopSpindle = 1;
              MotorAll.SpindleOn = false;
            }
          }
          break; 

        // Wait for next release
        case 1:
          if(bReleaseMovementCommon == true)
          {
            MotorAll.StepNoStopSpindle = 2;
            // Start delaytime with 2000ms
            MotorCommon[0].DelaytimeMoveCommand  = false;
            MotorCommon[0].CounterDelaytimeMoveCommand = 0;
            MotorCommon[0].ValueDelaytimeMoveCommand = WaitForSpindleOnAfterImmediateStop;
            // Switch spindle on
            MotorAll.SpindleOn = true;
          }
          break; 

          // Delaytime 2000ms elapsed, to switch spindle back on again
          case 2:
            if (MotorCommon[0].DelaytimeMoveCommand  == true)
            {
              MotorAll.StepNoStopSpindle = 0; // Goto step count dwelltime
            }
            break;

        default:
          break;
      }
 
      break; // case StepDwellTime:

    // ##### Handshake command done ##################################################################
    case StepHandshakeCommandDone:

        // ##### DebugMode, print values
        if(DEBUGMODE == true)
        {
          // Print values
          Serial.println("##### Step handshake command done"); 
        }

        // Set values command done
        MotorAll.CommandActive = false;
        MotorAll.bInchingStarted = false;
        MotorAll.MeasureToolLengthStarted = false;
        MotorAll.StepNo = StepNoAction;

      break; // case StepHandshakeCommandDone:

    default:
      break;

  } // switch (MotorAll.StepNo)

  // ######################################################################################################
  // ##### Digital outputs, status

  // ##### Transfer status reference sensor X, Y, Z
  if (MotorCommon[0].ReferenceSensor == true)
  {
    MotorSend.ReferenceSensorX = true;
  }
  else
  {
    MotorSend.ReferenceSensorX = false;
  }
  
  if (MotorCommon[1].ReferenceSensor == true)
  {
    MotorSend.ReferenceSensorY = true;
  }
  else
  {
    MotorSend.ReferenceSensorY = false;
  }
  
  if (MotorCommon[2].ReferenceSensor == true)
  {
    MotorSend.ReferenceSensorZ = true;
  }
  else
  {
    MotorSend.ReferenceSensorZ = false;
  }

  // ##### Transfer status emergency stop
  if(MotorAll.EmergencyStop == true)
  {
    MotorSend.EmergencyStopButton = true;
  }
  else
  {
    MotorSend.EmergencyStopButton = false;
  }

  // ##### Transfer status tool length sensor
  if(MotorAll.ToolLengthSensor == true)
  {
    MotorSend.ToolLengthSensor = true;
  }
  else
  {
    MotorSend.ToolLengthSensor = false;
  }

  // ##### Transfer status measure tool length done
  if(MotorAll.MeasureToolLengthDone == true)
  {
    MotorSend.MeasureToolLengthDone = true;
  }
  else
  {
    MotorSend.MeasureToolLengthDone = false;
  }

  // ##### Transfer status and digital output spindle on
  if(MotorAll.SpindleOn == true or MotorRecive.SpindleOn == true)
  {
    digitalWriteFast(DigitalOutputSpindleOn, HIGH);
    MotorSend.SpindleOn = true;
  }
  else
  {
    digitalWriteFast(DigitalOutputSpindleOn, LOW);
    MotorSend.SpindleOn = false;
  }

  // ##### Transfer status and digital output coolant on
  if(MotorAll.CoolantOn == true or MotorRecive.CoolantOn == true)
  {
    digitalWriteFast(DigitalOutputCoolantOn, HIGH);
    MotorSend.CoolantOn = true;
  }
  else
  {
    digitalWriteFast(DigitalOutputCoolantOn, LOW);
    MotorSend.CoolantOn = false;
  }

  // ##### Transfer status and digital output clamp on
  if(MotorAll.ClampOn == true or MotorRecive.ClampOn == true)
  {
    digitalWriteFast(DigitalOutputClampOn, HIGH);
    MotorSend.ClampOn = true;
  }
  else
  {
    digitalWriteFast(DigitalOutputClampOn, LOW);
    MotorSend.ClampOn = false;
  }

  // ##### Transfer status and digital output light on
  if(MotorAll.LightOn == true or MotorRecive.LightOn == true)
  {
    digitalWriteFast(DigitalOutputLightOn, HIGH);
    MotorSend.LightOn = true;
  }
  else
  {
    digitalWriteFast(DigitalOutputLightOn, LOW);
    MotorSend.LightOn = false;
  }

  // ##### Switch enable
  /*
  Releas: MotorRecive.Operate == true && MotorAll.EmergencyStop == false
  Releas              _____|-------------|__________
  MotorAll.Enable     _____|------------------|_____
  MotorAll.EnableStep __________|-------------|_____
  DigitalOut Enable   _____|------------------|_____

  MotorAll.EnableStep is also switched off when stopramp in linear or circular movement is done.
  MotorAll.EnableStep is also switched off during inching, immediatelly after release movement is gone
  MotorAll.EnableStep is also switched off during Measure tool length, immediatelly after release movement is gone
   */
  if (MotorRecive.Operate == true && MotorAll.EmergencyStop == false) 
  {
    MotorsReleased = true;
  }
  else
  {
    MotorsReleased = false;
  }
  
  switch (MotorAll.StepNoEnable)
  {
    case 0: // "Enable" on when motors are released
      if (MotorsReleased == true)
      {
        MotorAll.Enable = true;
        MotorAll.StepNoEnable = 1;
        MotorAll.CounterEnableStepAfterEnableOn = 0;
      }
      break;

    case 1: // "EnableStep" on after motors are released
      if (MotorAll.CounterEnableStepAfterEnableOn < WaitEnableStepAfterEnableOn)
      {
        MotorAll.CounterEnableStepAfterEnableOn ++;
      }
      else
      {
        MotorAll.EnableStep = true;
        MotorAll.StepNoEnable = 2;
      }
      if (MotorsReleased == false)
      {
        MotorAll.EnableStep = false;
        MotorAll.StepNoEnable = 0;
      }
      break;
      
    case 2: // Motors are no more released
      if (MotorsReleased == false)
      {
        MotorAll.StepNoEnable = 3;
        MotorAll.CounterEnableOffAfterOperateOff = 0;
      }
      break;

    case 3: // "EnableStep" and "Enable" off after motors are no more released
      if (MotorAll.CounterEnableOffAfterOperateOff < WaitEnableOffAfterOperateOff)
      {
        MotorAll.CounterEnableOffAfterOperateOff ++;
      }
      else
      {
        MotorAll.Enable = false;
        MotorAll.EnableStep = false;
        MotorAll.StepNoEnable = 4;
        MotorAll.CounterEnableOnAfterOperateOff = 0;
      }
      break;

    case 4: // Release next enable
      if (MotorAll.CounterEnableOnAfterOperateOff < WaitEnableOnAfterOperateOff)
      {
        MotorAll.CounterEnableOnAfterOperateOff ++;
      }
      else
      {
        MotorAll.StepNoEnable = 0;
      }
      break;

    default:
      break;     
  }

  // ##### Transfer digital output enable
  // Enable (0=Enabled, 1=Blocked)
  if (MotorAll.Enable == true)
  {
    digitalWriteFast(DigitalOutputMotor0Enable, LOW); // Switch output off
    digitalWriteFast(DigitalOutputMotor1Enable, LOW); // Switch output off
    digitalWriteFast(DigitalOutputMotor2Enable, LOW); // Switch output off
    //digitalWriteFast(DigitalOutputMotor3Enable, LOW); // Switch output off
    MotorSend.OperateOn = true;
  }
  else
  {
    digitalWriteFast(DigitalOutputMotor0Enable, HIGH); // Switch output on
    digitalWriteFast(DigitalOutputMotor1Enable, HIGH); // Switch output on
    digitalWriteFast(DigitalOutputMotor2Enable, HIGH); // Switch output on
    //digitalWriteFast(DigitalOutputMotor3Enable, HIGH); // Switch output on
    MotorSend.OperateOn = false;
    MotorCommon[0].RotationPositiveWasActive = false;
    MotorCommon[0].RotationNegativeWasActive = false;
    MotorCommon[1].RotationPositiveWasActive = false;
    MotorCommon[1].RotationNegativeWasActive = false;
    MotorCommon[2].RotationPositiveWasActive = false;
    MotorCommon[2].RotationNegativeWasActive = false;
    MotorCommon[3].RotationPositiveWasActive = false;
    MotorCommon[3].RotationNegativeWasActive = false;   
  }

    // ##### Delaytime release next steps
    for (i = 0; i < 4; i++)
    {
      if(MotorCommon[i].DelaytimeMoveCommand == false)
      {
        if(MotorCommon[i].CounterDelaytimeMoveCommand < MotorCommon[i].ValueDelaytimeMoveCommand)
        {
          MotorCommon[i].CounterDelaytimeMoveCommand ++;
        }
        else
        {
          MotorCommon[i].DelaytimeMoveCommand = true;
        }
      }
    }

  // ##### Transfer digital outputs motor 0
  // Step
  if ((MotorCommon[0].DigitalOutputMotor & 0b00000001) != 0 )
  {
    digitalWriteFast(DigitalOutputMotor0Step, HIGH); // Switch output on
  }
  else
  {
    digitalWriteFast(DigitalOutputMotor0Step, LOW);; // Switch output off
  }

  // DIR
  DirPosIsActive = false;
  if (((MotorCommon[0].StepsPositiveIsActive == true || MotorCommon[0].RotationPositiveIsActive == true) && MotorAll.PositioningIsActive == false)
  || (MotorCommon[0].PositioningIsActive == true && MotorCommon[0].DirectionPositivePositioning == true))
  {
    DirPosIsActive = true;
  }
  
  if(Dir[0] == true)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor0Dir, HIGH); // Switch output on
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor0Dir, LOW);; // Switch output off
    }
  }

  if(Dir[0] == false)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor0Dir, LOW);; // Switch output off
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor0Dir, HIGH); // Switch output on
    }
  }

  // ##### Transfer digital outputs motor 1
  // Step
  if ((MotorCommon[1].DigitalOutputMotor & 0b00000001) != 0 )
  {
    digitalWriteFast(DigitalOutputMotor1Step, HIGH); // Switch output on
  }
  else
  {
    digitalWriteFast(DigitalOutputMotor1Step, LOW);; // Switch output off
  }

  // DIR
  DirPosIsActive = false;
  if (((MotorCommon[1].StepsPositiveIsActive == true || MotorCommon[1].RotationPositiveIsActive == true) && MotorAll.PositioningIsActive == false)
  || (MotorCommon[1].PositioningIsActive == true && MotorCommon[1].DirectionPositivePositioning == true))
  {
    DirPosIsActive = true;
  }
  
  if(Dir[1] == true)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor1Dir, HIGH); // Switch output on
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor1Dir, LOW);; // Switch output off
    }
  }

  if(Dir[1] == false)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor1Dir, LOW);; // Switch output off
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor1Dir, HIGH); // Switch output on
    }
  }

  // ##### Transfer digital outputs motor 2
  // Step
  if ((MotorCommon[2].DigitalOutputMotor & 0b00000001) != 0 )
  {
    digitalWriteFast(DigitalOutputMotor2Step, HIGH); // Switch output on
  }
  else
  {
    digitalWriteFast(DigitalOutputMotor2Step, LOW);; // Switch output off
  }

  // DIR
  DirPosIsActive = false;
  if (((MotorCommon[2].StepsPositiveIsActive == true || MotorCommon[2].RotationPositiveIsActive == true) && MotorAll.PositioningIsActive == false)
  || (MotorCommon[2].PositioningIsActive == true && MotorCommon[2].DirectionPositivePositioning == true))
  {
    DirPosIsActive = true;
  }
  
  if(Dir[2] == true)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor2Dir, HIGH); // Switch output on
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor2Dir, LOW);; // Switch output off
    }
  }

  if(Dir[2] == false)
  {
    if(DirPosIsActive == true)
    {
      digitalWriteFast(DigitalOutputMotor2Dir, LOW);; // Switch output off
    }
    else
    {
      digitalWriteFast(DigitalOutputMotor2Dir, HIGH); // Switch output on
    }
  }

  // ##### Transfer digital output PWM spindle speed e.g. 5000-25000 1/min -> 0.1Khz-20Khz -> 0-10V
  /*
  Output PWM is only one cycle high, e.g. 15.5625us, rest of the period length = low
  SpeedSpindleMin e.g. 5000.0 1/min
  SpeedSpindleMax e.g. 25000.0 1/min
  SpindleSpeedMinHz e.g. 100.0 =100.0Hz = 0.1Khz;
  SpindleSpeedMaxHz e.g. 20000.0 =20000.0Hz = 20Khz;

  InputSpindleSpeed = 5000.0 - 25000.0 1/min -> PWM 0.1Khz-20Khz -> 0-10V
  InputSpindleSpeed =  5000.0 1/min-> Period length  0.1Khz = 10.0ms   or 10000us -> Pulse length = 15.5625us, Pause length = 9844.375us  ->  0V
  InputSpindleSpeed = 25000.0 1/min-> Period length 20Khz   =  0.050ms or    50us -> Pulse length = 15.5625us, Pause length =   34.4375us -> 10V
  */

  double dRequiredSpindleSpeed = 0.0;
  double dRequiredHz = 0.0;
  double dRequiredPeriodLength = 0.0;

  // Transfer required speed
  if(MotorRecive.SpindleOn == true) // Manual mode
  {
    dRequiredSpindleSpeed = double(MotorRecive.SpindleSpeedManualMode);
  }
  else // Automatic mode
  {
    dRequiredSpindleSpeed = MotorAll.dRequiredSpindleSpeed;
  }

  // Calculate required Hz, y=m*x+b
  dRequiredHz = (dRequiredSpindleSpeed-SpeedSpindleMin)*((SpindleSpeedMaxHz-SpindleSpeedMinHz)/(SpeedSpindleMax-SpeedSpindleMin))+SpindleSpeedMinHz;

  // Calculate Period length in us
  dRequiredPeriodLength = 1.0/dActualHzPWMSpindleSpeed*1000000.0;

  // Reset output DigitalOutputPWMSpindleSpeed
  ToggleOutputPWMSpindleSpeed = false;

  // Set output DigitalOutputPWMSpindleSpeed
  dCounterOutputPWMSpindleSpeed += TimeInterrupt;
  if(dCounterOutputPWMSpindleSpeed >= dRequiredPeriodLength)
  {
    dCounterOutputPWMSpindleSpeed = 0;
    ToggleOutputPWMSpindleSpeed = true;

    // Ramp actual Hz up / down
    if(dRequiredHz > dActualHzPWMSpindleSpeed)
    {
      dActualHzPWMSpindleSpeed += 5.0;
      if(dActualHzPWMSpindleSpeed > SpindleSpeedMaxHz)
      {
        dActualHzPWMSpindleSpeed = SpindleSpeedMaxHz;
      }
    }
     
    if(dRequiredHz < dActualHzPWMSpindleSpeed)
    {
      dActualHzPWMSpindleSpeed -= 5.0;
      if(dActualHzPWMSpindleSpeed < SpindleSpeedMinHz)
      {
        dActualHzPWMSpindleSpeed = SpindleSpeedMinHz;
      }
    }
  }

  // Transfer output DigitalOutputPWMSpindleSpeed
  digitalWriteFast(DigitalOutputPWMSpindleSpeed, ToggleOutputPWMSpindleSpeed);

  // #############################################################################################################
  // ##### Copy I2C send data. Volatile data must be copied value by value, no struct copy possible
  MotorSendCopy.OperateOn = MotorSend.OperateOn;
  MotorSendCopy.Reserve0_1 = MotorSend.Reserve0_1;
  MotorSendCopy.Reserve0_2 = MotorSend.Reserve0_2;
  MotorSendCopy.Reserve0_3 = MotorSend.Reserve0_3;
  MotorSendCopy.Reserve0_4 = MotorSend.Reserve0_4;
  MotorSendCopy.Reserve0_5 = MotorSend.Reserve0_5;
  MotorSendCopy.Reserve0_6 = MotorSend.Reserve0_6;
  MotorSendCopy.Reserve0_7 = MotorSend.Reserve0_7;
  
  MotorSendCopy.CommandDone = MotorSend.CommandDone;
  MotorSendCopy.ReadyForCommand = MotorSend.ReadyForCommand;
  MotorSendCopy.ProgramDone = MotorSend.ProgramDone;
  MotorSendCopy.Reserve1_3 = MotorSend.Reserve1_3;
  MotorSendCopy.Reserve1_4 = MotorSend.Reserve1_4;
  MotorSendCopy.Reserve1_5 = MotorSend.Reserve1_5;
  MotorSendCopy.Reserve1_6 = MotorSend.Reserve1_6;
  MotorSendCopy.Reserve1_7 = MotorSend.Reserve1_7;

  MotorSendCopy.ReferenceSensorX = MotorSend.ReferenceSensorX;
  MotorSendCopy.ReferenceSensorY = MotorSend.ReferenceSensorY;
  MotorSendCopy.ReferenceSensorZ = MotorSend.ReferenceSensorZ;
  MotorSendCopy.EmergencyStopButton = MotorSend.EmergencyStopButton;
  MotorSendCopy.SpindleOn = MotorSend.SpindleOn;
  MotorSendCopy.CoolantOn = MotorSend.CoolantOn;
  MotorSendCopy.ClampOn = MotorSend.ClampOn;
  MotorSendCopy.LightOn = MotorSend.LightOn;

  MotorSendCopy.ToolLengthSensor = MotorSend.ToolLengthSensor;
  MotorSendCopy.MeasureToolLengthDone = MotorSend.MeasureToolLengthDone;
  MotorSendCopy.Reserve3_2 = MotorSend.Reserve3_2;
  MotorSendCopy.Reserve3_3 = MotorSend.Reserve3_3;
  MotorSendCopy.Reserve3_4 = MotorSend.Reserve3_4;
  MotorSendCopy.Reserve3_5 = MotorSend.Reserve3_5;
  MotorSendCopy.Reserve3_6 = MotorSend.Reserve3_6;
  MotorSendCopy.Reserve3_7 = MotorSend.Reserve3_7;
  
  MotorSendCopy.ActPosMotor[0] = MotorCommon[0].dAbsolutePosMotor;
  MotorSendCopy.ActPosMotor[1] = MotorCommon[1].dAbsolutePosMotor;
  MotorSendCopy.ActPosMotor[2] = MotorCommon[2].dAbsolutePosMotor;
  MotorSendCopy.ActPosMotor[3] = MotorCommon[3].dAbsolutePosMotor;
  MotorSendCopy.ActualStepProgram = MotorSend.ActualStepProgram;
  MotorSendCopy.Checksum = MotorSend.Checksum;
} // End interrupt handler
