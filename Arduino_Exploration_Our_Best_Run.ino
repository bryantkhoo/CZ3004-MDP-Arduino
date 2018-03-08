#include "DualVNH5019MotorShield.h"
#include "PinChangeInt.h"
#include "SharpIR.h"

DualVNH5019MotorShield md;

#define MREncoder1 5
#define MLEncoder2 3

#define SRmodel 1080
// Double Check model number for LR IR
#define LRmodel 20150
#define s1 A1 //front_left
#define s2 A2 //front_right
#define s3 A3 //side_left
#define s4 A4 //side_right
#define s5 A5 //front_middle

#define s6 A0 //long_range

SharpIR sr1 =  SharpIR(s1, SRmodel);
SharpIR sr2 =  SharpIR(s2, SRmodel);
SharpIR sr3 =  SharpIR(s3, SRmodel);
SharpIR sr4 =  SharpIR(s4, SRmodel);
SharpIR sr5 =  SharpIR(s5, SRmodel);
SharpIR sr6 =  SharpIR(s6, LRmodel);

float T1, T2;
float RPMR, RPML, totalDis = 0;
int SPEEDR = 100, SPEEDL = 100, output = 0;
double dif, test, prev, integral;

int count_ML = 0;
unsigned long now_ML;
unsigned long EndTime_ML;
double duration_ML;

int count_MR = 0;
unsigned long now_MR;
unsigned long EndTime_MR;
double duration_MR;

/*Function declartion*/
double calculateRPM(double pulse);
void countPulse_MR(void);
void countPulse_ML(void);
void stopIfFault(void);
double getDistance(int op);
void left(float n);
void right(float n);
float piControlForward(float left, float right);
void goStraightOneTime();

/*working variables*/
double pre_Input_ML ;
double Input_ML = 0;
double Output_ML = 0;
double error_ML = 0;
double lastErr_ML = 0;
double secLastErr_ML = 0;

double pre_Input_MR;
double Input_MR = 0;
double Output_MR = 0;
double error_MR = 0;
double lastErr_MR = 0;
double secLastErr_MR = 0;
double Setpoint;
double SetpointFar;

double nowTest = micros();
double deltanowTest2 = 0;
double now2 = 0;
double nowTest2 = 0;
double rpm = 30.0 / 130.0;
int count;

boolean firstFlag = true;
boolean printFlag = false;
boolean caliFlag = true;
int counterSensor = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(MLEncoder2, INPUT);
  pinMode(MREncoder1, INPUT);
  md.init();
  Setpoint = 62;//62;
  SetpointFar = 83;
  PCintPort::attachInterrupt(MLEncoder2, countPulse_ML, RISING);
  PCintPort::attachInterrupt(MREncoder1, countPulse_MR, RISING);
  now2 = micros();
}

void loop() {
  char cc;
  if (printFlag == true)
  {
    printSensors();
  }
  if (Serial.available() > 0)
  {
    cc = char(Serial.read());
    readRobotCommands(cc);
  }

  goStraightManyGrids(100000);
  restartPID();
}

void readRobotCommands(char command) {
  switch (command) {
    // Movements for F to 10 grids
    // Maximum possibl grids to move = 17
    case 'F':
      goStraightOneGrid(11350);//11300//9600
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '2':
      goStraightManyGrids(23400);//22500);//23400);//24600);//21000;
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '3':
      goStraightManyGrids(36300);//35200);//36300);//31800;
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '4':
      goStraightManyGrids(49500);//47000);//49500);//43000
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '5':
      goStraightManyGrids(61300);//60000);//61300);//54300);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '6':
      goStraightManyGrids(74300);//72800);//74300);//65500;
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '7':
      goStraightManyGrids(87500);//85500);//87500);//76700);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '8':
      goStraightManyGrids(100800);//98000);//100800);//87200;
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '9':
      goStraightManyGrids(113400);//110500);//113400);//98600);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case '0':
      goStraightManyGrids(125800);//124000);//125800);//109000);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case 'R':
      right(0.25);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case 'L':
      left(0.25);
      restartPID();
      caliFlag = true;
      printFlag = true;
      break;

    case 'T':
      printFlag = true;
      break;

    // If we are receiving 'C' from algo to calibrate
    case 'C':
      CaliAngle();
      CaliAngle();
      delay(150);
      Serial.println("C DONE");
      break;

    default:
      break;
  }
}

/*Constants KP,KI,KD for each motor ML*/

float KP_ML = 4 ;//6.5//4.2
float KI_ML = 0.27;
float KD_ML = 5;//.29;

float KP_MR = 4;//4
float KI_MR = 0.277;
float KD_MR = 5;//.29149;


void goStraightOneTime()
{


  SPEEDL = 200; //250
  SPEEDR = 200;
  nowTest2 = micros();
  PIDCompute(KP_ML, KI_ML, KP_ML, KP_MR, KI_MR, KP_MR, Setpoint);
  md.setSpeeds(Output_ML * SPEEDL, Output_MR * SPEEDR);
  deltanowTest2 = micros() - nowTest;
  delayMicroseconds(4230);
}


int goStraightOneGrid(long value)
{
  while (1) {
    if (totalDis >= value) //125000)
    {
      totalDis = 0;
      md.setBrakes(355, 350);
      break;
    }
    else {
      goStraightOneTime();
      totalDis = totalDis + Input_ML + Input_MR;
    }
  }
}

void goStraightFarAway()
{
  SPEEDL = 300; //250
  SPEEDR = 300;
  nowTest2 = micros();
  PIDCompute(12, 0.14, 9, 12, 0.147, 9, SetpointFar);
  md.setSpeeds(Output_ML * SPEEDL, Output_MR * SPEEDR);
  deltanowTest2 = micros() - nowTest;
  delayMicroseconds(4230);
}

int goStraightManyGrids(long value)
{
  while (1) {
    if (totalDis >= value) //125000)
    {
      totalDis = 0;
      md.setBrakes(375, 375);
      break;
    }
    else {
      goStraightFarAway();
      totalDis = totalDis + Input_ML + Input_MR;
    }
  }
}

void right(float n)
{
  boolean flag = true;
  int rotate = 4 * n;
  SPEEDL = 300;
  SPEEDR = -300;
  while (flag) {
    if (totalDis >= 15650)//17170)//17200 //9600 //8100 //9400(13/03/17) //14650 (16/03/17) //10085 720=108085 1080=164485 10085
    {
      md.setBrakes(375, -375);
      totalDis = 0;
      count++;
      if (count == rotate) {
        flag = false;
        count = 0;
      }
    }

    else {
      PIDCompute(12, 0.14, 9, 12, 0.147, 9, 101);
      md.setSpeeds(Output_ML * SPEEDL, Output_MR * SPEEDR);
      totalDis = totalDis + Input_ML + Input_MR;
      delayMicroseconds(4230);
    }
  }
}

void left(float n)
{
  boolean  flag = true;
  SPEEDL = -300;
  SPEEDR = 300;

  while (flag) {
    if (totalDis >= 15650)
    {
      md.setBrakes(-375, 375);
      totalDis = 0;
      count++;

      if (count == 4 * n) {
        count = 0;
        flag = false;
      }
    }

    else {
      PIDCompute(12, 0.14, 9, 12, 0.147, 9, 101);
      md.setSpeeds(Output_ML * SPEEDL, Output_MR * SPEEDR);
      totalDis = totalDis + Input_ML + Input_MR;
      delayMicroseconds(4230);
    }
  }
}


/*Start of countPulse_ML*/
void countPulse_ML() {
  count_ML++;
  if (count_ML == 1) {
    now_ML = micros();
  } else if (count_ML == 5) {
    EndTime_ML = micros();
    duration_ML = (EndTime_ML - now_ML) / 4.0;
    Input_ML = calculateRPM(duration_ML);

    count_ML = 0;
  }
}
/*End of countPulse_ML*/

/*Start of countPulse_ML*/
void countPulse_MR() {
  count_MR++;
  if (count_MR == 1) {
    now_MR = micros();
  } else if (count_MR == 5) {
    EndTime_MR = micros();
    duration_MR = (EndTime_MR - now_MR) / 4.0;

    Input_MR = calculateRPM(duration_MR);

    count_MR = 0;
  }
}
/*End of countPulse_ML*/


// Convert pulse to RPM
double calculateRPM(double pulse) {
  if (pulse == 0) return 0;
  else return 60.00 / (pulse *  562.25 / 1000000.00);
}
// End of Convert pulse to RPM

double rpmToSpeed(double rpm) {
  double rpmR = map(rpm, 27.73403, 103.6323, 100, 300);
  return rpmR;
}

// Start of PID calculation
void PIDCompute(float KP_ML, float KI_ML, float KD_ML, float KP_MR, float KI_MR, float KD_MR, double Setpoint) {



  // Compute error
  error_ML = (Setpoint - Input_ML) / 130.0;
  error_MR = (Setpoint - Input_MR) / 130.0;
  //    Serial.print(Input_ML);
  //    Serial.print("\t");
  //    Serial.println(Input_MR);
  //compute k1, k2, k3 based on kp, ki,kd
  double k1_ML = KP_ML + KI_ML + KD_ML;
  double k2_ML = -KP_ML - 2 * KD_ML;
  double k3_ML = KD_ML;

  double k1_MR = KP_MR + KI_MR + KD_MR;
  double k2_MR = -KP_MR - 2 * KD_MR;
  double k3_MR = KD_MR;

  //Compute PID Output

  Output_ML = pre_Input_ML + k1_ML * error_ML + k2_ML * lastErr_ML + k3_ML * secLastErr_ML;
  Output_MR = pre_Input_MR + k1_MR * error_MR + k2_MR * lastErr_MR + k3_MR * secLastErr_MR;

  // Remember some variables for next time
  secLastErr_ML = lastErr_ML;
  lastErr_ML = error_ML;

  secLastErr_MR = lastErr_MR;
  lastErr_MR = error_MR;

  pre_Input_ML = Output_ML;
  pre_Input_MR = Output_MR;

}

void restartPID() {
  secLastErr_ML = lastErr_ML = error_ML = 0;

  secLastErr_MR = lastErr_MR = error_MR = 0;

  pre_Input_ML = Output_ML = 0;
  pre_Input_MR = Output_MR = 0;
  Input_ML = Input_MR = 0;
}

// This function gets distance of either FrontLeft/FrontRight for CALIBRATION purposes
double getDistance(int sensor)
{
  double sum = 0;
  double ir_val[5];
  // get an average of 3 readings the desired sensor
  if (sensor == 1) {
    return sr1.distance();
  }
  if (sensor == 2) {
    return sr2.distance();
  }
  if (sensor == 3) {
    return sr3.distance();
  }

  if (sensor == 4) {
    return sr4.distance();
  }
  if (sensor == 5) {
    return sr5.distance();
  }
  if (sensor == 6) {
    return sr6.distance();
  }
}

// This function returns a String of "1", "2" or "-1" for the specified sensor
String getDistanceOutput(int sensor)
{
  double sum = 0;
  double average = 0;

  // front left sensor SRFL
  if (sensor == 1) {
    // Get the sum of 10 values
    for (int i = 0; i < 10; i++) {
      sum = sum + sr1.distance();
    }
    average = sum / 10;
    if (1 <= average && average <= 16) {
      return "1";
    }
    else if (average >= 16 && average <= 27.5) {
      return "2";
    }
    else {
      return "-1";
    }
  }

  // front right sensor SRFR
  if (sensor == 2) {
    for (int i = 0; i < 10; i++) {
      sum = sum + sr2.distance();
    }
    average = sum / 10;

    if (1 <= average && average <= 16) {
      return "1";
    }
    else if (16 <= average && average <= 30) {
      return "2";
    }
    else {
      return "-1";
    }
  }

  // front centre sensor SRFC
  if (sensor == 5) {
    for (int i = 0; i < 10; i++) {
      sum = sum + sr5.distance();
    }
    average = sum / 10;

    if (1 <= average && average <= 13) {
      return "1";
    }
    else if (13 <= average && average <= 23) {
      return "2";
    }
    else {
      return "-1";
    }
  }

  // left sensor SRL
  if (sensor == 3) {
    for (int i = 0; i < 10; i++) {
      sum = sum + sr3.distance();
    }
    average = sum / 10;

    if (8 <= average && average <= 21) {
      return "1";
    }
    else if (21 < average && average <= 33.3) {
      return "2";
    }
    else {
      return "-1";
    }
  }
  // left long range sensor SRL
  if (sensor == 6) {
    for (int i = 0; i < 10; i++) {
      sum = sum + sr6.distance();
    }
    average = sum / 10;
    if (0 <= average && average <= 27) {
      return "0";
    }
    else if (27 < average && average <= 35.5) {
      return "3";
    }
    else if (35.5 < average && average <= 46) {
      return "4";
    }

    if (46 < average && average <= 58) {//16
      return "5";
    }
    else if (58 < average && average <= 71) {
      return "6";
    }

    if (71 < average && average <= 82) {//16
      return "7";
    }
    else if (82 < average && average <= 94) {
      return "8";
    }
    else {
      return "-1";
    }
  }
  // right sensor SRR
  if (sensor == 4) {
    for (int i = 0; i < 10; i++) {
      sum = sum + sr4.distance();
    }
    average = sum / 10;

    if (7 <= average && average <= 23) { //1,11
      return "1";
    }
    //    else if (23 <= average && average <= 34) { //11,21
    //      return "2";
    //    }
    else {
      return "-1";
    }
  }

}

void printSensors()
{
  String data = "SDATA;SRFL_";
  String SRFC = ";SRFC_";
  String SRFR = ";SRFR_";
  String SRL = ";SRL_";
  String SRR = ";SRR_";
  String SRLL = ";SLRL_"; //Long range left sensor

  String finalString = data + getDistanceOutput(1) + SRFC + getDistanceOutput(5) + SRFR + getDistanceOutput(2) + SRL + getDistanceOutput(3) + SRR + getDistanceOutput(4) + SRLL + getDistanceOutput(6);

  while (printFlag == true) {
    Serial.println(finalString);
    printFlag = false;
  }
}

// Calibrate robot to targetDist from the wall
void calibrate() {
  int SPEEDL = 70;
  int SPEEDR = 70;
  while (getDistance(1) < 30 && getDistance(2) < 30 && count != 50)//30
  {
    if ((getDistance(1) >= 10.65 && getDistance(1) < 11) || (getDistance(2) >= 10.65 && getDistance(2) < 11))//10.6,11.5 //10.95,11.05
    {
      md.setBrakes(100, 100);
      break;
    }
    else if (getDistance(1) < 10.65 || getDistance(2) < 10.65)
    {
      md.setSpeeds(-SPEEDL, -SPEEDR);
    }
    else {
      md.setSpeeds(SPEEDL, SPEEDR);
    }
  }
  md.setBrakes(100, 100);
}

void CaliAngle()
{
  double targetDist = 10.5;
  int SPEEDL = 70;
  int SPEEDR = 70;
  int count = 0;
  while (getDistance(1) < 30 && getDistance(2) < 30)
  {
    if (count == 150) {
      md.setBrakes(100, 100);
      caliFlag = false;
      calibrate();
      break;
    }
    double distDiff = getDistance(1) - getDistance(2);
    if (distDiff >= 0.15 || distDiff <= -0.15) {
      if ((distDiff >= 0.15  && distDiff < 7) && (getDistance(1) >= targetDist)) {
        md.setSpeeds(SPEEDL, 0);
        count++;
      }
      else if ((distDiff <= -0.15 && distDiff > -7) && (getDistance(1) < targetDist)) {
        md.setSpeeds(-SPEEDL, 0);
        count++;
      }
      else if ((distDiff >= 0.15 && distDiff < 7) && (getDistance(1) < targetDist)) {
        md.setSpeeds(0, -SPEEDR);
        count++;
      }
      else if ((distDiff <= -0.15 && distDiff > -7) && (getDistance(1) >= targetDist)) {
        md.setSpeeds(0, SPEEDR);
        count++;
      }
    }
    else
    {
      md.setBrakes(100, 100);
      caliFlag = false;
      calibrate();
      break;
    }
  }
}
