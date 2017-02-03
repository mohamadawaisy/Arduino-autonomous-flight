
#include <SPI.h>
#include <Pixy.h>
#include <PID_v1.h>
#include <Servo.h>
void checkBattaryLevel(int* YmiddleLIMIT, Servo Yservo);

//***********Global Params***********\\
Pixy pixy;
Servo Yservo;
Servo Xservo;
int YstartLIMIT = 47;
int YmiddleLIMIT = 79;
int YendLIMIT = 90;


int XstartLIMIT = 0;
int XmiddleLIMIT = 45;
int XendLIMIT = 90;

int UpTarget = 1;
int RightTarget = 1;
int DetectFlag=0;
double SetpointX,OutputY,OutputX,x,y;
double SetpointY=120;
//***********Global Params***********\\



//***********Local Params***********\\
uint16_t blocks;
char buf[200];
int UpCorr;
int DownCorr;
int startY;
//double aggKp = 0.09, aggKi = 0.15, aggKd = 0.03;
//double aggKp = 0.1, aggKi = 1, aggKd = 0.04;
//double aggKp = 0.09, aggKi =0 , aggKd =0 ;
//0.03   0.02
//***********Local Params***********\\



// This is the main Pixy object 123




int TempY;
int avg = 0;

//double consKp = 0.06, consKi = 0.09, consKd = 0.01;

double aggKp=4, aggKi=0.3, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
PID myPIDy(&y, &OutputY, &SetpointY, aggKp, aggKi, aggKd, DIRECT);
PID myPIDx(&x, &OutputX, &SetpointX, aggKp, aggKi, aggKd, DIRECT);
int target = 120;
void setup()
{
  // tell servo to go to position in variable 'pos'

  Serial.begin(9600);
  /* do {
             if(flag==1)
             {
             flag=0;
               target=(int)Serial.read();
               Serial.print("        I received:         ");
             }else
             {
               // read the incoming byte:
                 incomingByte = Serial.read();

                 // say what you got:
                 Serial.print("        I received:         ");
                 Serial.println(incomingByte, DEC);
             }

         }while(Serial.available() <= 0);
         */
          PrintToSerial("Starting..a a a ..");
  pixy.init();
  SetpointY = target;
  //turn the PID on
  myPIDy.SetOutputLimits(-10, 10);
  myPIDy.SetSampleTime(30);
  myPIDy.SetMode(AUTOMATIC);

  myPIDx.SetOutputLimits(-5, 5);
  myPIDx.SetSampleTime(20);
  myPIDx.SetMode(AUTOMATIC);
  OutputY = 0;
  OutputX=0;
  
  while(!DetectFlag)
  {
     PrintToSerial("wait DetectFlag..");
    GetInput();
  }
  Xservo.attach(8);
  Yservo.attach(9);
  delay(3000);
  Yservo.write(47);
  delay(1000);
  Yservo.write(90);
  delay(1000);
  Yservo.write(90);
  
  
  checkBattaryLevel(&YmiddleLIMIT,Yservo);
}


void loop()
{
           GetInput();
         
         
          if (y < SetpointY)
          {
            myPIDy.SetControllerDirection(DIRECT);
            UpTarget = 0;
          } else
          {
           // myPIDy.SetControllerDirection(REVERSE);
            UpTarget = 1;
          }


      //    if (x < SetpointX)
      //    {
      //      myPIDx.SetControllerDirection(DIRECT);
      //      RightTarget = 1;
      //    } else
      //    {
     //       myPIDx.SetControllerDirection(REVERSE);
     //       RightTarget = 0;
     //     }
          //***************PID***********PID*****PID*********PID******PID*****************
          //double gap = abs(SetpointY - Input); //distance away from SetpointY

            //we're far from SetpointY, use aggressive tuning parameters
            //myPIDy.SetTunings(aggKp, aggKi, aggKd);
              
      
  
            if (OutputY<=0)
            {
              UpCorr = YmiddleLIMIT - OutputY/3.2;
              Yservo.write(UpCorr);  // the 0 means GO up faster
            } else
            {
              OutputY = OutputY;
              DownCorr = YmiddleLIMIT - OutputY;
              Yservo.write(DownCorr);  //the YendLIMIT means go down faster
            }
          


        //     if (RightTarget)     ////TODO***************************************************
        //    {
        //      UpCorr = XmiddleLIMIT + OutputX/2;
        //      Yservo.write(UpCorr);  // the 0 means GO up faster
        //    } else
        //    {
        //      OutputY = OutputY;
        //      DownCorr = XmiddleLIMIT + OutputX;
        //      Yservo.write(DownCorr);  //the YendLIMIT means go down faster
        //    }

       //   TempY = y;
       //   if (y > SetpointY)
       //   {
       //     y = SetpointY - (y - SetpointY); // PROBLEM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       //   }

           double gap = abs(SetpointY-y); //distance away from setpoint
         
  if (gap < 20)
  {  //we're close to setpoint, use conservative tuning parameters
    myPIDy.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPIDy.SetTunings(aggKp, aggKi, aggKd);
  }
          if (myPIDy.Compute() == true)
          {
       //     y = TempY;

            //***************PID***********PID*****PID*********PID******PID*****************



            sprintf(buf, "Data, Y:%d  ,SetpointY:%d  ,correction:%d  ,UpTarget:%d  ,DetectFlag:%d ", (int)y, (int)SetpointY, (int)OutputY, UpTarget,DetectFlag);
             PrintToSerial(buf);
            
            sprintf(buf, "%03d %03d %03d\n", (int)y, (int)SetpointY, (int)OutputY);
            Serial.print(buf);


          } else {
            PrintToSerial("CAN'T COMPUTE PID!!");
          }
        
     
  
}


void servoMove(float up)
{

}

void checkBattaryLevel(int* YmiddleLIMIT, Servo Yservo)
{
  PrintToSerial("start check batary");
  int temp= 90;
  TempY=y;
  startY=y;
  while (!isUp2()&&temp>YstartLIMIT)
  {
    PrintToSerial("wait up..");
      GetInput();
      temp--;
      Yservo.write(temp -1);  // the 0 means GO up faster
      delay(300);
  }
  *YmiddleLIMIT=temp;
 // Yservo.write(*YmiddleLIMIT);
  sprintf(buf, "***  checkBattaryLevel finished successfully YmiddleLIMIT is: %d: ***", *YmiddleLIMIT);
  PrintToSerial(buf);
  sprintf(buf, "");
  
 
}
int isUp2()
{
   
  if ((y - startY)>3)
    return 1;
  else
    return 0;
}
int isUp()
{
   
  if ((y - TempY)>5)
    return 1;
  else
    return 0;
}

int GetInput()
{
  uint16_t blocks = pixy.getBlocks();
  int j;
  // If there are detect blocks, print them!
  if (blocks)
  {
    DetectFlag=1;
      for (j = 0; j < blocks; j++)
      {
        
        y = pixy.blocks[j].y;
        y = 199 - y;
          
        x = pixy.blocks[j].x;
      }
       sprintf(buf, "Data, Y:%d  ,SetpointY:%d  ,correction:%d  ,UpTarget:%d  ,DetectFlag:%d ***", (int)y, (int)SetpointY, (int)OutputY, UpTarget,DetectFlag);
             PrintToSerial(buf);
   }
   else
   {
    DetectFlag=0;
     CantDetect();
     
   }
}




void CantDetect()
{
  int TempCorr;
  PrintToSerial("CAN'T DITECT THE DRONE !!!!!");
  UpTarget=0;
  TempCorr = YmiddleLIMIT+2 ;
  Yservo.write(TempCorr);  //the YendLIMIT means go down faster
  
}

void PrintToSerial(char* str)
{
   Serial.print("\n");
   Serial.print(str);
   Serial.print("\n");
}

