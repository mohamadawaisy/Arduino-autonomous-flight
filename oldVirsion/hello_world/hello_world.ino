
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
double SetpointY=160;
//***********Global Params***********\\



//***********Local Params***********\\
uint16_t blocks;
char buf[200];
int UpCorr;
int DownCorr;

double aggKp = 0.1, aggKi = 1, aggKd = 0.04;
//double aggKp = 0.09, aggKi =0 , aggKd =0 ;
//0.03   0.02
//***********Local Params***********\\



// This is the main Pixy object 123




int TempY;
int avg = 0;

//double consKp = 0.06, consKi = 0.07, consKd = 0.05;


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
  myPIDy.SetOutputLimits(-15, 15);
  myPIDy.SetSampleTime(20);
  myPIDy.SetMode(AUTOMATIC);

  myPIDx.SetOutputLimits(-5, 5);
  myPIDx.SetSampleTime(20);
  myPIDx.SetMode(AUTOMATIC);
  OutputY = 0;
  OutputX=0;
  
  while(!DetectFlag)
  {
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
            if (UpTarget)
            {
              UpCorr = YmiddleLIMIT - OutputY/3.5;
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
          if (myPIDy.Compute() == true)
          {
       //     y = TempY;

            //***************PID***********PID*****PID*********PID******PID*****************



            sprintf(buf, "Data, Y:%d  ,SetpointY:%d  ,correction:%d  ,UpTarget:%d  ,DetectFlag:%d ", (int)y, (int)SetpointY, (int)OutputY, UpTarget,DetectFlag);
             PrintToSerial(buf);
            
            sprintf(buf, "%03d %03d %03d\n", y, (int)SetpointY, (int)OutputY);
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
  
  int temp= 90;
  TempY=y;
  while (!isUp()&&temp>YstartLIMIT)
  {
      GetInput();
      temp--;
      Yservo.write(temp );  // the 0 means GO up faster
      delay(1000);
  }
  *YmiddleLIMIT=temp;
 // Yservo.write(*YmiddleLIMIT);
  sprintf(buf, "***  checkBattaryLevel finished successfully YmiddleLIMIT is: %d: ***", *YmiddleLIMIT);
  PrintToSerial(buf);
  *YmiddleLIMIT=temp;
}

int isUp()
{
   
  if ((y - TempY)>1)
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
  //UpTarget=0;
  TempCorr = YmiddleLIMIT+2 ;
  Yservo.write(TempCorr);  //the YendLIMIT means go down faster
  
}

void PrintToSerial(char* str)
{
   Serial.print("\n");
   Serial.print(str);
   Serial.print("\n");
}

