#include <PID_v1.h>



#include <SPI.h>
#include <Pixy.h>
#include <PID_v1.h>
#include <Servo.h>


// This is the main Pixy object 123
Pixy pixy;
Servo Yservo;
int startLIMIT = 47;
int middleLIMIT = 79;
int endLIMIT = 90;
int UpFlag = 1;
int UpCorr;
int DownCorr;
int TempY;
int avg = 0;
float pos = 0;

static int i = 0;
int flag = 1;
int j, incomingByte;
uint16_t blocks;
char buf[200];
int e;
int y;
double Setpoint, Input, Output;

double aggKp = 0.09, aggKi = 0.03, aggKd = 0.02;

//double consKp = 0.06, consKi = 0.07, consKd = 0.05;


PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
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
  Serial.print("Starting..a a a ..\n");

  pixy.init();
  Setpoint = target;
  //turn the PID on
  myPID.SetOutputLimits(0, 15);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
  Output = 0;
  Yservo.attach(9);
  delay(3000);
  Yservo.write(47);
  delay(1000);
  Yservo.write(90);
  delay(1000);
  Yservo.write(90);
}


void loop()
{






  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    {
      sprintf(buf, "Detected %d:\n", blocks);
      for (j = 0; j < blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        y = pixy.blocks[j].y;
        {


          //***************PID***********PID*****PID*********PID******PID*****************
          Input = 199 - y;
          y = 199 - y;
          if (y < Setpoint)
          {
            UpFlag = 1;
          } else
          {
            UpFlag = 0;
          }
          double gap = abs(Setpoint - Input); //distance away from setpoint



          
          
            //we're far from setpoint, use aggressive tuning parameters
            myPID.SetTunings(aggKp, aggKi, aggKd);
            if (UpFlag)
            {
              UpCorr = middleLIMIT - Output;
              Yservo.write(UpCorr);  // the 0 means GO up faster
            } else
            {
              Output = Output;
              DownCorr = middleLIMIT + Output;
              Yservo.write(DownCorr);  //the endLimit means go down faster
            }
          

          TempY = y;
          if (y > Setpoint)
          {
            y = Setpoint - (y - Setpoint); // PROBLEM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          
          }
          if (myPID.Compute() == true)
          {
            y = TempY;

            //***************PID***********PID*****PID*********PID******PID*****************

            Serial.print("\n");
            analogWrite(1, (int)Output);


            sprintf(buf, "Data, Y:%d  ,target:%d  ,correction:%d  ,UpFlag:%d", y, target, (int)Output, UpFlag);


            Serial.print(buf);
            Serial.print("\n");
            sprintf(buf, "%03d %03d %03d\n", y, target, (int)Output);
            Serial.print(buf);


          } else {
            Serial.print("\nfalse\n");

          }
        }
      }
    }
  }else
  {
     Serial.print("\nCAN'T DITECT THE DRONE !!!!!\n");
     UpFlag=0;
       DownCorr = middleLIMIT+2 ;
              Yservo.write(DownCorr);  //the endLimit means go down faster
  }
}


void servoMove(float up)
{

}

