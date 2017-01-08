
   
#include <SPI.h>  
#include <Pixy.h>
#include <PID_v1.h>
#include <Servo.h> 


// This is the main Pixy object 123
Pixy pixy;
Servo Yservo;

float pos = 0; 

static int i = 0;
int flag=1;
  int j,incomingByte;
  uint16_t blocks;
  char buf[200]; 
  int e;
  int y;
double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
double aggKp=0.03, aggKi=0.01, aggKd=0.01;
double consKp=0.7, consKi=0.02, consKd=0.5;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
int target=150;
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
  Serial.print("Starting..\n");

  pixy.init();
  Setpoint=target;
  //turn the PID on
  myPID.SetOutputLimits(0,41);
  myPID.SetSampleTime(20);
  myPID.SetMode(AUTOMATIC);
}


void loop()
{ 
   Yservo.attach(9);
   Yservo.write((41-Output)); 
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
  //  if (i%20==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
    //  Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
       // Serial.print(buf); 
       // Serial.print(pixy.blocks[j].signature);
      //  pixy.blocks[j].print();
       
       y=pixy.blocks[j].y;
   //    if(y>130)
       {

//***************PID***********PID*****PID*********PID******PID*****************
          Input =199- y;
         y=199- y;
          
          double gap = abs(Setpoint-Input); //distance away from setpoint
         
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
          if(myPID.Compute()==true)
          {
           // Serial.print("\ntrue\n");
//***************PID***********PID*****PID*********PID******PID***************** 
               //Output1=34.543435;
              // i=(int)(Output);
           Serial.print("\n");
           analogWrite(1, (int)Output);
           sprintf(buf, "Data, Y:%d  ,target:%d  ,correction:%d", y,target,(int)Output);
           Serial.print(buf);
           Serial.print("\n");
           sprintf(buf, "%03d %03d %03d\n", y,target,(int)Output);
           Serial.print(buf);
           
         
          }else{
            Serial.print("\nfalse\n");
          
          }
       }
      }
    }
  }  
}


void servoMove(float up)
{
  
}

