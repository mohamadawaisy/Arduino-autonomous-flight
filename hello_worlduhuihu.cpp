#include <PID_v1.h>



#include <SPI.h>
#include <Pixy.h>
#include <PID_v1.h>
#include <Servo.h>

void CalculateCorr(float* Corr);
void checkBattaryLevel(int* middleLIMIT, Servo Yservo);
void InitializeController(int min, int max, Servo Yservo, int* middleLIMIT);
void initializeDroneConnection(int pin);

int isUp();
// This is the main Pixy object 123
Pixy pixy;
Servo Yservo;

int startLIMIT = 47;         //the joystick limits
int middleLIMIT = 80;
int endLIMIT = 90;

int UpTarget = 0;
int GoUp = 1;
int IsUp = 0;
float correction = 0;
int UpCorr;
int DownCorr;
int TempY;
int avg = 0;
float pos = 0;
float prevCorr = 0;

static int i = 0;
int flag = 1;
int j, incomingByte;
uint16_t blocks;
char buf[200];
int e;
int y;
int prevY;
int avgY;
double Setpoint, Input, Output;


int target = 60;
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

	initializeDroneConnection(9);
	InitializeController(47, 90, Yservo, &middleLIMIT);
}


void loop()
{

	// grab blocks!
	blocks = pixy.getBlocks();

	// If there are detect blocks, print them!
	prevY = (y + prevY) / 2;
	if (blocks)
	{
		i++;
		{
			//sprintf(buf, "Detected %d:\n", blocks);
			for (j = 0; j < blocks; j++)
			{
				//  sprintf(buf, "  block %d: ", j);

				y = pixy.blocks[j].y;
				{


					//***************PID***********PID*****PID*********PID******PID*****************
					Input = 199 - y;
					y = 199 - y;
					if (y < Setpoint)
					{
						GoUp = 1;
					}
					else
					{
						GoUp = 0;
					}
					double gap = abs(Setpoint - Input); //distance away from setpoint





														//we're far from setpoint, use aggressive tuning parameters
					CalculateCorr(&correction);

					if (UpTarget)
					{
						UpCorr = middleLIMIT - correction / 3;
						Yservo.write(UpCorr);  // the 0 means GO up faster
					}
					else
					{

						DownCorr = middleLIMIT - correction;
						Yservo.write(DownCorr);  //the endLimit means go down faster
					}


					Serial.print("\n");



					sprintf(buf, "Data, Y:%d  ,target:%d  ,correction:%d  ,UpTarget:%d  ,middleLIMIT:%d", y, target, (int)correction, UpTarget, middleLIMIT);


					Serial.print(buf);
					Serial.print("\n");
					sprintf(buf, "%03d %03d %03d\n", y, target, (int)Output);
					Serial.print(buf);
				}
			}
		}
	}
	else
	{
		Serial.print("\nCAN'T DITECT THE DRONE !!!!!\n");
		UpTarget = 0;
		DownCorr = middleLIMIT + 2;
		Yservo.write(DownCorr);  //the endLimit means go down faster
	}
}

void initializeDroneConnection(int pin)
{
	Yservo.attach(pin);
	delay(3000);
	Yservo.write(47);
	delay(1000);
	Yservo.write(90);
	delay(1000);
	Yservo.write(90);
}
void InitializeController(int min, int max, Servo Yservo, int* middleLIMIT)
{
	startLIMIT = min;         //the joystick limits
	checkBattaryLevel(middleLIMIT, Yservo);
	endLIMIT = max;
}

void checkBattaryLevel(int* middleLIMIT, Servo Yservo)
{

	int temp = 90;

	while (!isUp() && temp>startLIMIT)
	{
		blocks = pixy.getBlocks();
		if (blocks)
		{
			y = pixy.blocks[0].y;
			y = 199 - y;
		}

		temp--;
		Yservo.write(temp - 1);  // the 0 means GO up faster
		delay(270);
	}
	Yservo.write(*middleLIMIT - 2);
	sprintf(buf, "\n***  checkBattaryLevel finished successfully middleLIMIT is: %d: ***\n", *middleLIMIT);
	Serial.print(buf);
	*middleLIMIT = temp - 2;
}

int isUp()
{
	if ((y - prevY)>3)
		return 1;
	else
		return 0;
}

void CalculateCorr(float* Corr)
{

	float Afactor;
	if (y>target)
		UpTarget = 1;
	else
		UpTarget = 0;

	if (UpTarget)
	{
		if (isUp())
		{
			Afactor = (y - target);
			*Corr = -((Afactor / 7)*0.8 + prevCorr*0.2) / 2;
			//Afactor=(y-prevY);
			//*Corr=*Corr*(Afactor/2);
		}
		else {
			Afactor = (y - target);
			*Corr = -((Afactor / 10)*0.8 + prevCorr*0.2) / 2;
		}
		if (*Corr<-15)
		{
			*Corr = -15;
		}
	}
	else
	{
		if (isUp())
		{
			Afactor = (target - y);
			*Corr = +((Afactor / 5)*0.5 + prevCorr*0.5) / 2;
			//Afactor=(y-prevY);
			//*Corr=*Corr*(Afactor/2);
		}
		else
		{
			Afactor = (target - y);
			*Corr = +((Afactor / 2.6));
		}
		if (*Corr>15)
		{
			*Corr = 15;
		}
	}
	prevCorr = *Corr;
}


