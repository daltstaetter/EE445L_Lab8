// LSM9DSO.c
#include "LSM9DSO.h"
#include <stdio.h>
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "DAC.h"
#include "lm3s1968.h"
#include "fixed.h"
#include <stdlib.h>

#define SIZE 				6
#define HIGH 				1
#define LOW  				0
#define NEWLINE			0xD
#define TAB					0x9

void Delay(unsigned long ulCount);
void Fixed_sDecOut22s(unsigned long n);
void GyroStuff(unsigned long* ulDataTx, unsigned long* ulDataRx);
void AccelStuff(unsigned long* ulDataTx, unsigned long* ulDataRx);
void SPIread(enum sensor type, unsigned long* ulDataTx, unsigned long* ulDataRx);

extern unsigned long DataX[100];
extern unsigned long DataY[100];
extern unsigned long DataZ[100];
extern int angleX;

static enum gyro_scale gScale;
static enum accel_scale aScale;
//static enum mag_scale mScale;


/* gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
// it is scaled by 1,000,000 bc of integer division
*/
unsigned long gRes, aRes, mRes;

int currentAngleX = 0;
int currentAngleY = 0;
int currentAngleZ = 0;

long XLangleX = 0;
long XLangleY = 0;
long XLangleZ = 0;

// create a low pass filter to reduce the jitter
long filtGx[6] = {0};
long filtGy[6] = {0};
long filtGz[6] = {0};

long filtAx[6] = {0};
long filtAy[6] = {0};
long filtAz[6] = {0};

/* We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(void), readAccel(void), and readMag(void) first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
*/
short gx, gy, gz; // x, y, and z axis readings of the gyroscope
short ax, ay, az; // x, y, and z axis readings of the accelerometer
short mx, my, mz; // x, y, and z axis readings of the magnetometer

// xmAddress and gAddress store the SPI chip select pin
// for each sensor.
unsigned char xmAddress, gAddress;

extern void Delay(unsigned long ulCount);

// need it in radians
static long atan2(short y, short x)
{
	static long angle123=0;
	long lx;
	long ly;
	long swap = 0;;
	if(y > x)
	{
		swap = x;
		x = y;
		y = swap;
	}
	
	lx = (long)x;
	ly = (long)y;
	
	ly = (ly+50)/100;
	lx = (lx+50)/100;
	
	angle123 = (3*lx*lx-ly*ly+(3*lx*lx/2))/(3*lx*lx)*ly;
	angle123 += (((((((ly*ly+2)/5)*ly+lx/2)/lx)*ly+lx/2)/lx)*ly+lx/2)/(lx*lx*lx);
	if(angle123 > 1000)
	{	return 90000; } 
	angle123 *= 572;
	if(swap)
	{
		angle123 = 90000-angle123;
	}
	
	
/*	if(y==0)
//	{
//		if(x>=0)
//		{
//			return 0;
//		}
//		return 180;
//	}
//	else if(x == 0)
//	{
//		if(y>0)
//		{
//			return 90;
//		}
//		return -90;
//	}
//	else if(x>0 && y>0)
//	{
//		angle123 = 3667*lx*ly;
//		angle123 /= (64*lx*lx+17*ly*ly);
//		//angle = (3667*lx*ly)/(64*lx*lx+17*ly*ly);
//		return angle123;
//	}
//	else if(x>0 && y<0)
//	{
//		angle123 = 3667*lx*ly;
//		angle123 /= (64*lx*lx+17*ly*ly);
////		angle = (3667*lx*ly)/(64*lx*lx+17*ly*ly);
//		angle123 += 360;
//		return angle123;
//	}
//	else if(x<0 && y>0)
//	{
//		angle123 = 3667*abs(lx)*ly;
//		angle123 /= (64*lx*lx+17*ly*ly);
////		angle = (3667*abs(lx)*ly)/(64*abs(lx)*abs(lx)+17*ly*ly);
//		angle123 = 180 - angle123;
//		return angle123;
//	}
//	else
//	{
//		lx = abs(lx);
//		ly = abs(ly);
//		angle123 = (3667*lx*ly)/(64*lx*lx+17*ly*ly);
//		angle123 += 180;
//		return angle123;
//	}
*/
return angle123;
}

unsigned short begin(enum gyro_scale gScl, enum accel_scale aScl, enum mag_scale mScl,
enum gyro_odr gODR, enum accel_odr aODR, enum mag_odr mODR)
{
	//unsigned char gTest;
	//unsigned char xmTest;
	//unsigned long* testPtr;
	/* Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	//mScale = mScl;
	*/
	gScale = gScl;
	aScale = aScl;


	/* Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	//calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	*/
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	// Now, initialize our hardware interface.
//initSPI();	// Initialize SPI

	/* To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
  gTest = gReadByte(WHO_AM_I_G);	// Read the gyro WHO_AM_I
  xmTest = xmReadByte(WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I
	*/

	// Gyro initialization stuff:
initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
/*
//  setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
//	setGyroScale(gScale); // Set the gyro range
*/
	// Accelerometer initialization stuff:
initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
/*
	setAccelODR(aODR); // Set the accel data rate.
//	setAccelScale(aScale); // Set the accel range.
*/

	/* Magnetometer initialization stuff:
	//initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	//setMagODR(mODR); // Set the magnetometer output data rate.
	//setMagScale(mScale); // Set the magnetometer's range.
	*/

	// Once everything is initialized, return the WHO_AM_I registers we read:
	
return 0;// (xmTest << 8) | gTest;
}

// http://www.school-for-champions.com/algebra/square_root_approx.htm
unsigned long sqrt(unsigned long arg)
{
	//sqrt(arg) ~ 0/5*(arg/guess + guess)
	unsigned long error;
  unsigned long appValue;
	unsigned long guess = 12;
	error = 10;
	for(; error > 2; )
	{
		appValue = (arg+guess*guess+guess)/(guess*2);
		error = abs(appValue - guess);
		guess = appValue;		
	}
	return appValue;	
}

void chipSelectPin(enum sensor type, unsigned char value)
{
/*	if (type == GYRO)
//	{
//		if(value)
//		{
//			GPIO_PORTG_DATA_R |= 0x04;
//		}
//		else
//		{
//			GPIO_PORTG_DATA_R &= ~0x04;
//		}
//	}
//	
//	if (type == XM)
//	{
//		if(value)
//		{
//			GPIO_PORTG_DATA_R |= 0x01;
//		}
//		else
//		{
//			GPIO_PORTG_DATA_R &= ~0x01;
//		}
//	}
*/
	if (type == GYRO)
	{
		if(value)
		{
			GPIO_PORTB_DATA_R |= 0x04;
		}
		else
		{
			GPIO_PORTB_DATA_R &= ~0x04;
		}
	}
	
	if (type == XM)
	{
		if(value)
		{
			GPIO_PORTB_DATA_R |= 0x01;
		}
		else
		{
			GPIO_PORTB_DATA_R &= ~0x01;
		}
	}
}
/*#define ACCELEROMETER_SENSITIVITY 8192.0
//#define GYROSCOPE_SENSITIVITY 65.536
// 
//#define M_PI 3.14159265359	    
// 
//#define dt 0.01							// 10 ms sample rate!    
// 
//void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
//{
//    float pitchAcc, rollAcc;               
// 
//    // Integrate the gyroscope data -> int(angularSpeed) = angle
//    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
//    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
// 
//    // Compensate for drift with accelerometer data if !bullshit
//    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
//    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
//    {
//	// Turning around the X axis results in a vector on the Y-axis
//        pitchAcc = atan2((float)accData[1], (float)accData[2]) * 180 / M_PI;
//        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
// 
//	// Turning around the Y axis results in a vector on the X-axis
//        rollAcc = atan2((float)accData[0], (float)accData[2]) * 180 / M_PI;
//        *roll = *roll * 0.98 + rollAcc * 0.02;
//    }
//} 
*/


void SPIsingleByte(enum sensor type, unsigned long ulDataTx, unsigned long* ulDataRx)
{
	chipSelectPin(type,LOW);
	SSIDataPut(SSI1_BASE, ulDataTx);
	
	// Wait until SSI0 is done transferring all the data in the transmit FIFO.
	while(SSIBusy(SSI1_BASE))
	{}
	
	SSIDataGet(SSI1_BASE, ulDataRx);
	
	*ulDataRx &= 0x000000FF;
	chipSelectPin(type,HIGH);
}
unsigned long* GetData(enum sensor type, unsigned char numBytes, unsigned long* ulDataTx,unsigned long* ulDataRx)
{
	unsigned int i;
	//static unsigned long ulDataRx[SIZE];
	/*unsigned long ulDataTx[SIZE] = {0x00008F00,0x000009200,0x0000A000,0x0000A400,0x0000A500,0x0000A600};
	// ^^ XL important addresses
	//unsigned long ulDataTx[SIZE] = {0x0000A800,0x00000A900,0x0000AA00,0x0000AB00,0x0000AC00,0x0000AD00};
	// ^^ Gyro XYZ data addresses
	//unsigned long ulDataTx[SIZE] = {0x00008F00,0x00000A000,0x0000A100,0x0000A200,0x0000A300,0x0000A400};
	// ^^ Gyro important addresses
	*/
	
	// clear debug Rx buffer
	for(i=0;i<numBytes;i++)
	{
		ulDataRx[i] = 0;
	}	

	chipSelectPin(type,LOW);
	while(SSIDataGetNonBlocking(SSI1_BASE, &ulDataRx[i]))
	{
	}
	chipSelectPin(type,HIGH);
	Delay(10);
	
	for(i=0;i<numBytes;i++)
	{
		SPIsingleByte(type,ulDataTx[i],&ulDataRx[i]);
		
		if(i < numBytes)
		{
			printf("Tx: 0x%04X",(unsigned int)ulDataTx[i]); // prints 4 hex digits with leading zeros
			printf("%c", TAB);
			printf("Rx: 0x%02X",(unsigned int)ulDataRx[i]); // prints 2 hex digits with leading zeros
			printf("%c", NEWLINE);
			Delay(4000);           // delay ~1 sec at 12 MHz
		}
	}
	return  &ulDataRx[0];
}
void lowPassFilterData(enum sensor type, long dataX, long dataY, long dataZ)
{
	if(type == GYRO)
	{
		filtGx[5] = filtGx[4];
		filtGx[4] = filtGx[3];
		filtGx[3] = filtGx[2];
		filtGx[2] = filtGx[1];
		filtGx[1] = filtGx[0];
		filtGx[0] = dataX;

		filtGy[5] = filtGy[4];
		filtGy[4] = filtGy[3];
		filtGy[3] = filtGy[2];
		filtGy[2] = filtGy[1];
		filtGy[1] = filtGy[0];
		filtGy[0] = dataY;
		
		filtGz[5] = filtGz[4];
		filtGz[4] = filtGz[3];	
		filtGz[3] = filtGz[2];
		filtGz[2] = filtGz[1];
		filtGz[1] = filtGz[0];
		filtGz[0] = dataZ;
	}
	else
	{
		filtAx[5] = filtAx[4];
		filtAx[4] = filtAx[3];
		filtAx[3] = filtAx[2];
		filtAx[2] = filtAx[1];
		filtAx[1] = filtAx[0];
		filtAx[0] = dataX;

		filtAy[5] = filtAy[4];
		filtAy[4] = filtAy[3];
		filtAy[3] = filtAy[2];
		filtAy[2] = filtAy[1];
		filtAy[1] = filtAy[0];
		filtAy[0] = dataY;
		
		filtAz[5] = filtAz[4];
		filtAz[4] = filtAz[3];	
		filtAz[3] = filtAz[2];
		filtAz[2] = filtAz[1];
		filtAz[1] = filtAz[0];
		filtAz[0] = dataZ;
		
	}
}

void GetDataXYZ(enum sensor type)
{
/*	static int j = 0;
//	static int tempX = 0;
//	static int sum = 0;
//	int runSumX;
//	int runSumY;
//	int runSumZ;
	*/
	unsigned int i;
	short tempAx,tempAy,tempAz;
	unsigned long ulDataRx[SIZE];
	unsigned long ulDataTx[SIZE] = {0x0000A800,0x00000A900,0x0000AA00,0x0000AB00,0x0000AC00,0x0000AD00};
		
	SPIread(type, ulDataTx, ulDataRx);
	
	if(type == GYRO)
	{
		GyroStuff(ulDataTx,ulDataRx);
	}
	else
	{
		AccelStuff(ulDataTx, ulDataRx);
	}	
}
/////////////
void GyroStuff(unsigned long* ulDataTx, unsigned long* ulDataRx)
{
		gx = (ulDataRx[0]&0xFF) + ((ulDataRx[1]&0xFF) << 8); 
		gy = (ulDataRx[2]&0xFF) + ((ulDataRx[3]&0xFF) << 8);
		gz = (ulDataRx[4]&0xFF) + ((ulDataRx[5]&0xFF) << 8);
		
//		printf("%d%c",gx+545,NEWLINE);
//		printf("%d%c",gy-392,NEWLINE);
//		printf("%d%c%c",gz-4900,NEWLINE,NEWLINE);
		
		//gx = calcGyro(gx)+4095; // x offset data for 245DPS 
		//gy = calcGyro(gy)-3010; // y offset data for 245DPS 
		//gz = calcGyro(gz)+29376;//-12213; // z offset data for 245DPS 
		
		gx = calcGyro(gx+600);
		//Fixed_sDecOut22s((unsigned long) gx); // x offset data for 245DPS 
		gy = calcGyro(gy-392);
		//Fixed_sDecOut22s((unsigned long) gy); // y offset data for 245DPS 
		gz = calcGyro(gz-4980);
		//Fixed_sDecOut22s((unsigned long) gz);//-12213; // z offset data for 245DPS 
		
		
//		Fixed
//		Fixed_sDecOut22s((unsigned long) gx);
//		Fixed_sDecOut22s((unsigned long) gy);
//		Fixed_sDecOut22s((unsigned long) gz);
//		printf("%c",NEWLINE);printf("%c",NEWLINE);
		
		// http://www.hobbytronics.co.uk/accelerometer-gyro creates a good filter
		// output filter data
		lowPassFilterData(GYRO,(long)gx,(long)gy,(long)gz);
			gx = (filtGx[0]+filtGx[1]+filtGx[2]+filtGx[3])/4;
			gy = (filtGy[0]+filtGy[1]+filtGy[2]+filtGy[3])/4;
			gz = (filtGz[0]+filtGz[1]+filtGz[2]+filtGz[3])/4;

//			gx = (filtGx[0]+filtGx[1]+filtGx[2]+filtGx[3]+filtGx[4]+filtGx[5])/6;
//			gy = (filtGy[0]+filtGy[1]+filtGy[2]+filtGy[3]+filtGy[4]+filtGy[5])/6;
//			gz = (filtGz[0]+filtGz[1]+filtGz[2]+filtGz[3]+filtGz[4]+filtGz[5])/6;
		
		if(filtGx[0] - filtGx[1] < 150)
			gx = 0;
		if(filtGy[0] - filtGy[1] < 150)
			gy = 0;
		if(filtGz[0] - filtGz[1] < 150)
			gz = 0;
//		if(filtGx[1] - filtGx[0] < 150)
//			gx = 0;
//		if(filtGy[1] - filtGy[0] < 150)
//			gy = 0;
//		if(filtGz[1] - filtGz[0] < 150)
//			gz = 0;
		
//		Fixed_sDecOut22s((unsigned long) gx);
//		Fixed_sDecOut22s((unsigned long) gy);
//		Fixed_sDecOut22s((unsigned long) gz);
//		printf("%c",NEWLINE);
//		
	  //if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) 
	if (gx <= 24500 || gx >= -24500) 
	{
    currentAngleX += gx;
  }
	if (gy <= 24500 || gy >= -24500) 
	{
    currentAngleY += gy;
  }
	if (gz <= 24500 || gz >= -24500) 
	{
    currentAngleZ += gz;
  }
	
	//Keep our angle between 0-359 degrees
  if (currentAngleX < 0)
    currentAngleX += 36000;
  else if (currentAngleX > 35900)
    currentAngleX -= 36000;
	//Keep our angle between 0-359 degrees
  if (currentAngleY < 0)
    currentAngleY += 36000;
  else if (currentAngleY > 35900)
    currentAngleY -= 36000;
	//Keep our angle between 0-359 degrees
  if (currentAngleZ < 0)
    currentAngleZ += 36000;
  else if (currentAngleZ > 35900)
    currentAngleZ -= 36000;
	
	
//	Fixed_sDecOut22s((unsigned long) gx);						printf("%c",TAB);
//	Fixed_uDecOut2((unsigned long) currentAngleX);  printf("%c",NEWLINE);
//	Fixed_sDecOut22s((unsigned long) gy);						printf("%c",TAB);
//	Fixed_uDecOut2((unsigned long) currentAngleY);  printf("%c",NEWLINE);
//  Fixed_sDecOut22s((unsigned long) gz);					  printf("%c",TAB);
//	Fixed_uDecOut2((unsigned long) currentAngleZ);  printf("%c",NEWLINE);

	
	//		Fixed_sDecOut22s((unsigned long) currentAngleX);
//		Fixed_sDecOut22s((unsigned long) currentAngleY);
//		Fixed_sDecOut22s((unsigned long) currentAngleZ);
		printf("%c",NEWLINE);

/*		if(j < 100)
//		{
//			DataX[j] = gx;
//			DataY[j] = gy;
//			DataZ[j] = gz;
//			j++;
//		}
//		if( j == 100)
//		{
//			j = 0;
//			runSumX=0;
//			runSumY=0;
//			runSumZ=0;
//			for(j=0; j < 100; j++)
//			{
//				runSumX += DataX[j];
//				runSumY += DataY[j];
//				runSumZ += DataZ[j];
//				
//				if(j == 98)
//				{
//					j++;j--;
//				}
//			}
//			while(1)
//			{
//				printf("%d%c",runSumX/100,NEWLINE);
//				printf("%d%c",runSumY/100,NEWLINE);
//				printf("%d%c",runSumZ/100,NEWLINE);
//				while(1)
//				{}
//			}
//		}
		
//		printf("Gx: %d%c",gx,NEWLINE);
//		printf("Gy: %d%c",gy,NEWLINE);
//		printf("Gz: %d%c",gz,NEWLINE);
*/
	
	
	
	
}





long findAngleMeas(long y, long x)
{
	unsigned long magnitude;
	unsigned long xDegree;
	unsigned long yDegree;
	
	long xDegreeSigned;
	long yDegreeSigned;
	
	long xTemp;
	long yTemp;
	const unsigned long MAX = 16384;
	
	if(x > MAX)
		{x = MAX;}
	if(y > MAX)
		{y = MAX;}
	if( x < 0)
		{x = 0;}
	if(y < 0)
		{y = 0;}
	
	xTemp = atan2(y,x);
	printf("AngleX: %u%c",xTemp,NEWLINE);

	
/*	xTemp = x;
//	yTemp = y;
//	
//	
//	//xDegree = atan2(
//	
//	
//	if(xTemp > MAX)
//	{
//		xTemp = MAX;
//	}
//	else if(xTemp < -MAX)
//	{
//		xTemp = -MAX;
//	}
//	if(yTemp > MAX)
//	{
//		yTemp = MAX;
//	}
//	else if(yTemp < -MAX)
//	{
//		yTemp = -MAX;
//	}
//	
//	// put in the region of positive integers 0-32678
//	xTemp += MAX;
//	yTemp += MAX;
//	// zero -> -90 deg
//	// 2*MAX = 32768 -> 90 degrees
//	xDegree = (((((xTemp*100*355+37)/75)*179+56)/113)*24)/32768;
//	xDegreeSigned = (long)(xDegree-9000);
//	
//	yDegree = (((((yTemp*100*355+37)/75)*179+56)/113)*24)/32768;
//	xDegreeSigned = (long)(yDegree-9000);
//	
//	if (x < 0 && y > 0)
//	{
//		
//		
//	}
//	magnitude	= sqrt(x*x+y*y);
//	
//	
*/
return xTemp;
}

void AccelStuff(unsigned long* ulDataTx, unsigned long* ulDataRx)
{
	short tempAx,tempAy,tempAz;
	long lTempAx,lTempAy,lTempAz,mag;	
	
	tempAx = (ulDataRx[0]&0xFF) + ((ulDataRx[1]&0xFF) << 8)+2030; 
	tempAy = (ulDataRx[2]&0xFF) + ((ulDataRx[3]&0xFF) << 8);
	tempAz = (ulDataRx[4]&0xFF) + ((ulDataRx[5]&0xFF) << 8);	

	lTempAx = abs((long)tempAx);
	lTempAy = abs((long)tempAy);
	lTempAz = abs((long)tempAz);
	
	mag = 4*sqrt((lTempAy*lTempAy+8)/16+(lTempAz*lTempAz+8)/16);
	XLangleX = findAngleMeas(lTempAx,mag);
	
	mag = 4*sqrt((lTempAx*lTempAx+8)/16+(lTempAz*lTempAz+8)/16);
	XLangleY = findAngleMeas(lTempAy,mag);
	
	mag = 4*sqrt((lTempAx*lTempAx+8)/16+(lTempAy*lTempAy+8)/16);
	XLangleZ = findAngleMeas(lTempAz,mag);
	
	// This wont work for  the +/- 16g range
	ax = calcAccel(tempAx/((aScale+1)));
	ay = calcAccel(tempAy/((aScale+1)));////(aScale));
	az = calcAccel(tempAz/((aScale+1)));///(aScale));
	
	//ax = (2*ax + 21)/ 2; // calibration offset
	ay = (ay*100)/104-2;
	
//	mag = sqrt(ay*ay+az*az);
//	findAngleMeas(ax,mag);
//	
//	mag = sqrt(ax*ax+az*az);
//	findAngleMeas(ay,mag);
//	
//	mag = sqrt(ax*ax+ay*ay);
//	findAngleMeas(az,mag);
	
//	printf("%d%c",tempAx,NEWLINE);
//	printf("%d%c",tempAy,NEWLINE);
//	printf("%d%c",tempAz,NEWLINE);
	
// Filter accel data
////////////////////////////////////////////////////////////
		lowPassFilterData(XM,(long)ax,(long)ay,(long)az);
		ax = (filtAx[0]+filtAx[1]+filtAx[2]+filtAx[3])/4;
		ay = (filtAy[0]+filtAy[1]+filtAy[2]+filtAy[3])/4;
		az = (filtAz[0]+filtAz[1]+filtAz[2]+filtAz[3])/4;

//			ax = (filtAx[0]+filtAx[1]+filtAx[2]+filtAx[3]+filtAx[4]+filtAx[5])/6;
//			ay = (filtAy[0]+filtAy[1]+filtAy[2]+filtAy[3]+filtAy[4]+filtAy[5])/6;
//			az = (filtAz[0]+filtAz[1]+filtAz[2]+filtAz[3]+filtAz[4]+filtAz[5])/6;
////////////////////////////////////////////////////////////

//		Fixed_sDecOut3((long) ax);
//		Fixed_sDecOut3((long) ay);
//		Fixed_sDecOut3((long) az);
	
	Fixed_sDecOut22s((long) ax); printf("%d",ax);printf("%c",NEWLINE);//printf("%d%c",atan2(ax,ay),NEWLINE);
	Fixed_sDecOut22s((long) ay); printf("%d",ay);printf("%c",NEWLINE);//printf("%d%c",atan2(ay,az),NEWLINE);
	Fixed_sDecOut22s((long) az); printf("%d",az);printf("%c",NEWLINE);//printf("%d%c",atan2(ax,az),NEWLINE);
	Delay(250000);
//		
//		Fixed_uDecOut2((unsigned long) abs(ax/100));
//		Fixed_uDecOut2((unsigned long) abs(ay/100));
//		Fixed_uDecOut2((unsigned long) abs(az/100));
	
}
signed long calcGyro(short gyro)
{
	signed long scaledGyroData;
	//char sign = 0;
	if(gyro < 0)
	{
		//sign = -1;
		scaledGyroData = (gRes*(long)(~gyro+1) + 5000)/10000;
		scaledGyroData = ~scaledGyroData + 1;
	}
	else
	{
		scaledGyroData = (gRes*(long)gyro + 5000)/10000;
	}
	// Return the gyro raw reading times our pre-calculated DPS/(ADC tick):
	return scaledGyroData; 
}

signed long calcAccel(short accel)
{
	signed long scaledAccelData;
	if(accel < 0)
	{
		scaledAccelData = (aRes * (long)(~accel+1) + 50000)/100000;
		scaledAccelData = ~scaledAccelData + 1;
	}
	else
	{ scaledAccelData = (aRes * (long)accel + 50000)/100000; }
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return scaledAccelData;
}

void initGyro(void)
{
	unsigned long ulRead;
	unsigned long ulSend = 0x00000A000;//,0x0000A100,0x0000A200,0x0000A300,0x0000A400};
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	// Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	// DR[1:0] - Output data rate selection
	// 00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	// BW[1:0] - Bandwidth selection (sets cutoff frequency)
	// Value depends on ODR. See datasheet table 21.
	// PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	// Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled) 
	// gWriteByte(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	*/
DAC_Out(GYRO, 0x4F,CTRL_REG1_G,0);
//	GetData(GYRO,1,&ulSend,&ulRead);
	
	/* CTRL_REG2_G sets up the HPF
	// Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	// HPM[1:0] - High pass filter mode selection
	// 00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
	// 10=normal, 11=autoreset on interrupt
	// HPCF[3:0] - High pass filter cutoff frequency
	// Value depends on data rate. See datasheet table 26.
	gWriteByte(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	*/
	DAC_Out(GYRO, 0x00, CTRL_REG2_G, 0);
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	// Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	// I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	// I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	// H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	// PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	// I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	// I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	// I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	// I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) 
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(CTRL_REG3_G, 0x88);
	*/

	/* CTRL_REG4_G sets the scale, update mode
	// Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	// BDU - Block data update (0=continuous, 1=output not updated until read
	// BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	// FS[1:0] - Full-scale selection
	// 00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	// ST[1:0] - Self-test enable
	// 00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	// SIM - SPI serial interface mode select
	// 0=4 wire, 1=3 wire 
  gWriteByte(CTRL_REG4_G, 0x00); // Set scale to 245 dps
	*/
DAC_Out(GYRO, 0x90, CTRL_REG4_G, 0); //BDU & 500 DPS

	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	// Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	// BOOT - Reboot memory content (0=normal, 1=reboot)
	// FIFO_EN - FIFO enable (0=disable, 1=enable)
	// HPen - HPF enable (0=disable, 1=enable)
	// INT1_Sel[1:0] - Int 1 selection configuration
	// Out_Sel[1:0] - Out selection configuration 
	gWriteByte(CTRL_REG5_G, 0x00);
		
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	configGyroInt(0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
	*/
}



void initAccel()
{
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	// Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	// BOOT - Reboot memory content (0: normal, 1: reboot)
	// FIFO_EN - Fifo enable (0: disable, 1: enable)
	// WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	// HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	// HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	// HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled) 
  //xmWriteByte(CTRL_REG0_XM, 0x00);
	*/
  DAC_Out(XM, 0x00, CTRL_REG0_XM,0);

	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	// Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	// AODR[3:0] - select the acceleration data rate:
	// 0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz,
	// 0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
	// 1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	// BDU - block data update for accel AND mag
	// 0: Continuous update
	// 1: Output registers aren't updated until MSB and LSB have been read.
	// AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
	// 0: Axis disabled, 1: Axis enabled 
  // xmWriteByte(CTRL_REG1_XM, 0x67); // 100Hz data rate, x/y/z all enabled
	// Serial.println(xmReadByte(CTRL_REG1_XM));
  */
  DAC_Out(XM, 0x6F, CTRL_REG1_XM,0);
	
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	// Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	// ABW[1:0] - Accelerometer anti-alias filter bandwidth
	// 00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	// AFS[2:0] - Accel full-scale selection
	// 000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	// AST[1:0] - Accel self-test enable
	// 00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	// SIM - SPI mode selection
	// 0=4-wire, 1=3-wire
  // xmWriteByte(CTRL_REG2_XM, 0x00); // Set scale to 2g
*/
  DAC_Out(XM, 0x00,CTRL_REG2_XM,0);
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	// Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY

	// Accelerometer data ready on INT1_XM (0x04)
  // xmWriteByte(CTRL_REG3_XM, 0x04);
	*/
}


// from the book, Sec. 7-5 pg 372
// send the 16-bit data to the SSI, return a reply
void DAC_Out(enum sensor type, unsigned char data, unsigned char subAddress, unsigned char csPin)
{	
	unsigned short TxBytes = 0;
	TxBytes |= (0x00FF & data);
	TxBytes &= SINGLEWRITE; // sets 2 MSB to 0
	TxBytes |= ((0x003F & subAddress) << 8); // shift 6-bit address bits into place, i.e. bit13 to bit8;
	
	// first write the address then the data
	/////////////////////////////////////
	// I need to add a specifier for the chip select bit
	// to differentiate among gyro, accel, & magnetometer
	/////////////////////////////////////
	//GPIO_PORTG_DATA_R &= ~0x04;// Open communication
	chipSelectPin(type,LOW);
	// write the MS_bit first
	// If write, bit 0 (MS) should be 0
	// If single write, bit 1 should be 0
	while(SSI1_SR_TNF == 0) 
	{}; // wait until room in FIFO
	SSI1_DR_R = TxBytes;  // data out
	Delay(60); // for some reason it needs this delay of >= 48, it doesn't get
						 // stored in the XL/G if not
	
	//GPIO_PORTG_DATA_R |= 0x04; // Close communication
	chipSelectPin(type,HIGH);
}

void initSPI()
{ 
	/*
	// Now set up the SPI port to talk to the LSM9DS0
  */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinConfigure(GPIO_PE0_SSI1CLK);
  GPIOPinConfigure(GPIO_PE1_SSI1FSS);
  GPIOPinConfigure(GPIO_PE2_SSI1RX);
  GPIOPinConfigure(GPIO_PE3_SSI1TX);
  GPIOPinTypeSSI(GPIO_PORTE_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
	
  /* LSM9DS0 SPI Specs
  // Max SPI Clock : 10 MHz
  // Data Order : MSB transmitted first
  // Clock Polarity: high when idle => SPO = 1;
  // Clock Phase : sample on rising edge => SPH = 1
  //
  // We read and write 8 bits to the LSM9DS0 but we need the Stellaris to drive the clock.  
	// We send 16 bits, MSB are specifiers(R/W,Inc,Address respectively), LSB is data (send MSbit 1st)
	// R/W - bit 15, Read = 1, Write = 0;
	// Inc - bit 14, (Auto Increment Address) Inc = 1
	// Address - bit 13 to bit 8, MOSI address to Tx data to
	// data - bit 7 to bit 0, data to store at said address
	// When we do a write, all 16 bits are written.
  // When we do a read, we take the command byte and shift left 8 bits, writing 0's as the last eight bits.
  // Since this mode is full duplex, we'll get 16 bits back but mask off the top 8, leaving only the read data we are
  */
  SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 5000000, 16);
  SSIEnable(SSI1_BASE);
}

void calcgRes()
{
	/* Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	// look at http://electronics.stackexchange.com/questions/39024/how-do-i-get-gyro-sensor-data-l3g4200d-into-degrees-sec

	*/
	switch (gScale)
	{
		case G_SCALE_245DPS:
		gRes = (245*1000000 + 16384)/32768;
		break;
		case G_SCALE_500DPS:
		gRes = (500*1000000 + 16384)/32768;
		break;
		case G_SCALE_2000DPS:
		gRes = (2000*1000000 + 16384)/32768;
		break;
	}
}

void calcaRes()
{
	/* Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	*/
	aRes = aScale == A_SCALE_16G ? (16.0*1000000) / 32768.0 :
	((((unsigned long) aScale + 1.0)*10000000) * 2.0) / 32768.0;
	printf("%caRes: %lu %c",NEWLINE, aRes,NEWLINE);
}

void SPIread(enum sensor type, unsigned long* ulDataTx, unsigned long* ulDataRx)
{
	unsigned char i;
		// Clear Rx Buffer
	for(i=0;i<SIZE;i++)
	{
		ulDataRx[i] = 0;
	}	
	/* Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	*/
	chipSelectPin(type,LOW);
	while(SSIDataGetNonBlocking(SSI1_BASE, &ulDataRx[i]))
	{
	}
	chipSelectPin(type,HIGH);
	Delay(1000);
	/* Send the data using the "blocking" put function.  This function
	// will wait until there is room in the send FIFO before returning.
	// This allows you to assure that all the data you send makes it into
	// the send FIFO.
	*/
	
	// debuggin on logic analyzer to calculate  
	// dT for the Complimentary filter
	if(type == GYRO)
		{	GPIO_PORTG_DATA_R ^= 0x01;}
	for(i=0;i<SIZE;i++)
	{
		/*
		// Send the data using the "blocking" put function.  This function
		// will wait until there is room in the send FIFO before returning.
		// This allows you to assure that all the data you send makes it into
		// the send FIFO.
		*/
		chipSelectPin(type,LOW);
		SSIDataPut(SSI1_BASE, ulDataTx[i]);
		/*
		// Wait until SSI1 is done transferring all the data in the transmit FIFO.
		*/
		while(SSIBusy(SSI1_BASE))
		{}
		/*
		// Receive the data using the "blocking" Get function. This function
		// will wait until there is data in the receive FIFO before returning.
		*/
		SSIDataGet(SSI1_BASE, &ulDataRx[i]);
		/*
		// Since we are using 8-bit data, mask off the MSB that was read full
		// duplex while we were sending our command byte.
		*/ 
		ulDataRx[i] &= 0x000000FF;
		/*
		// Display the data that SSI1 received.
		// The datasheet says this value should be 4.
		*/
		chipSelectPin(type,HIGH);
	}
}

/*void initMag()
{	
	//  CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
	// Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
	// TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
	// M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
	// M_ODR[2:0] - Magnetometer data rate select
	// 000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
	// LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
	// 0=interrupt request not latched, 1=interrupt request latched
	// LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
	// 0=irq not latched, 1=irq latched
	xmWriteByte(CTRL_REG5_XM, 0x14); // Mag data rate - 100 Hz

	//CTRL_REG6_XM sets the magnetometer full-scale
	//Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
	//MFS[1:0] - Magnetic full-scale selection
	//00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs 
	xmWriteByte(CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS

	//CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
	//AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
	//AHPM[1:0] - HPF mode selection
	//00=normal (resets reference registers), 01=reference signal for filtering,
	//10=normal, 11=autoreset on interrupt event
	//AFDS - Filtered acceleration data selection
	//0=internal filter bypassed, 1=data from internal filter sent to FIFO
	//MLP - Magnetic data low-power mode
	//0=data rate is set by M_ODR bits in CTRL_REG5
	//1=data rate is set to 3.125Hz
	//MD[1:0] - Magnetic sensor mode selection (default 10)
	//00=continuous-conversion, 01=single-conversion, 10 and 11=power-down
	xmWriteByte(CTRL_REG7_XM, 0x00); // Continuous conversion mode

	//CTRL_REG4_XM is used to set interrupt generators on INT2_XM
	//Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM

	xmWriteByte(CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)

	//INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
	//Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
	//XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
	//PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
	//IEA - Interrupt polarity for accel and magneto
	//0=active-low, 1=active-high
	//IEL - Latch interrupt request for accel and magneto
	//0=irq not latched, 1=irq latched
	//4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
	//MIEN - Enable interrupt generation for magnetic data
	//0=disable, 1=enable) 
	xmWriteByte(INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
}

void readAccel()
{
	unsigned char temp[6]; // We'll read six bytes from the accelerometer into temp
	xmReadBytes(OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void readMag()
{
	unsigned char temp[6]; // We'll read six bytes from the mag into temp
	xmReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void readGyro()
{
	unsigned char temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

signed long calcMag(short mag)
{
	signed long scaledMagData = (mRes * (long)mag)/10000;

	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return scaledMagData;
}

void setGyroScale(enum gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	unsigned char temp = gReadByte(CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void setAccelScale(enum accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void setMagScale(enum mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(CTRL_REG6_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void setGyroODR(enum gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	unsigned char temp = gReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp);
	
}
	
void setAccelODR(enum accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_XM, temp);
}
	
void setMagODR(enum mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(CTRL_REG5_XM, temp);
}

void configGyroInt(unsigned char int1Cfg, unsigned short int1ThsX, unsigned short int1ThsY, unsigned short int1ThsZ, unsigned char duration)
{
	gWriteByte(INT1_CFG_G, int1Cfg);
	gWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	
	if (duration)
	{
		gWriteByte(INT1_DURATION_G, 0x80 | duration);
	}
	else
	{
		gWriteByte(INT1_DURATION_G, 0x00);
	}
}

void calcmRes()
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	mRes = mScale == M_SCALE_2GS ? (2.0*1000000) / 32768.0 :
	(unsigned long) ((mScale << 2)*1000000) / 32768.0;
}

void gWriteByte(unsigned char subAddress, unsigned char data)
{
	// write a byte using the
	// gyro-specific address SPI CS pin.
	SPIwriteByte(gAddress, subAddress, data);
}

void xmWriteByte(unsigned char subAddress, unsigned char data)
{
	// write a byte using the
	// accelerometer-specific PI CS pin.
	SPIwriteByte(xmAddress, subAddress, data);
}

unsigned char gReadByte(unsigned char subAddress)
{
	// read a byte using the
	// gyro-specific SPI CS pin.

	return SPIreadByte(gAddress, subAddress);
}

void gReadBytes(unsigned char subAddress, unsigned char * dest, unsigned char count)
{
	// read multiple bytes using the
	// gyro-specific SPI CS pin.
	SPIreadBytes(gAddress, subAddress, dest, count);
}

unsigned char xmReadByte(unsigned char subAddress)
{
	// read a byte using the
	// accelerometer-specific SPI CS pin.
	return SPIreadByte(xmAddress, subAddress);
}

void xmReadBytes(unsigned char subAddress, unsigned char * dest, unsigned char count)
{
	// read multiple bytes using the
	// accelerometer-specific SPI CS pin.

	SPIreadBytes(xmAddress, subAddress, dest, count);
}

void SPIwriteByte(unsigned char csPin, unsigned char subAddress, unsigned char data)
{
//	digitalWrite(csPin, LOW); // Initiate communication

//	// If write, bit 0 (MSB) should be 0
//	// If single write, bit 1 should be 0
//	SPI.transfer(subAddress & 0x3F); // Send Address
//	SPI.transfer(data); // Send data

//	digitalWrite(csPin, HIGH); // Close communication
}

unsigned char SPIreadByte(unsigned char csPin, unsigned char subAddress)
{
	unsigned char temp;
//	// Use the multiple read function to read 1 byte.
//	// Value is returned to `temp`.
//	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

void SPIreadBytes(unsigned char csPin, unsigned char subAddress,
unsigned char * dest, unsigned char count)
{
//	digitalWrite(csPin, LOW); // Initiate communication
//	// To indicate a read, set bit 0 (msb) to 1
//	// If we're reading multiple bytes, set bit 1 to 1
//	// The remaining six bytes are the address to be read
//	if (count > 1)
//	SPI.transfer(0xC0 | (subAddress & 0x3F));
//	else
//	SPI.transfer(0x80 | (subAddress & 0x3F));
//	for (int i=0; i<count; i++)
//	{
//		dest[i] = SPI.transfer(0x00); // Read into destination array
//	}
//	digitalWrite(csPin, HIGH); // Close communication
}
*/


