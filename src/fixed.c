#include <stdio.h>
#include "fixed.h"
#include <string.h>
#include "Output.h"

extern void Delay(unsigned long ulCount);

// const will place these structures in ROM
//const struct outTestCase{       // used to test routines
//  unsigned long InNumber;       // test input number
//  char OutBuffer[10];           // Output String  
//};
//typedef const struct outTestCase outTestCaseType;
//outTestCaseType outTests3[16]={ 
//{     0,  "  0.00" }, //      0/256 = 0.00  
//{     4,  "  0.01" }, //      4/256 = 0.01  
//{    10,  "  0.03" }, //     10/256 = 0.03
//{   200,  "  0.78" }, //    200/256 = 0.78
//{   254,  "  0.99" }, //    254/256 = 0.99
//{   505,  "  1.97" }, //    505/256 = 1.97
//{  1070,  "  4.17" }, //   1070/256 = 4.17
//{  5120,  " 20.00" }, //   5120/256 = 20.00
//{ 12184,  " 47.59" }, //  12184/256 = 47.59
//{ 26000,  "101.56" }, //  26000/256 = 101.56
//{ 32767,  "127.99" }, //  32767/256 = 127.99
//{ 32768,  "128.00" }, //  32768/256 = 128
//{ 34567,  "135.02" }, //  34567/256 = 135.02
//{123456,  "482.25" }, // 123456/256 = 482.25
//{255998,  "999.99" }, // 255998/256 = 999.99
//{256000,  "***.**" }  // error
//};
//unsigned int Errors,AnError;
char Buffer[8];

/****************Fixed_uDecOut2s***************
 converts fixed point number to ASCII string
 format unsigned 32-bit with resolution 0.01
 range 0 to 999.99
 Input: unsigned 32-bit integer part of fixed point number
         greater than 99999 means invalid fixed-point number
 Output: null-terminated string exactly 6 characters plus null
 Examples
 12345 to "123.45"  
 22100 to "221.00"
   102 to "  1.02" 
    31 to "  0.31" 
100000 to "***.**"    */ 
void Fixed_uDecOut2s(unsigned long n,  char *string)
{
	int hundreds;
	int tens;
	int ones;
	int tenths;
	int one_hundredths;
	int temp_num;
	
	if (n > 99999)
	{
		// return invalid fixed-point number as ***.**
		string[0] = '*';
		string[1] = '*';
		string[2] = '*';
		string[3] = '.';
		string[4] = '*';
		string[5] = '*';
		string[6] = 0;

		return;
	}

	else
	{
		temp_num = n;

		hundreds = n/10000;    					// capture fix-point hundreds place
		temp_num %= 10000;     					// temp_num is now between 0-9999
	
		tens = temp_num/1000;   				// capture fix-point tens place
		temp_num %= 1000;     					// temp_num is now between 0-999
	
		ones = temp_num/100;   					// capture fix-point ones place
		temp_num %= 100;       					// temp_num is now between 0-99
	
		tenths = temp_num/10;   				// capture fix-point tenths place
		temp_num %= 10;                         // temp_num is now between 0-9
	
		one_hundredths = temp_num;   		    // capture fix-point one_hundredths place
	
		string[0] = hundreds + '0';
		string[1] = tens + '0';
		string[2] = ones + '0';
		string[3] = '.';
		string[4] = tenths + '0';
		string[5] = one_hundredths + '0';
		string[6] = 0;

		if (string[0] == '0')
		{
			string[0] = ' ';

			if(string[1] == '0')
			{
				string[1] = ' ';
			}
		}

		return;
	}
}


 /****************Fixed_uDecOut2***************
 outputs the fixed-point value on the OLED
 format unsigned 32-bit with resolution 0.01
 range 0 to 999.99
 Input: unsigned 32-bit integer part of fixed point number
         greater than 99999 means invalid fixed-point number
 Output: none
 Examples
 12345 to "123.45"  
 22100 to "221.00"
   102 to "  1.02" 
    31 to "  0.31" 
100000 to "***.**"    */ 
void Fixed_uDecOut2(unsigned long n) 
{
	Fixed_uDecOut2s(n,Buffer);
	printf("%s",Buffer);
	return;
}
//
void Fixed_sDecOut22s(unsigned long n) 
{
	int i;
  long nTemp = n;
	if(nTemp<0)
	{
		Buffer[0] = ' ';
		nTemp = ~nTemp+1; // get positive value
		Fixed_uDecOut2s(nTemp,&Buffer[1]);
		for(i=0;Buffer[i] == ' ';i++)
		{}
		Buffer[--i] = '-';
	}
	else
	{ 
		Buffer[0] = ' ';
		Fixed_uDecOut2s(n,&Buffer[1]);
	}
	printf("%s ",Buffer);
	return;
}
////////////////////////////////////

 /****************Fixed_uDecOut2***************
 outputs the fixed-point value on the OLED
 format unsigned 32-bit with resolution 0.01
 range 0 to 999.99
 Input: unsigned 32-bit integer part of fixed point number
         greater than 99999 means invalid fixed-point number
 Output: none
 Examples
 12345 to "123.45"  
 22100 to "221.00"
   102 to "  1.02" 
    31 to "  0.31" 
100000 to "***.**"    */ 
void Fixed_sDecOut2(long n) 
{
	Fixed_uDecOut2s(n,Buffer);
	printf("%s",Buffer);
	return;
}
//
void Fixed_sDecOut2s(unsigned long n) 
{
	int i;
  long nTemp = n;
	
	if(nTemp<0)
	{
		nTemp = ~nTemp+1; // get positive value
		Fixed_uDecOut2s(nTemp,Buffer);
		for(i=0;Buffer[i] == ' ';i++)
		{}
		Buffer[--i] = '-';
	}
	else
	{ 
		Fixed_uDecOut2s(n,Buffer);
	}
	
	printf("%s",Buffer);
	return;
}
/****************Fixed_sDecOut3s***************
 converts fixed point number to ASCII string
 format signed 32-bit with resolution 0.001
 range -9.999 to +9.999
 Input: signed 32-bit integer part of fixed point number
 Output: null-terminated string exactly 6 characters plus null
 Examples
  2345 to " 2.345"  
 -8100 to "-8.100"
  -102 to "-0.102" 
    31 to " 0.031" 
   
 */ 
void Fixed_sDecOut3s(long n, char *string) 
{
	
	int temp_num;
	int ones;
	int tenths;
	int one_hundredths;
	int one_thousandths;
	
	if (n > 9999 || n < -9999)
		n /= 10;
	
	if (n > 9999 || n < -9999)
	{
		// return invalid fixed-point number as *.***
		string[0] = ' ';
		string[1] = '*';
		string[2] = '.';
		string[3] = '*';
		string[4] = '*';
		string[5] = '*';
		string[6] = 0;
	}
	else {
		
		
		//checks if negative
		if(n < 0) {
			string[0] = '-';
			n = -1*n;
		}
		else {
			string[0] = ' ';
		}
		
		temp_num = n;

		ones = n/1000;    					// capture fix-point hundreds place
		temp_num %= 1000;     					// temp_num is now between 0-9999
		string[1] = ones + 0x30;
		
		string[2] = 0x2E;						// '.' = 0x2E
		
		tenths = temp_num/100;   				// capture fix-point tens place
		temp_num %= 100;     					// temp_num is now between 0-999
		string[3] = tenths + 0x30;
		
		one_hundredths = temp_num/10;   				// capture fix-point tenths place
		temp_num %= 10;  								// temp_num is now between 0-9
		string[4] = one_hundredths + 0x30;
		
		
		one_thousandths = temp_num;   		// capture fix-point one_hundredths place
		string[5] = one_thousandths + 0x30;
		
		string[6] = 0;									//adds null
	}
	
	return;
}
//////////////////////////////////////////
void Fixed_sDecOut3sss(long n, char *string) 
{
	
	int temp_num;
	int ones;
	int tenths;
	int one_hundredths;
	int one_thousandths;
	
	if (n > 9999 || n < -9999)
	{
		// return invalid fixed-point number as *.***
		string[0] = ' ';
		string[1] = '*';
		string[2] = '.';
		string[3] = '*';
		string[4] = '*';
		string[5] = '*';
		string[6] = 0;
	}
	else {
		
		
		//checks if negative
		if(n < 0) {
			string[0] = '-';
			n = -1*n;
		}
		else {
			string[0] = ' ';
		}
		
		temp_num = n;

		ones = n/1000;    					// capture fix-point hundreds place
		temp_num %= 1000;     					// temp_num is now between 0-9999
		string[1] = ones + 0x30;
		
		string[2] = 0x2E;						// '.' = 0x2E
		
		tenths = temp_num/100;   				// capture fix-point tens place
		temp_num %= 100;     					// temp_num is now between 0-999
		string[3] = tenths + 0x30;
		
		one_hundredths = temp_num/10;   				// capture fix-point tenths place
		temp_num %= 10;  								// temp_num is now between 0-9
		string[4] = one_hundredths + 0x30;
		
		
		one_thousandths = temp_num;   		// capture fix-point one_hundredths place
		string[5] = one_thousandths + 0x30;
		
		string[6] = 0;									//adds null
	}
	
	return;
}


/****************Fixed_sDecOut3***************
 converts fixed point number to OLED
 format signed 32-bit with resolution 0.001
 range -9.999 to +9.999
 Input: signed 32-bit integer part of fixed point number
 Output: none
 OLED has exactly 6 characters
 Examples
  2345 to " 2.345"  
 -8100 to "-8.100"
  -102 to "-0.102" 
    31 to " 0.031" 
 */ 
void Fixed_sDecOut3(long n) 
{
	Fixed_sDecOut3s(n,Buffer);
	printf("%s%c",Buffer,NEWLINE);
	return;
}


/**************Fixed_uBinOut8s***************
 unsigned 32-bit binary fixed-point with a resolution of 1/256. 
 The full-scale range is from 0 to 999.99. 
 If the integer part is larger than 256000, it signifies an error. 
 The Fixed_uBinOut8 function takes an unsigned 32-bit integer part 
 of the binary fixed-point number and outputs the fixed-point value on the OLED. 
 Input: unsigned 32-bit integer part of fixed point number
 Output: null-terminated string
Parameter output string
     0     "  0.00"
     2     "  0.01"
    64     "  0.25"
   100     "  0.39"
   500     "  1.95"
   512     "  2.00"
  5000     " 19.53"
 30000     "117.19"
255997     "999.99"
256000     "***.**"
*/
void Fixed_uBinOut8s(unsigned long n,  char *string)
{
	// put into fixed-point format then call snprintf
	
   int remainder;
	int fraction;
	int integer;

//	char a[6] = {0};
//	char c[6] = {0};

	fraction = 0;
	integer = 0;


	if ( n > 255999)
	{
		string[0] = '*';
		string[1] = '*';
		string[2] = '*';
		string[3] = '.';
		string[4] = '*';
		string[5] = '*';
		string[6] = 0;
		return;
	}

	remainder = n & 0x000000FF; // isolates the decimal digits, bits 0-7

	integer = n/256;

	if ( remainder == 255)
		remainder = 254; // prevents rollover from the next eqn
	fraction = (remainder*25)/64;  // eqn is reduced from (1/256)*(remainder*99+32)

	snprintf(&string[0],3,"%3d",integer);      // this is a microsoft function & isn't the...
	string[3] = '.';                            // ...standard snprintf, it doesn't null terminate
	snprintf(&string[4],2,"%02d",fraction);
	string[6] = 0; // not really necessary but just to be safe

	return;
}

/**************Fixed_uBinOut8***************
 unsigned 32-bit binary fixed-point with a resolution of 1/256. 
 The full-scale range is from 0 to 999.99. 
 If the integer part is larger than 256000, it signifies an error. 
 The Fixed_uBinOut8 function takes an unsigned 32-bit integer part 
 of the binary fixed-point number and outputs the fixed-point value on the OLED. 
 Input: unsigned 32-bit integer part of fixed point number
 Output: none
Parameter OLED display
     0	  0.00
     2	  0.01
    64	  0.25
   100	  0.39
   500	  1.95
   512	  2.00
  5000	 19.53
 30000	117.19
255997	999.99
256000	***.**
*/
void Fixed_uBinOut8(unsigned long n)
{
	Fixed_uBinOut8s(n,Buffer);
	printf("%s%c",Buffer,NEWLINE);
	return;
}
	

int main1(void)
{ // possible main program that tests your functions
	//unsigned int i;
	
	
	Output_Init();
  Output_Color(15);	
  //printf("Hello, world.");
  //printf("%c", NEWLINE);
  Delay(4000000);           // delay ~1 sec at 12 MHz
	

	//printf("long: %d\n",sizeof(long));
	//printf("int: %d\n",sizeof(int));
	//printf("double: %d\n",sizeof(double));
	//printf("float: %d\n",sizeof(float));
	//printf("char: %d\n",sizeof(char));
	//printf("long long: %d\n",sizeof(long long));
	
	Fixed_uBinOut8(0);
	Fixed_uBinOut8(2);
	Fixed_uBinOut8(64);
	Fixed_uBinOut8(100);
	Fixed_uBinOut8(500);
	Fixed_uBinOut8(512);
	Fixed_uBinOut8(5000);
	Fixed_uBinOut8(30000);
	Fixed_uBinOut8(255997);
	Fixed_uBinOut8(256000);

	Fixed_uBinOut8(0);
	Fixed_uBinOut8(4);
	Fixed_uBinOut8(10);
	Fixed_uBinOut8(200);
	Fixed_uBinOut8(254);
	Fixed_uBinOut8(505);
	Fixed_uBinOut8(1070);
	Fixed_uBinOut8(5120);
	Fixed_uBinOut8(12184);
	Fixed_uBinOut8(26000);

	Fixed_uBinOut8(32767);
	Fixed_uBinOut8(32768);
	Fixed_uBinOut8(34567);
	Fixed_uBinOut8(123456);
	Fixed_uBinOut8(255998);
	Fixed_uBinOut8(256000);
//
//	{     0,  "  0.00" }, //      0/256 = 0.00  
//{     4,  "  0.01" }, //      4/256 = 0.01  
//{    10,  "  0.03" }, //     10/256 = 0.03
//{   200,  "  0.78" }, //    200/256 = 0.78
//{   254,  "  0.99" }, //    254/256 = 0.99
//{   505,  "  1.97" }, //    505/256 = 1.97
//{  1070,  "  4.17" }, //   1070/256 = 4.17
//{  5120,  " 20.00" }, //   5120/256 = 20.00
//{ 12184,  " 47.59" }, //  12184/256 = 47.59
//{ 26000,  "101.56" }, //  26000/256 = 101.56
//{ 32767,  "127.99" }, //  32767/256 = 127.99
//{ 32768,  "128.00" }, //  32768/256 = 128
//{ 34567,  "135.02" }, //  34567/256 = 135.02
//{123456,  "482.25" }, // 123456/256 = 482.25
//{255998,  "999.99" }, // 255998/256 = 999.99
//{256000,  "***.**" }  // error
//};




	//getchar();
	//getchar();

	////Errors = 0;
	//
	//Fixed_uDecOut2s(12345, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(12345);

	//Fixed_uDecOut2s(22100, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(22100);

	//Fixed_uDecOut2s(102, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(102);

	//Fixed_uDecOut2s(31, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(31);

	//Fixed_uDecOut2s(99999, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(99999);

	//Fixed_uDecOut2s(100000, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(100000);

	//Fixed_uDecOut2s(000000, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_uDecOut2(0);
	//getchar();

	//printf("\n\n");

///////////////
//	
//	Fixed_sDecOut3s(1234, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(1234);
//
//	Fixed_sDecOut3s(2210, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(2210);
//
//	Fixed_sDecOut3s(102, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(102);
//
//
//	Fixed_sDecOut3s(31, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(31);
//
//	Fixed_sDecOut3s(9999, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(9999);
//
//	Fixed_sDecOut3s(10000, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(10000);
//
//	Fixed_sDecOut3s(0, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(0);
//
//	Fixed_sDecOut3s(-1234, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-1234);
//
//	Fixed_sDecOut3s(-2210, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-2210);
//
//	Fixed_sDecOut3s(-102, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-102);
//
//	Fixed_sDecOut3s(-31, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-31);
//
//	Fixed_sDecOut3s(-9999, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-9999);
//
//	Fixed_sDecOut3s(-10000, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-10000);
//
//	Fixed_sDecOut3s(-0, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(0);
//
////////////////
//	Fixed_sDecOut3s(-9999, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-9999);
//
//	Fixed_sDecOut3s(-10000, Buffer);
//	printf("%s\t", Buffer);
//	Fixed_sDecOut3(-10000);

	//Fixed_sDecOut3s(-0, Buffer);
	//printf("%s\t", Buffer);
	//Fixed_sDecOut3(0);


	getchar();
	getchar();


//////////// The previous functions output whats expected


	
//	
//  for(i=0; i<16; i++)
//	{
//    Fixed_uBinOut8s(outTests3[i].InNumber,Buffer);

//    if(strcmp(Buffer, outTests3[i].OutBuffer))
//		{
//      Errors++;
//      AnError = i;
//    }
//  }

	
	//for(;;) {} /* wait forever */
	return 0;
}























// was for Fixed_sOutDec3s()
//int i;
	//// find out if n is in range
	//if( n > 99999 || n < -99999)
	//{
	//	n = 100000; // make it always positive so it doesn't return '-*.***'
	//}

	//// determine the sign of the number, n.
	//if(n >= 0)
	//{
	//	string[0] = ' ';
	//}
	//else
	//{
	//	string[0] = '-';
	//	n *= -1;
	//}

	//Fixed_uDecOut2s(n, &string[1]);
	//string[6] = 0; // bc everything was shifted right by 1 bc it was given &string[1]

	//swap(&string[3], &string[4]);
	//swap(&string[2], &string[3]);

	//for(i = 1; string[i] != 0; i++)
	//{
	//	if(string[i] == ' ')
	//	{
	//		string[i] = '0';
	//	}
	//}
