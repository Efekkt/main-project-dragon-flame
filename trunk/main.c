/////////////////////////////////////////////////////////	
//      "Main Code"                                    //
//		This code was created by Dragon Flame Team     //
//		19/09/2014                                     //
/////////////////////////////////////////////////////////

#include "derivative.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#define Labview
//#define differential_motors

extern volatile uint16_t BatSenseADC_Value;
volatile signed short Derived [128];
int16_t Minimum, Minimum_LineScan_Left, Minimum_LineScan_Right;
int16_t Maximum;
uint8_t right_edge, line_detected_index_right;
uint8_t left_edge, line_detected_index_left;
uint8_t setpoint_left, setpoint_left_linescan;
uint8_t setpoint_right,setpoint_right_linescan;
int16_t Error;
float m; 
float m_norm;
float Error_M;
const uint8_t MAXIMUM_TESTED_ERROR = 42;
const float Kp = 1;

void Battery_Level(void)
{
	//Lecturing of the charge of the Battery and displaying on the LED's
		
		if((float)BatSenseADC_Value < 1100 && TFC_Ticker[0] >= 500)
		{
				TFC_Ticker[0] = 0;
				TFC_SetBatteryLED_Level(1); //Battery 10% or less
		}
		if((float)BatSenseADC_Value<=1325 && (float)BatSenseADC_Value>1100)
		{
			TFC_SetBatteryLED_Level(1); //Battery 25% or less
		}
		if((float)BatSenseADC_Value<=1550 && (float)BatSenseADC_Value>1325)
		{
			TFC_SetBatteryLED_Level(2); //Battery 50% or less
		}
		if((float)BatSenseADC_Value<=1775 && (float)BatSenseADC_Value>1550)
		{
			TFC_SetBatteryLED_Level(3); //Battery 75% or less
		}
		if((float)BatSenseADC_Value<=2000 && (float)BatSenseADC_Value>1775)
		{
			TFC_SetBatteryLED_Level(4); //Battery 100% or less
		}	
}
void Derived_Signal(void)
{								///////////////////////////////////////////////////
	uint32_t i=0;				//Deriving the signal using the next expression: //
								//Central Difference   F[x]= (F[x+1]-F[x-1])/2   //
	for(i=0; i<128; i++)		///////////////////////////////////////////////////
	{
		Derived[i] = (LineScanImage0[i +1] - LineScanImage0[i - 1])/2;
	}
}

void Minimum_Value(uint32_t min, uint32_t max)
{
	 uint32_t i;
	 Minimum = 20000;				    ////////////////////////////////////////////////////
	 	 	 	 	 	 	 	 	 	////////////////////////////////////////////////////
	 for(i=min; i<max; i++)				// Calculating the Minimum value and index from   //
	 {									// Derived array.We despise some pixels for data  //
		if(Derived[i]<Minimum)			// Processing of the signal w/ min and max.		  //
		{								////////////////////////////////////////////////////
			Minimum=Derived[i];
			right_edge=i;
		}
	 }
}
void Minimum_Value_LineScan_Left(uint32_t min, uint32_t max)
{
	 uint32_t i;
	 Minimum_LineScan_Left = 20000;				    ////////////////////////////////////////////////////
	 	 	 	 	 	 	 	 	 	////////////////////////////////////////////////////
	 for(i=min; i<max; i++)				// Calculating the Minimum value and index from   //
	 {									// Derived array.We despise some pixels for data  //
		if(LineScanImage0[i]<Minimum_LineScan_Left)			// Processing of the signal w/ min and max.		  //
		{								////////////////////////////////////////////////////
			Minimum_LineScan_Left=LineScanImage0[i];
			line_detected_index_left=i;
		}
	 }
}
void Minimum_Value_LineScan_Right(uint32_t min, uint32_t max)
{
	 uint32_t i;
	 Minimum_LineScan_Right = 20000;				    ////////////////////////////////////////////////////
	 	 	 	 	 	 	 	 	 	////////////////////////////////////////////////////
	 for(i=min; i<max; i++)				// Calculating the Minimum value and index from   //
	 {									// Derived array.We despise some pixels for data  //
		if(LineScanImage0[i]<Minimum_LineScan_Right)			// Processing of the signal w/ min and max.		  //
		{								////////////////////////////////////////////////////
			Minimum_LineScan_Right=LineScanImage0[i];
			line_detected_index_right=i;
		}
	 }
}
void Maximum_Value(uint32_t min, uint32_t max)
{
	 uint32_t i;
	 Maximum = 0;				////////////////////////////////////////////////////
	 	 	 	 	 	 	 	 	 	////////////////////////////////////////////////////
	 for(i=min; i<max; i++)				// Calculating the Maximum value and index from   //
	 {									// Derived array. We despise some pixels for Data //			
		if(Derived[i]>Maximum)			// Processing of the signal w/ min and max.	      //
		{								////////////////////////////////////////////////////
			Maximum=Derived[i];
			left_edge=i;
		}
	 }
}

int main(void)
{	
	uint32_t i=0;
	TFC_Init();
	
	for(;;)
	{	    
		//TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
			TFC_Task();					///////////////////////////////////////
			Battery_Level();			//									 //
			Derived_Signal();			//	     Initializing Functions      //
			Minimum_Value(10,115);      //									 //	
			Maximum_Value(10,115);		///////////////////////////////////////
			Minimum_Value_LineScan_Left(0,64);
			Minimum_Value_LineScan_Right(64,128);
			
		//Setting Setpoint and creating Error function.
		switch((TFC_GetDIP_Switch()>>1)&0x03)
		{
			default:
			case 0 :
				if(TFC_PUSH_BUTTON_0_PRESSED)
				{
				setpoint_left = left_edge;
				setpoint_right = right_edge;
				}
			break;
			
			case 1 :

				if(Maximum > (-1)*Minimum)
				{
					Error = setpoint_left - left_edge;
				}
				else
				{
					Error = setpoint_right - right_edge; 
				}
				
				//Control Process m is the signal control
				m=Kp*Error;
                                                                       //////////////////////////////////////////////////////
				//Normalizing signal control to put it in the servo    // The number that is dividing is the number tested // 
				 m_norm = (float)m/MAXIMUM_TESTED_ERROR; 		       // of the maximum Error 						       //
				 	 	 	 	 	 	 	 	 	 	 	 	 	   //////////////////////////////////////////////////////
				// Setting maximum rotate 
				 if(m_norm < -0.666)
					 {
						 m_norm = -0.666;
					 }
					 if(m_norm > 0.547)
					 {
						 m_norm = 0.547;
					 }
					 
				 //Sending Values to servo
					 TFC_SetServo(0,(-1)*m_norm);
				
				//Implementing some control to DC motors
#ifdef differential_motors
	
				 	TFC_HBRIDGE_ENABLE;
				 	
				 	if(Error < - 12)
				 	{
				 		TFC_SetMotorPWM(-0.4,0.9);
				 	}
				 	else if(Error > 12 )
				 	{
				 		TFC_SetMotorPWM(0.9,-0.4);
				 	}
				 	else if(Error <= 12 && Error >= -12)
				 	{
				 		TFC_SetMotorPWM(0.4,0.4);
				 	}
#endif
#ifdef Labview	
				
				//Printing Data Cam
				if(TFC_Ticker[0]>100 && LineScanImageReady==1)
				{
					TFC_Ticker[0] = 0;
				    LineScanImageReady=0;
				    TERMINAL_PRINTF("\r\n");
				    TERMINAL_PRINTF("L:");
				    
				 for(i=0;i<128;i++)
				   {
				    TERMINAL_PRINTF("%d,",LineScanImage0[i]);
				   }
				    						
				 for(i=0;i<128;i++)
				   {
					TERMINAL_PRINTF("%d",Derived[i]); 
				    if(i==127)
				    TERMINAL_PRINTF("\r\n",Derived[i]);
				    else
				    TERMINAL_PRINTF(",",Derived[i]);
				    }										
				    							
				  }
#endif
#ifndef differential_motors
				//Putting the motors on
					if(TFC_PUSH_BUTTON_0_PRESSED)
					{
						TFC_HBRIDGE_ENABLE;
					}
					if(TFC_PUSH_BUTTON_1_PRESSED)
					{
						TFC_HBRIDGE_DISABLE;
					}
					TFC_SetMotorPWM(TFC_ReadPot(1),TFC_ReadPot(1));
#endif
			break;
			
			case 2 :
					if(TFC_PUSH_BUTTON_0_PRESSED)
					{
						setpoint_left_linescan = line_detected_index_left;
						setpoint_right_linescan = line_detected_index_right;
					}
			break;
			case 3 :
				
				if(Minimum_LineScan_Left > Minimum_LineScan_Right)
				{
					Error = setpoint_left_linescan - line_detected_index_left;
				}
				else
				{
					Error = setpoint_right_linescan - line_detected_index_right; 
				}
								
			    //Control Process m is the signal control
				  m=Kp*Error;
				                                                                       //////////////////////////////////////////////////////
				//Normalizing signal control to put it in the servo    // The number that is dividing is the number tested // 
				  m_norm = (float)m/MAXIMUM_TESTED_ERROR; 		       // of the maximum Error 						       //
								 	 	 	 	 	 	 	 	 	 	 	 	 	   //////////////////////////////////////////////////////
				// Setting maximum rotate 
				if(m_norm < -0.666)
				{
				  m_norm = -0.666;
				}
			    if(m_norm > 0.547)
				{
				  m_norm = 0.547;
				}
									 
				//Sending Values to servo
				TFC_SetServo(0,(-1)*m_norm);
								
				//Implementing some control to DC motors
#ifdef differential_motors
					
				TFC_HBRIDGE_ENABLE;
								 	
				if(Error < - 12)
				{
					TFC_SetMotorPWM(-0.4,0.9);
				}
			    else if(Error > 12 )
				{
					TFC_SetMotorPWM(0.9,-0.4);
				}
				else if(Error <= 12 && Error >= -12)
				{
					TFC_SetMotorPWM(0.4,0.4);
				}
#endif
#ifdef Labview	
								
				//Printing Data Cam
				if(TFC_Ticker[0]>100 && LineScanImageReady==1)
				{
					TFC_Ticker[0] = 0;
					LineScanImageReady=0;
					TERMINAL_PRINTF("\r\n");
					TERMINAL_PRINTF("L:");
								    
				for(i=0;i<128;i++)
				{
					 TERMINAL_PRINTF("%d,",LineScanImage0[i]);
				}
								    						
				for(i=0;i<128;i++)
				{
					TERMINAL_PRINTF("%d",Derived[i]); 
					if(i==127)
					TERMINAL_PRINTF("\r\n",Derived[i]);
				else
					TERMINAL_PRINTF(",",Derived[i]);
				}										
								    							
			    }
#endif
#ifndef differential_motors
				//Putting the motors on
				if(TFC_PUSH_BUTTON_0_PRESSED)
				{
					TFC_HBRIDGE_ENABLE;
				}
				if(TFC_PUSH_BUTTON_1_PRESSED)
				{
					TFC_HBRIDGE_DISABLE;
				}
					TFC_SetMotorPWM(TFC_ReadPot(1),TFC_ReadPot(1));
#endif
			break;
		}
	}		
	return 0;
}
