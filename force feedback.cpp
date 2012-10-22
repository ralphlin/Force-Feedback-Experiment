/*********************************************************************
*
* force feedback.cpp
*
* Written January, 2009
* By Ralph Lin
* University of Washington
* Neurobotics Lab
*
* Description:
*	This program is used to collect data for a human-in-the-loop
*   robotics experiment to characterize neural reflexes while
*   gripping an object. Briefly, the subject holds a custom
*   machined aluminum object attached to the end of a SensAble
*   Technologies Phantom 1.5 Haptic Robot. The subject's grip force
*   is measured with a fsr force sensor, while the subject's 
*   electrical muscle signals are recorded via EMG electrodes 
*   attached to the skin. 
*
*   Over the duration of the experiment, the robot will apply forces
*   to the gripped object attempting to "jerk" the object from the
*   subject's grasp while recording the subjects' grip and muscle 
*   response. These data were used to create artificial robotics
*   reflexes to help advanced prosthetics integrate with brain
*   computer interfaces for complex object manipulation.
*
* Notes:
*	This program was designed to run in Windows (WAIMEA) with the 
*   SensAble Phantom 1.5 robot, and NiDAQ cards to read force sensor 
*   and EMG data.
*
*********************************************************************/


//#include "stdafx.h"
#include <windows.h>

#include <conio.h>
#include "NIDAQmx.h"

#include <fstream>
#include <string>
#include <iostream>

#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include "stdlib.h"
#include <cstdlib>
#include <ctime>

using namespace std;

// TRIALS is number of trials desired
// INTERVAL is the length of each individual trial in seconds
// INTERVAL_BETWEEN_TRIALS seconds
// FORCE_DURATION is length of force application in seconds
// INITIAL_REST is period of time in seconds where EMG is recorded before
// trials begin
// FORCE_THRESHOLD is voltage threshold for biofeedback (max force
// that subject should hold the object) and should be equivalent to ~0.6 N

#define TRIALS 20
#define INTERVAL 5.0
#define INTERVAL_BETWEEN_TRIALS 2.0
#define FORCE_DURATION 0.2500
#define INITIAL_REST 10.0
#define FORCE_THRESHOLD 0.0349

/* FORCE_THRESHOLD changed from 1.854 when switching to new battery
   FORCE_THRESHOLD changed from 1.888 when switching to new circuit
   Equivalent to ~ 0.6 N

#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

/* Handle to haptic device. */
HHD ghHD = HD_INVALID_HANDLE;

/* Handle to haptic rendering context. */
HHLRC ghHLRC;

/* Effect ID */
HLuint gEffect;

/* Effect properties */
float gGain = .2f;
float gMagnitude = .5f;

/* Initialize Haptics */
void initHL(void)
{
    HDErrorInfo error;

    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        exit(-1);    
    }   
    
    ghHLRC = hlCreateContext(ghHD);
    hlMakeCurrent(ghHLRC);

    gEffect = hlGenEffects(1);

}

/* Haptic force update */
void updateEffect()
{
    hlBeginFrame();
    HLboolean bActive = false;
    hlGetEffectbv(gEffect, HL_EFFECT_PROPERTY_ACTIVE, &bActive);
    if (bActive)
    {
        hlEffectd(HL_EFFECT_PROPERTY_GAIN, gGain);
        hlEffectd(HL_EFFECT_PROPERTY_MAGNITUDE, gMagnitude);    
        hlUpdateEffect(gEffect);
    }
    else
    {
        printf("No effect active\n");
    }
    hlEndFrame();
}

/* Stop haptic effect */
void stopEffect()
{
    hlBeginFrame();
    HLboolean bActive = false;
    hlGetEffectbv(gEffect, HL_EFFECT_PROPERTY_ACTIVE, &bActive);
    if (bActive)
    {
        hlStopEffect(gEffect);
    }
    hlEndFrame();    
}

/* Start haptic effect type */
void startEffectType(HLenum newEffectType)
{
    stopEffect();
    
    hlBeginFrame();
    if (newEffectType == HL_EFFECT_SPRING)
    {
        hduVector3Dd position;
        hlGetDoublev(HL_DEVICE_POSITION,position);
        hlEffectdv(HL_EFFECT_PROPERTY_POSITION, position);
    }
    else if (newEffectType == HL_EFFECT_CONSTANT)
    {
        hlEffectd(HL_EFFECT_PROPERTY_DURATION, 100);
        hlEffectdv(HL_EFFECT_PROPERTY_DIRECTION, hduVector3Dd(0,1,0));
    }

    hlStartEffect(newEffectType, gEffect);
    hlEndFrame();
    updateEffect();
}

/* Main function */
int main(int argc, char *argv[])
{
	int32			error=0;
	TaskHandle		taskHandle=0;
	int32			totalRead=0;
	float64			data[6];
	float			forceData[50];
	char			errBuff[2048]={'\0'};
	char			data_string[200];
	float			forceBuffer[25];
	LARGE_INTEGER	ticksPerSecond;
	LARGE_INTEGER	tick;				// A point in time
	
	// get the high resolution counter's accuracy
	QueryPerformanceFrequency(&ticksPerSecond);

	double			period=0.001;		// in seconds
	double			q=0;				// time elapsed since beginning (sec)
	double			ticksElapsed;
	double			freq;
	int				rate=0;
	double			interval=0;			// interval between samples
	LARGE_INTEGER	prevtick;
	LARGE_INTEGER	newtick;
	bool			flag=true;
	int				count = 0;
	int32			readData;
	float			forceTotal;
	hduVector3Dd	position;
	int				windowCount = 0;
	int				windowSize = 50;
	double			sumForce = 0;
	double			tempCount = 0;		// first force data in an array of 50 data points
	double			forceAvg = 0;

	freq=(double)ticksPerSecond.QuadPart;

	/* Configure NiDAQ and init task */
	DAQmxCreateTask("",&taskHandle);
	DAQmxCreateAIVoltageChan(taskHandle,"Dev2/ai7,Dev2/ai0,Dev2/ai3,Dev2/ai4,Dev2/ai1,Dev2/ai6","",DAQmx_Val_Cfg_Default,-10.0,10.0,DAQmx_Val_Volts,NULL);
	DAQmxStartTask(taskHandle);

	/* Initialize Phantom robot */
	initHL();
	
	/* Open file streams */
	ofstream mydata;
	ofstream mytiming;
	mydata.open("c:/Users/Ralph/force feedback/volEMGData.txt", ios::trunc);
	mytiming.open("c:/Users/Ralph/force feedback/voltiming.txt", ios::trunc);
	
	/*********************************************/
	// create trial timing and random forces with 
	// TRIALS number of trials
	// trial duration is INTERVAL in seconds
	/*********************************************/

	float		random;
	float		startTimes[TRIALS];
	float		endTimes[TRIALS];
	float		forceLevel[TRIALS];
	int			i;

	srand((unsigned)time(0));

	for (i=0;i<TRIALS;i++)
	{
		random = ( rand() % 10000 + 1 )/10000.0;
		startTimes[i]=INITIAL_REST + i*INTERVAL + i*INTERVAL_BETWEEN_TRIALS + random*INTERVAL;
		endTimes[i]=startTimes[i]+FORCE_DURATION;
		forceLevel[i] = 0.15f; // ( rand() % 4 + 1 )/4.0 * 0.4f;
		sprintf(data_string,"%f \t %f \t %f \n",startTimes[i], endTimes[i], forceLevel[i]);
		mytiming<<data_string;
	}

	int			currentTrial=1;
	bool		forceOn=false;
	bool		experimentFinished=false;
	long		samples=0;

	printf("Ready to begin experiment. Press Enter to begin\n");
	getchar();

	gMagnitude=0.09f;
	startEffectType(HL_EFFECT_CONSTANT);

	while( !_kbhit() ) {
		
		QueryPerformanceCounter(&tick);		
		ticksElapsed=(double)(tick.QuadPart%ticksPerSecond.QuadPart); //the number of counts since last cycle
		rate=(int)(100000*ticksElapsed/freq); // the number of tens of micro seconds since last second
		
		if(flag==true){
		QueryPerformanceCounter(&newtick);
		flag=false;
		}		

		if(rate%(int)(period*100000) < 10){
			QueryPerformanceCounter(&prevtick);
			flag=true;			
			interval=(double)(prevtick.QuadPart-newtick.QuadPart)/freq;			

			if (count != 0) {
				q+=interval; //period;
				
				/* Read force and EMG data from NiDAQ */
				DAQmxReadAnalogF64(taskHandle,-1,-1,DAQmx_Val_GroupByChannel, data, 6, &readData, NULL);
				
				//windowSize, sumForce, tempCount, forceAvg
				samples++;				
				forceData[windowCount] = data[1];
				if (samples  == windowSize) {			// finds the sum of the first 50 data points only once		
					tempCount = forceData[0];
					for (int i = 0; i <= windowSize-1; i++) {
						sumForce = sumForce + forceData[i];
					}
				}
				if (samples > windowSize) {			// subtracts the first data point from sumForce and adds the current force data
					int temp = windowCount+1;
					if (temp >= 50) {
						temp = 0;
					}
					tempCount = forceData[temp];	
					sumForce = sumForce - tempCount + data[1];
				}
				if (samples%windowSize == 0) {			// resets the index that force data is saved to
					windowCount = 0;
				}
				else {
					windowCount++;
				}
				forceAvg = sumForce/windowSize;			

				/* get current position of phantom */
				hdGetDoublev(HD_CURRENT_POSITION,position);

				/* output data to file */
				sprintf(data_string,"%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \n", q, data[0], data[1], data[2], data[3], data[4], data[5], position, forceAvg, tempCount);
				mydata<<data_string;

				if (samples <=25) {
					forceBuffer[samples-1]=data[1];
				}
				else {
					forceBuffer[samples%25]=data[1];
					forceTotal=0;
					for (int j=0;j<25;j++) {
						forceTotal=forceTotal+forceBuffer[j];
					}
					if (experimentFinished) {
						printf("Experiment is done!\n");
						break;
					}
					else {  // instructs subject to modulate grip within band
						if ( (forceTotal/25.0) < (FORCE_THRESHOLD-0.004) ) {
							printf("+++\n");
						}
						else {
							if ( (forceTotal/25.0) < (FORCE_THRESHOLD+0.004) ) {
								printf("-\n");
							}
							else {
								printf("---\n");
							}
						}
					}
				}
		
				/* Applies haptic force, but modulate by scaled forceAvg */
				if ( ((q >= startTimes[currentTrial-1])&&(!forceOn)) && (!experimentFinished) ) {
					
					// if forceAvg is between FORCE_THRESHOLD and 1.80, then modify force
					// if below 1.80V, then nullify upward effect gMagnitude = 0.09f

					if (forceAvg < 0.0349) {
						gMagnitude=forceLevel[currentTrial-1];
					} 
					else if ((forceAvg > 0.0349) && (forceAvg < 0.037)) {
						gMagnitude=forceLevel[currentTrial-1]-(0.13/(-0.0349+0.037))*(-0.0349+forceAvg); // 0.18 - scaling factor*force component, must remain positive! 
					}
					else if (forceAvg > 0.037
						) {
						gMagnitude=0.05f;
					}
					updateEffect();
					forceOn=true;
				}
				if ((q >= endTimes[currentTrial-1])&&(forceOn)) {

					gMagnitude=0.09f;
					updateEffect();
					forceOn=false;
					currentTrial++;
					if (currentTrial == TRIALS + 1) {
						experimentFinished=true;
					}
				} 

				/* Check for HL errors */
				HLerror error;
				while (HL_ERROR(error = hlGetError()))
				{
					fprintf(stderr, "HL Error: %s\n", error.errorCode);
                
					if (error.errorCode == HL_DEVICE_ERROR)
					{
						hduPrintError(stderr, &error.errorInfo,
                                  "Error during haptic rendering\n");
					}
				}

			}
			count = 1;
		}
	}
	/* close data streams */
	mydata.close();
	mytiming.close();
	_getch();

	
	// Release the effect id.
    hlDeleteEffects(gEffect, 1);

    // Cleanup.
	DAQmxStopTask(taskHandle);
	DAQmxClearTask(taskHandle);
	hlMakeCurrent(NULL);
    hlDeleteContext(ghHLRC);
    hdDisableDevice(ghHD); 

	printf("End of program, press Enter key to quit\n");
	getchar();

	return 0;
}



