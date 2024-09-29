/*
 * @file waveform.c
 *
 * Created on: Nov 9, 2023
 * @author   Ivan Lee (KKU-BIMSRL)
 *
 */


#include "waveform.h"
#include <math.h>   // for cos function.......

void Wave_Init(WaveformTypeDef *wave, float Freq, float sr ){
    wave->F0 = Freq;
    wave->T0 = 1.0/sr;   									// Sample interval....
    wave->a1 = 2.0* cos( TWO_PI * wave->F0 * wave->T0 );  	// Initial Frequency: 100 Hz...
    wave->b0 = sin( TWO_PI * wave->F0 * wave->T0 );
    wave->yn2 = 0.0; 										// Initial value: y[0] = 0
    wave->yn1 = wave->b0;  									// Initial value: y[1] = b0
}

float Wave_Forward(WaveformTypeDef *wave) {
	wave->yn0 = wave->a1 * wave->yn1 - wave->yn2; 	// y[n] = a1*y[n-1] - y[n-2]
	wave->yn2 = wave->yn1;        					// y[n-2] = y[n-1]
	wave->yn1 = wave->yn0;        					// y[n-1] = y[n]
    return (wave->yn0);
}


void Wave_Reset(WaveformTypeDef *wave) {
	wave->a1 = 2.0* cos( TWO_PI * wave->F0 * wave->T0 );  	// Initial Frequency: 100 Hz...
	wave->b0 = sin( TWO_PI * wave->F0 * wave->T0 );
	wave->yn2 = 0.0; 										// Initial value: y[0] = 0
	wave->yn1 = wave->b0;  									// Initial value: y[1] = b0
}

/**
* @brief Function to generates pseudo-random numbers ( 0 ~ 1.0).
*/
float PseudoNoise(void) {
  float rvalue;
  rvalue = (rand()%1000 + 20) / 1000.0;
  return(rvalue);
}



void f(){
/* USER CODE BEGIN WHILE */
  i=0;
  while (1)  {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        j= (i++)%50;
        printf("SWD printf!...%04d\n",j);
        fvalue1 = ForwardWave(&Wave1);
        fvalue2 = ForwardWave(&Wave2);
        fvalue3 = 0.3* fvalue2 + fvalue1;
        IIR4_filterBlock(&filter1,&fvalue3, &output, 1);
        // int IIR4_filterBlock( IIR4Type * pThis, float * pInput, float * pOutput, unsigned int count )

        HAL_Delay(50);
    /* USER CODE END WHILE */

}




