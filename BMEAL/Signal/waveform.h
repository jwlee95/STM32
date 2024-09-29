/*
 * @file waveform.h
 *
 * Created on: Nov 9, 2023
 * @author   Ivan Lee (KKU-BIMSRL)
 *
 */


#ifndef WAVEFORM_H_
#define WAVEFORM_H_

#include <stdlib.h>   // to generates pseudo-random numbers

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif  // TWO_PI

/**
 * @brief Class to Sin waveform generation
 *
 */
typedef struct{
  float a1, b0, yn0, yn1, yn2;
  float a1_1, b0_1, yn0_1, yn1_1, yn2_1;
  float F0, F0_1;
  float T0; 		// sampling period. ex. 1.0/100.0;
} WaveformTypeDef;


/* Function prototypes -----------------------------------------------*/

void Wave_Init(WaveformTypeDef *wave,float Freq, float sr );
float Wave_Forward(WaveformTypeDef *wave);
void Wave_Reset(WaveformTypeDef *wave );
float PseudoNoise(void);


#endif /* WAVEFORM_H_ */
