main(){
/* USER CODE BEGIN PV */

int i, _Dir, SW1Flag, SW2Flag, gCount;

/* USER CODE END PV */

  while (1)  {

	  printf("[gClount:%d] Switch Flag: SW1: %d /  SW2: %d \n", gCount, SW1Flag, SW2Flag);
	  count=0x01;
	  for(i=0;i<8;i++){
		  LED_Stick(count<<i);
		  HAL_Delay(100);
	  }

	  count=0x80;
	  for(i=0;i<8;i++){
		  LED_Stick(count>>i);
		  HAL_Delay(100);
	  }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	gCount++;
	if(GPIO_Pin == SW1_Pin){
		SW1Flag = 1;
		SW2Flag = 0;
	}else if(GPIO_Pin == SW2_Pin){
		SW1Flag = 0;
		SW2Flag = 1;
	}

}
