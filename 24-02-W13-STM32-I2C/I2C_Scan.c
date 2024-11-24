
// STM32CubeIde의 코드 부분을 제외한 순수하게 추가되는 코드.......

int main(void)
{
int i,å ret;
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    HAL_Delay(1000);

    /*-[ I2C Bus Scanning ]-*/
    printf("Starting I2C Scanning: \r\n");
    for(i=1; i<128; i++)
    {
    	ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    	if (ret != HAL_OK){ /* No ACK Received At That Address */
    		printf("No Device :[0x%2x]\n ",i );
               }else if(ret == HAL_OK) {
    		 printf("=====> Device found @[0x%2x]\n",i);
        	}
    }
    printf("\r\n=== End of Scanning!....====\n");
    /*--[ Scanning Done ]--*/

    while (1)
    {
    }
}
