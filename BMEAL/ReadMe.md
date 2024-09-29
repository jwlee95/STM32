# BME Abstraction Layer (BMEAL) Driver

- BMEAL의 폴더의 위치(새로운 폴더를 생성하고, 하위 폴더를 복사하여 넣는다)   
![](BMEAL-Folder-Location.png)

- BMEAL을 사용하는 방법은 main.c에서 다름과 같이 include한다.
 
```c     
#include "main.h"
#include "../ECUAL/LCD16x2/LCD16x2.h"
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
 
    LCD_Init();
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("It Works! GG izi");
 
    while (1)
    {
 
    }
}
```
