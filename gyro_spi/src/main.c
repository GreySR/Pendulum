#include "stdio.h"
#include "stm32f429xx.h"
#include "arm_math.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_exti.h"
#include "MedianFilter.h"

#define SIZE 700 // количество измерений
#define TX_BUFFER_SIZE 64
#define NUM_ELEMENTS 49

float32_t omega_xyz[3];
float32_t xyzw[SIZE];
float32_t xyzw_medfilt[SIZE]; // значения после фильтрации
float32_t time1[SIZE], time2[SIZE];
float32_t mean, mean1; // среднее по массиву xyzw, в котором сумма четырех омег 
float32_t ox, oy, oz, om, s, T = 0., l = 0.;
size_t pre = 199; 
size_t arr = 4599; 
size_t j = 0; 
size_t row_d = 0; // номер строки дисплея. будет менять от 1 до 10 включительно
float32_t g = 9.80665; // ускорение свободного падения
char str[TX_BUFFER_SIZE]; // массив для sprintf, для вывода длины на дисплей
uint8_t k;
uint32_t BaudRate = 115200;
float32_t tic = 0.01; // 10мс
float32_t T1 = 0., T2 = 0.;

static sMedianFilter_t medianFilter;
static sMedianNode_t medianBuffer[NUM_ELEMENTS];



static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* The voltage scaling allows optimizing the power consumption when the device is
    clocked below the maximum system frequency, to update the voltage scaling value
    regarding system frequency refer to product datasheet. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    /* Activate the Over-Drive mode */
    HAL_PWREx_EnableOverDrive();
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_CFGR_PPRE1_DIV8;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void InitializeRCC_GPIO(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);

    // TIM2
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_1);

    // LED GREEN/RED
    LL_GPIO_SetPinMode(GPIOG, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOG, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
    
    // USER BUTTON (reset)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);   

    // USART1
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7);
}

void ConfigurationUSART() { 
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART1, 22500000, LL_USART_OVERSAMPLING_16, BaudRate);    
    LL_USART_EnableDirectionRx(USART1);
    LL_USART_EnableDirectionTx(USART1);
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_Enable(USART1);
}

void ConfigurationTIM(void) {
    LL_TIM_SetPrescaler(TIM2, pre);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM2, arr);
    LL_TIM_EnableIT_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
}

void SendXYZ2PC() {
    if (j == SIZE)
        j = 0;    
    char tx_buffer[TX_BUFFER_SIZE];
    if (j % 50 == 0)
        sprintf(tx_buffer, "%f %f %f\r\n ", ox, oy, oz);
    else
        sprintf(tx_buffer, "%f %f %f ", ox, oy, oz);
       
    for (size_t i = 0; i < strlen(tx_buffer); i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1)); // Wait for the end of old data transmission
        LL_USART_TransmitData8(USART1, tx_buffer[i]);
    }
}

void period() {
    T = 0.;
    T1 = 0.;
    T2 = 0.;
    arm_mean_f32(xyzw_medfilt, SIZE, &mean);
    uint8_t f = 1;    
    uint8_t k1 = 0, k2 = 0;
    if(xyzw_medfilt[0] <= mean) {
        for(size_t i = 0; i < SIZE - 1; ++i) {
            if(xyzw_medfilt[i] > mean && f == 1) {
                time1[k1] = i;
                k1++;
                f = 0;
            }
            if(xyzw_medfilt[i] < mean && f == 0) {
                time2[k2] = i;
                k2++;
                f = 1;
            }
        }
    } else {
        for(size_t i = 0; i < SIZE - 1; ++i) {
            if(xyzw_medfilt[i] < mean && f == 1) {
                time1[k1] = i;
                k1++;
                f = 0;
            }
            if(xyzw_medfilt[i] > mean && f == 0) {
                time2[k2] = i;
                k2++;
                f = 1;
            }
        }
    }
    k = (k1 <= k2) ? k1 : k2;
    for(size_t i = 1; i < k; ++i) {
        T1 = (time1[i] - time1[i - 1]) * tic + T1;
        T2 = (time2[i] - time2[i - 1]) * tic + T2;        
    }
    T1 /= (k - 1);
    T2 /= (k - 1);
    T = (T1 + T2) / 2;    
    T += 0.05 * T;
}

void length() {
    l = 0.;
    l = (T * T * g) / (4 * PI * PI);
}

void display() {    
    LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_14);
    sprintf(str, "l=%.3f,T=%.2f", l, T);         
    BSP_LCD_DisplayStringAtLine(row_d++, (unsigned char *)str);
    if (row_d == 10) {
        row_d = 0;
        BSP_LCD_Clear(LCD_COLOR_WHITE);
    }
}

void TIM2_IRQHandler() {    
    TIM2->SR &= ~TIM_SR_UIF;
    LL_GPIO_TogglePin(GPIOG, LL_GPIO_PIN_13);
    BSP_GYRO_GetXYZ(omega_xyz);
    ox = omega_xyz[0];
    oy = omega_xyz[1];
    oz = omega_xyz[2];
    arm_sqrt_f32(ox * ox + oy * oy + oz * oz, &om);
    s = ox + oy + oz + om;
    SendXYZ2PC();     
    xyzw[j] = s;
    xyzw_medfilt[j] = MEDIANFILTER_Insert(&medianFilter, s);
    j++;
    if (j == SIZE) {        
        period();
        length();
        display();
    }
}

// Сброс
void EXTI0_IRQHandler() {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    j = 0;
    row_d = 0;
    BSP_LCD_Clear(LCD_COLOR_WHITE);
}


int main() {    
    HAL_Init();
    SystemClock_Config();
    InitializeRCC_GPIO();
    ConfigurationUSART();
    ConfigurationTIM();
    BSP_GYRO_Init();
    BSP_LCD_Init();    
    BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
    BSP_LCD_DisplayOn();
    
    BSP_LCD_Clear(LCD_COLOR_WHITE); // Clear display
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); // Choose background color
    BSP_LCD_SetTextColor(LCD_COLOR_ORANGE); // Set work color, not only for text    
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

    medianFilter.numNodes = NUM_ELEMENTS;
    medianFilter.medianBuffer = medianBuffer;
    MEDIANFILTER_Init(&medianFilter);
    
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);
    while(1) {
        
    }
}