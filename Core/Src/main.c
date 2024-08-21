/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IDLE                    0
#define ABRE_ARQUIVO_ESCRITA    1
#define ESPERA_RECEPCAO         2
#define FECHA_ARQUIVO           3
#define ABRE_ARQUIVO_LEITURA    4
#define TOCA_AUDIO              5

// Macros for us delay found on: https://deepbluembedded.com/stm32-systick-timer-microseconds-delay-us-delay-function/
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
 
#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_buffer[256];
uint8_t data_to_transfer[129];
bool data_ready = false;

uint8_t tx_buffer[256]; // For debugging

uint32_t currentMillis, previousMillis;
uint8_t Estado;

char byte_read;
char *sd_read_buffer = &byte_read;

FATFS       FatFs;                //Fatfs handle
FIL         fil;                  //File handle
FRESULT     fres;                 //Result after operations
UINT        WWC;
char        buf[5];

char lcd_buff[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  lcd_init(); 

  HAL_Delay(2000);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  lcd_clear();
  lcd_put_cur(0,0);
  lcd_send_string("Preparando SD");
  HAL_Delay(2000);
  lcd_put_cur(0,0);

  do {
    // Montando o cartão SD
    fres = f_mount(&FatFs, "", 1);
    if(fres != FR_OK){
      lcd_send_string("SD Not Found");
      Estado = IDLE;
      break;
    }
    lcd_send_string("SD Mounted!");
    HAL_Delay(2000);

    // Obtendo espaço no disco
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;
    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
    lcd_clear();
    lcd_put_cur(0,0);
    lcd_send_string("Total: ");
    sprintf(lcd_buff, "%lu", totalSpace);
    lcd_send_string(lcd_buff);
    lcd_put_cur(1,0);
    lcd_send_string("Free: ");
    sprintf(lcd_buff, "%lu", freeSpace);
    lcd_send_string(lcd_buff);
    HAL_Delay(2000);

    lcd_clear();
    lcd_put_cur(0,0);
    lcd_send_string("IDLE");
    /*lcd_clear();*/
    /*lcd_put_cur(0,0);*/
    /*lcd_send_string("Data read:");*/
    /*f_gets(buf, sizeof(buf), &fil);*/
    /*lcd_put_cur(1,0);*/
    /*lcd_send_string(buf);*/
    /*HAL_Delay(5000);*/


  } while (false);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  Estado = IDLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    switch (Estado) {
        case IDLE:
            break;

        case ESPERA_RECEPCAO:
            if(data_ready){
                //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                /*strcpy((char*)tx_buffer, "\n\rEscrita no SD:\n\r");*/
                /*HAL_UART_Transmit(&huart2, tx_buffer, strlen((const char*)tx_buffer), 100);*/
                /*HAL_UART_Transmit(&huart2, data_to_transfer, strlen((const char*)data_to_transfer), 100);*/
                f_puts((const char*)data_to_transfer, &fil);
                memset(data_to_transfer, '\0', 129);
                //lcd_send_string((char*)rx_buffer);
                data_ready = false;
                /*HAL_UART_Receive_DMA(&huart1, rx_buffer, 256);*/
            }
            break;

        case ABRE_ARQUIVO_ESCRITA:
            // Criando o arquivo que será usado. Se o arquivo já existe, será sobreescrito.
            fres = f_open(&fil, "Audio_teste.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
            lcd_clear();
            lcd_put_cur(0,0);
            if(fres != FR_OK){
                lcd_send_string("File not created");
                lcd_put_cur(1,0);
                lcd_send_string("Error ");
                sprintf(lcd_buff, "%d", fres);
                lcd_send_string(lcd_buff);
                Estado = IDLE;
                break;
            }
            lcd_send_string("File open!");
            HAL_Delay(2000);

            lcd_clear();
            lcd_put_cur(0,0);
            lcd_send_string("Esperando...");
            lcd_put_cur(1,0);
            Estado = ESPERA_RECEPCAO;

            HAL_UART_Receive_DMA(&huart1, rx_buffer, 256);
            break;

        case ABRE_ARQUIVO_LEITURA:
            // Abrindo o arquivo de audio
            fres = f_open(&fil, "Audio_teste.txt", FA_READ);
            if (fres != FR_OK) {
                lcd_clear();
                lcd_put_cur(0,0);
                lcd_send_string("File not read");
                lcd_put_cur(1,0);
                lcd_send_string("Error ");
                sprintf(lcd_buff, "%d", fres);
                lcd_send_string(lcd_buff);
                Estado = IDLE;
                break;
            }
            /*fres = f_open(&fil, "Audio.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);*/
            /*lcd_clear();*/
            /*lcd_put_cur(0,0);*/
            /*if(fres != FR_OK){*/
            /*    lcd_send_string("File not created");*/
            /*    lcd_put_cur(1,0);*/
            /*    lcd_send_string("Error ");*/
            /*    sprintf(err, "%d", fres);*/
            /*    lcd_send_string(err);*/
            /*    Estado = IDLE;*/
            /*    break;*/
            /*}*/
            lcd_clear();
            lcd_put_cur(0,0);
            lcd_send_string("File open!");
            HAL_Delay(2000);

            lcd_clear();
            lcd_put_cur(0,0);
            lcd_send_string("Data read:");
            lcd_put_cur(1, 0);
            for (int i = 0; i < 29999; i++) {
                /*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);*/
                f_gets(sd_read_buffer, 2, &fil); 
                /*HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);*/
                // 22050Hz Audio => T = 45.35us
                TIM1->CCR1 = (uint32_t) byte_read;
                DELAY_US(39);

                //sprintf(err, "%lu", (uint32_t) byte_read);
                //lcd_send_string(err);
                //HAL_Delay(500);
            }
            TIM1->CCR1 = 0; 
            /*f_gets(buf, sizeof(buf), &fil);*/
            /*lcd_put_cur(1,0);*/
            /*lcd_send_string(buf);*/
            /*HAL_Delay(1000);*/
            /*f_gets(buf, sizeof(buf), &fil);*/
            /*lcd_send_string(buf);*/
            HAL_Delay(1000);

            Estado = FECHA_ARQUIVO;
            break;

        case FECHA_ARQUIVO:
            lcd_clear();
            lcd_put_cur(0,0);
            lcd_send_string("Closing file...");
            HAL_Delay(2000);
            f_close(&fil);

            lcd_clear();
            lcd_put_cur(0,0);
            lcd_send_string("IDLE");

            Estado = IDLE;
            break;

        case TOCA_AUDIO:
            break;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int HTC = 0, FTC = 0;
int indx = 0;

bool SizeRead = true;
uint32_t file_size = 0;

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
  if (!SizeRead) {
    // size reading script
    indx += 124;
    SizeRead = true;
  }
  else {
    /*memcpy(tx_buffer, rx_buffer, 256);*/
    memcpy(data_to_transfer, rx_buffer, 128);
    /*memset(tx_buffer, '\0', 256);*/
    memset(rx_buffer, '\0', 128);
    data_ready = true;
  }
  HTC = 1;
  FTC = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    /*UNUSED(huart);*/

    //f_puts((const char*)rx_buffer, &fil);
    //f_write(&fil, rx_buffer, strlen((char*)rx_buffer), &WWC);
    //lcd_send_string((char*)rx_buffer);
    /*memcpy(tx_buffer, rx_buffer, 256);*/
    /*strcpy((char*)tx_buffer, "\n\rComplete callback: \n\r");*/
    /*HAL_UART_Transmit(&huart2, tx_buffer, strlen((const char*)tx_buffer), 100);*/
    /*HAL_UART_Transmit(&huart2, rx_buffer, 256, 100);*/
    memcpy(data_to_transfer, rx_buffer + 128, 128);
    /*memset(tx_buffer, '\0', 256);*/
    memset(rx_buffer + 128, '\0', 128);
    data_ready = true;
    HTC = 0;
    FTC = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    currentMillis = HAL_GetTick();

    if(GPIO_Pin == GPIO_PIN_10 && (currentMillis - previousMillis > 1000)){
        if(Estado == IDLE){
            Estado = ABRE_ARQUIVO_LEITURA;
        }
    }
    else if(GPIO_Pin == GPIO_PIN_11 && (currentMillis - previousMillis > 1000)){
        if(Estado == IDLE){
            Estado = ABRE_ARQUIVO_ESCRITA;
        }
        else if (Estado == ESPERA_RECEPCAO){
            Estado = FECHA_ARQUIVO;
        }
    }

    previousMillis = currentMillis;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
