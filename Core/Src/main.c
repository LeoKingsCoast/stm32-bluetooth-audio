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

#define DATA_BUFFER_SIZE        128
#define DUTY_CICLE_MULTIPLIER   ((TIM1->ARR)/256)

// Macros para delay de microsegundos, obtidas de: https://deepbluembedded.com/stm32-systick-timer-microseconds-delay-us-delay-function/
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

FATFS FatFs; // Definição do formato de arquivo FatFs

uint8_t rx_buffer[2*DATA_BUFFER_SIZE];
uint8_t data_to_transfer[DATA_BUFFER_SIZE + 1];
bool data_ready = false;

uint32_t currentMillis, previousMillis;
/*uint8_t Estado;*/
enum state {
  IDLE,
  ABRE_ARQUIVO_ESCRITA,
  ESPERA_RECEPCAO,
  FECHA_ARQUIVO,
  TOCA_AUDIO,
  ERRO
} Estado;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// funções de controle do LCD
void lcd_string_top(char *);
void lcd_string_bottom(char *);

// funções de controle do SD Card
int SD_Mount();
void SD_Read_Size();
void SD_error_feedback(FRESULT);

// função para reprodução do áudio
void tocaAudio(FIL *file_ptr);

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

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // pino B5 conectado ao !Enable do amplificador LM4818
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  lcd_clear();
  lcd_string_top("Montando SD");
  HAL_Delay(2000);

  if(SD_Mount() == 1){
    lcd_string_top("SD Mounted!");
    HAL_Delay(2000);

    //SD_Read_Size();

    Estado = IDLE;
    lcd_clear();
    lcd_string_top("IDLE");
  }
  else{
    lcd_string_top("SD Not Found    ");
    Estado = ERRO;
  }

  FIL fil; // Variável para armazenar os arquivos
  FRESULT fres; // Variável para armazenar feedback dos comandos para o cartão SD

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    switch (Estado) {
      case IDLE:
      case ERRO:
        break;

      case ESPERA_RECEPCAO:
        if(data_ready){
          // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED 13
          f_puts((const char*)data_to_transfer, &fil);
          memset(data_to_transfer, '\0', DATA_BUFFER_SIZE + 1);
          data_ready = false;
        }
        break;

      case ABRE_ARQUIVO_ESCRITA:
        // Criando o arquivo que será usado. Se o arquivo já existe, será sobreescrito.
        fres = f_open(&fil, "Audio_teste.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
        lcd_clear();
        if(fres != FR_OK){
          SD_error_feedback(fres);
          Estado = ERRO;
          break;
        }
        lcd_string_top("File open!");
        HAL_Delay(2000);

        HAL_UART_Receive_DMA(&huart1, rx_buffer, 2*DATA_BUFFER_SIZE);
        lcd_clear();
        lcd_string_top("Esperando...");
        Estado = ESPERA_RECEPCAO;
        break;

      case TOCA_AUDIO:
        // Abrindo o arquivo de áudio
        fres = f_open(&fil, "Audio_teste.txt", FA_READ);
        lcd_clear();
        if (fres != FR_OK) {
          SD_error_feedback(fres);
          Estado = ERRO;
          break;
        }
        lcd_string_top("File open!");
        HAL_Delay(2000);

        // tocando o áudio
        HAL_TIM_Base_Start_IT(&htim1);
        tocaAudio(&fil);
        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_Delay(1000);

        Estado = FECHA_ARQUIVO;
        break;

      // O estado FECHA_ARQUIVO poderia ser uma função. Mas ao escrever, achei melhor fazer um estado para 
      // melhor integrar com a interrupção do botão 11 ao fechar o arquivo após receber o áudio por bluetooth
      case FECHA_ARQUIVO:
        lcd_clear();
        lcd_string_top("Closing file...");
        HAL_Delay(2000);
        f_close(&fil);

        lcd_clear();
        lcd_string_top("IDLE");

        Estado = IDLE;
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

// ======================= Interrupções =======================

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
    memcpy(data_to_transfer, rx_buffer, DATA_BUFFER_SIZE);
    memset(rx_buffer, '\0', DATA_BUFFER_SIZE);
    data_ready = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    /*UNUSED(huart);*/
    memcpy(data_to_transfer, rx_buffer + DATA_BUFFER_SIZE, DATA_BUFFER_SIZE);
    memset(rx_buffer + DATA_BUFFER_SIZE, '\0', DATA_BUFFER_SIZE);
    data_ready = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    currentMillis = HAL_GetTick();

    if(GPIO_Pin == GPIO_PIN_10 && (currentMillis - previousMillis > 1000)){
        if(Estado == IDLE){
            Estado = TOCA_AUDIO;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED 13
}

// ============================================================

// ============== Funções de Controle do LCD ==================

void lcd_string_top(char * str){
  lcd_put_cur(0, 0);
  lcd_send_string(str);
}

void lcd_string_bottom(char * str){
  lcd_put_cur(1, 0);
  lcd_send_string(str);
}

// ============================================================

// ============= Funções de controle do SD Card ==============
int SD_Mount(){
  FRESULT fres; // Variável para armazenar feedback dos comandos para o cartão SD

  // Montando o cartão SD
  fres = f_mount(&FatFs, "", 1);
  if(fres != FR_OK){
    return 0;
  }

  return 1;
}

void SD_Read_Size(){
    // Obtendo espaço no disco
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;
    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);

    char lcd_buff[100];

    // Escrevendo espaço no lcd
    lcd_clear();
    lcd_string_top("Total: ");
    sprintf(lcd_buff, "%lu", totalSpace);
    lcd_send_string(lcd_buff);
    lcd_string_bottom("Free: ");
    sprintf(lcd_buff, "%lu", freeSpace);
    lcd_send_string(lcd_buff);
    HAL_Delay(2000);
}

void SD_error_feedback(FRESULT fres){
  char lcd_buff[100];

  lcd_string_top("Error in file");
  lcd_string_bottom("handling.Code:");
  sprintf(lcd_buff, "%d", fres);
  lcd_send_string(lcd_buff);
}

// ============================================================
//
void tocaAudio(FIL *file_ptr){
  DWORD file_size;
  file_size = f_size(file_ptr);
  char byte_read;
  char *sd_read_buffer = &byte_read;

  lcd_string_top("Playing...");
  for (int i = 0; i < file_size; i++) {
    f_gets(sd_read_buffer, 2, file_ptr); // lê um byte do arquivo no SD card
    TIM1->CCR1 = DUTY_CICLE_MULTIPLIER * ((uint32_t) byte_read); // Escreve o byte no registrador do duty cicle. Áudio de 22050Hz => T = 45.35us
    DELAY_US(39); // Delay de aproximadamente 5 us da leitura do cartão SD
  }
  TIM1->CCR1 = 0; // Interrompe o PWM
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
