# Implementando este Projeto do Zero

Siga essas instruções se desejar implementar o projeto por si mesmo ou entender detalhes de como foi feito.

## Configuração Inicial

### Configuração CubeMX

#### Configuração do clock

- Ir à aba RCC e selecionar **High Speed Clock (HSE):** Crystal/Ceramic Resonator.
- Ir até Clock Configuration, selecionar HSE no primeiro multiplexador, PLLCLK no segundo e configurar a máxima frequência em HCLK (72 MHz).

#### Botões de controle

- Selecionar os pinos PB10 e PB11 como GPIO_EXTIxx.
- Ir até GPIO > NVIC.
    - Selecionar a opção "EXTI line \[15:10\] interrupts"

#### Sinal de PWM

- Em Timers, selecionar TIM1.
- Configurações TIM1:
    - **Clock Source:** Internal Clock
    - **Channel1**: PWM Generation CH1
    - **Counter Period:** 1024

#### Cartão SD

- Em Connectivity, selecionar SPI1
- Configurações SPI1:
    - **Mode:** Full-Duplex Master
    - **Prescaler:** 16 
- Em Middleware and Software Packs, selecionar FATFS
- Configurações FATFS:
    - Selecionar **User-defined** 
    - **USE-LFN (Use Long Filename)**: Enabled with static working buffer on the BSS
    - **MAX_SS:** 4096 

#### Módulo Bluetooth

- Em Connectivity, selecionar USART1.
- Configurações USART1 > Parameter Settings:
    - **Mode:** Asyncronous
    - **Baud Rate:** 9600 Bits/s 
- Configurações USART1 > NVIC Settings:
    - Selecionar **USAR1 global interrupt** 
- Configurações USART1 > DMA Settings:
    - Selecionar **Add** > **USART1_RX**
    - **Mode:** Circular 

#### Display LCD

- Em Connectivity, selecionar I2C1.
- Configurações I2C1:
    - **I2C:** I2C

#### Gerando o código

- Em Project Manager > Project:
    - Nomear o projeto
    - Escollher localização do projeto
    - **Toolchain/IDE:** Makefile

- Em Project Manager > Project:
    - Selecionar: **Copy only the necessary library files** 
    - Selecionar: **Generate peripheral initialization as a pair of '.c/.h' files per peripheral** 

### Alterações no código

#### Adicionando bibliotecas

- Baixar o diretório `libraries` desse repositório.
- Aqui, <projeto> é o diretório onde seu projeto está localizado
- Adicionar os seguintes arquivos a <projeto>/Core/Src: `fatfs_sd_card.c`, `i2c-lcd.c`
- Adicionar os seguintes arquivos a <projeto>/Core/Inc: `fatfs_sd_card.h`, `i2c-lcd.h`

- Alterar o Makefile para que encontre as bibliotecas:
    - Em C_Sources, adicionar as seguintes linhas:
```
C_Sources = \
# ...
Core/Src/fatfs_sd_card.c \
Core/Src/i2c-lcd.c \
# ...
```

#### Alterando arquivos gerados pela CubeMX para leitura do Cartão SD

- Adicionar as seguintes linhas de código em <projeto>/Core/Src/stm32f1xx_it.c NOS ESPAÇOS CORRESPONDENTES:
```c
/* USER CODE BEGIN PV */
extern uint16_t Timer1, Timer2;
/* USER CODE END PV */

...

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  if(Timer1 > 0)
        Timer1--;
  if(Timer2 > 0)
        Timer2--;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
```

- Modificar o arquivo <projeto>/FATFS/Target/user_diskio.c para que as funções fiquem da seguinte forma:
```c
/* Includes ------------------------------------------------------------------*/
...
#include <fatfs_sd_card.h>

...

DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    //Stat = STA_NOINIT;
    //return Stat;
    return SD_disk_initialize(pdrv);
  /* USER CODE END INIT */
}

...

DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    //Stat = STA_NOINIT;
    //return Stat;
    return SD_disk_status(pdrv);
  /* USER CODE END STATUS */
}

...

DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    //return RES_OK;
    return SD_disk_read(pdrv, buff, sector, count);
  /* USER CODE END READ */
}

...

DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
    //return RES_OK;
    return SD_disk_write(pdrv, buff, sector, count);
  /* USER CODE END WRITE */
}

...

DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    //DRESULT res = RES_ERROR;
    //return res;
    return SD_disk_ioctl(pdrv, cmd, buff);
  /* USER CODE END IOCTL */
}
```

## Utilização dos Módulos

- A comunicação com o módulo Bluetooth é feita através do canal USART, utilizando a função `HAL_UART_Receive_DMA()`, que habilita a interrupção do UART com uso de DMA. As rotinas de tratamento de interrupção são `HAL_UART_RxHalfCpltCallback` e `HAL_UART_RxCpltCallback`, que são ativadas respectivamente quando o buffer de leitura é preenchido pela metade ou preenchido completamente.
- A comunicação com o Cartão SD é feita através do módulo SPI, através de um driver vindo da STM32CubeMX para lidar com o sistema de arquivos FATFS, juntamente com a biblioteca `fatfs_sd_card`. 
- A escrita no display lcd é feita através da biblioteca `i2c-lcd`, pelo módulo I2C.
- A reprodução do áudio é feita através do módulo TIM, controlando diretamente o registrador TIM1CCR1, que controla o duty-cicle do PWM.

Após a preparação mostrada acima, o resto do projeto pode ser reproduzido copiando os conteúdos de `main.c`
