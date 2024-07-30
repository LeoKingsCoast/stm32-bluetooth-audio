# Introdução 

Este é um projeto de Iniciação Científica para minha graduação. O projeto consiste em um sistema controlado por um microcontrolador STM32F103C8T6 capaz de receber arquivos de áudio por bluetooth, armazená-los em um cartão Micro SD e reproduzí-lo.

# Passos para Implementação do Projeto

Se você quiser fazer uma montagem funcional deste projeto. Siga os seguintes passos:

- Se você estiver utilizando Linux, siga os passos seguintes, eles devem funcionar também em MacOS. Para a implementação apresentada aqui, é usado um Makefile para a compilação do código e um [Driver para o ST Link]() para fazer o upload do código para a placa STM32. Se estiver usando Windows ou se desejar utilizar a STM32CubeIDE disponibilizada pela ST, vá para [link](). 

- Veja [Componentes utilizados](#componentes-utilizados) e [Esquemático e Explicação do Circuito](#esquemático-e-explicação-do-circuito) para montar o ciruito.

- Clone este repositório para sua máquina:
```bash
git clone
```

- Siga os passos em [link]() para obter os programas necessários para fazer o upload do código para o microcontrolador.

- Vá para o diretório do projeto e compile o código
```bash
cd Bluetooth_Audio_Player
make
```

- Após montar o circuito e conectar a placa ao ST-Link, vá para o diretório build e faça o upload do arquivo binário para a placa
```bash
cd build
st-flash --reset write Bluetooth_Audio_Player.bin 0x8000000
```

## Testando o circuito

- Ao apertar o botão conectado em PA10, um audio deve ser tocado caso já esteja armazenado

- Ao apertar o botão conectado em PA11, o sistema irá iniciar a espera de um sinal de audio para armazenar no cartão SD. ATENÇÃO: Se um arquivo já estiver presente com o nome "Audio", ele será sobreescrito no momento que o botão for apertado

- Para fazer o envio do sinal, volte ao repositório principal do projeto e vá para `envioSerial/`
```bash
cd envioSerial
```

# Componentes utilizados

- Placa STM32 Bluepill (Microcontrolador STM32F103C8T6)
- Módulo Micro SD
- Módulo Bluetooth HC-06
- Display LCD (Opcional)
- 2x Resistores 10k
- 2x Botões
- **Outros**
- Auto-falante

# Esquemático e Explicação do Circuito

# Recriando o código

## Configuração Inicial

### Configuração CubeMX

#### Configuração do clock

- Ir à aba RCC e selecionar **High Speed Clock (HSE):** Crystal/Ceramic Resonator.
- Ir até Clock Configuration, selecionar HSE no primeiro multiplexador, PLLCLK no segundo e configurar a máxima frequência em HCLK (72 MHz).

#### Botões de controle

- Selecionar os pinos PB10 e PB11 como GPIO_EXTIxx.
- Ir até GPIO > NVIC.

#### Sinal de PWM

- Em Timers, selecionar TIM1.
- Configurações TIM1:
    - **Clock Source:** Internal Clock
    - **Channel1**: PWM Generation CH1
    - **Counter Period:** 8192

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
