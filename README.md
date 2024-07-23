# Introdução 

Lorem

# Configuração CubeMX

## Configuração do clock

- Ir à aba RCC e selecionar **High Speed Clock (HSE):** Crystal/Ceramic Resonator.
- Ir até Clock Configuration, selecionar HSE no primeiro multiplexador, PLLCLK no segundo e configurar a máxima frequência em HCLK (72 MHz).

## Botões de controle

- Selecionar os pinos PB10 e PB11 como GPIO_EXTIxx.
- Ir até GPIO > NVIC.

## Sinal de PWM

- Em Timers, selecionar TIM1.
- Configurações TIM1:
    - **Clock Source:** Internal Clock
    - **Channel1**: PWM Generation CH1
    - **Counter Period:** 8192

## Cartão SD

- Em Connectivity, selecionar SPI1
- Configurações SPI1:
    - **Mode:** Full-Duplex Master
    - **Prescaler:** 16 
- Em Middleware and Software Packs, selecionar FATFS
- Configurações FATFS:
    - Selecionar **User-defined** 
    - **USE-LFN (Use Long Filename)**: Enabled with static working buffer on the BSS
    - **MAX_SS:** 4096 

## Módulo Bluetooth

- Em Connectivity, selecionar USART1.
- Configurações USART1 > Parameter Settings:
    - **Mode:** Asyncronous
    - **Baud Rate:** 9600 Bits/s 
- Configurações USART1 > NVIC Settings:
    - Selecionar **USAR1 global interrupt** 
- Configurações USART1 > DMA Settings:
    - Selecionar **Add** > **USART1_RX**
    - **Mode:** Circular 

## Display LCD

- Em Connectivity, selecionar I2C1.
- Configurações I2C1:
    - **I2C:** I2C


