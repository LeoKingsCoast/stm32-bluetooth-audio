# Introdução 

Este é um projeto de Iniciação Científica para minha graduação. O projeto consiste em um sistema controlado por um microcontrolador STM32F103C8T6 capaz de receber arquivos de áudio por bluetooth, armazená-los em um cartão Micro SD e reproduzí-lo.

# Passos para Implementação do Projeto

Se você quiser fazer uma montagem funcional deste projeto para testes, siga os passos a seguir. Se desejar implementar o projeto do zero, vendo em detalhes como preparar cada componente, veja [Implementando este Projeto do Zero](FROMSCRATCH.md).

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

