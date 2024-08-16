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

- Para fazer o envio do sinal, volte ao repositório principal do projeto e vá para `envioSerial/`. Após isso, mova o arquivo de audio no formato mp3 para o diretório `envioSerial/`. IMPORTANTE: O arquivo deve possuir o nome `audio.mp3` Após isso, compile o programa que será executado para fazer o envio do áudio.
```bash
cd envioSerial
mv /path/to/audio/audio.mp3 /path/to/project/envioSerial/
make
```

- Pressione o botão conectado a PA11 para preparar o STM32 para receber o áudio. 

- Se não tiver feito ainda, conecte o módulo bluetooth ao seu computador da mesma forma que qualquer outro dispositivo bluetooth. Se a conexão tiver sido bem sucedida, o LED do módulo irá parar de piscar.

- Execute o programa e espere o envio terminar. Após o término do envio, aperte novamente o botão conectado ao PA11 para voltar ao estado normal do STM32. ATENÇÃO: Não aperte o botão PA11 novamente até que o LED PC13 (embutido na placa Bluepill) pare de piscar, ele indica que os dados ainda estão sendo escritos no cartão SD. Se o botão for apertado antes disso, o audio terá seu fim cortado.

- Nota: O processo de armazenamento dos dados no cartão SD é lento, um arquivo de áudio de 1.5s com sample rate de 22050 Hz leva cerca de 2 minutos para ser escrito. Esse tempo pode ser diminuido alterando o sample rate.

## Problemas que podem ocorrer

- O programa que envia o áudio por bluetooth assume que o módulo foi conectado através da porta seria /dev/rfcomm0. Verifique se este é o caso. Você pode usar o comando `dmesg` após conectar o módulo bluetooth para verificar em qual porta ele foi conectado. É possível que o seu driver bluetooth também mostre essa informação dentre as propriedades do dispositivo conectado.

- Se o dispositivo estiver conectado na porta correta, mas o programa ainda não conseguir conectar, é possível que seu usuário não possua permissão para acessar a porta bluetooth. Nesse caso, execute:
```bash
sudo adduser $USER dialout
```
    - *Será preciso relogar para que esse comando seja efetivado

# Componentes utilizados

- Placa STM32 Bluepill (Microcontrolador STM32F103C8T6)
- Módulo Micro SD
- Módulo Bluetooth HC-06
- Display LCD (Opcional)
- 2x Botões
- 1x Resistor 270, 2x Resistores 10k, 1x Resistor 20k, 1x Resistor 100k, 1x Resistor 4.7Meg
- 1x Capacitor 680nF, 1x Capacitor 1uF, 1x Capacitor 2.2uF
- 1x Indutor 100uH
- Amplificador Operacional LM4818
- Auto-falante
- **Outros**

# Esquemático e Explicação do Circuito

A seguir é mostrado o esquemático do circuito, juntamente com uma imagem do protótipo.


