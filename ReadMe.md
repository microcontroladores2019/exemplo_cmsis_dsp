# Descrição do Projeto

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

**Análise Espectral com Microfone MP45DT02**

O objetivo desse projeto foi utilizar a arquitetura de DSP do microprocessador Cortex M4 presente na placa de desenvolvimento STM32F4-Discovery. Uma vez configurado o microfone e implementado o protocolo de comunição I2S, é feita a análise espectral do sinal para obter a frequência dominante do seu espectro. Dependendo da faixa em que se obtém um máximo em módulo da FFT, acende-se um LED apropriado.  

Além da utilização da FFT, também necessita-se das funcionalidades de DSP do microcontrolador para trabalhar com a codificação do sinal medido pelo microfone. 

Pelo microfone ser digital, não há necessidade de utilizar um ADC, conversor analógico digital. No entando, isso implica na impossibilidade das medições feitas pelo microfone serem feitas diretamente na modulação PCM, Pulse Code Modulation, que é a modulação gerada por um processo de quantização. Em vez disso, utiliza-se a modulação PDM, Pulse Density Modulation. É necessário um processo de filtragem para realizar a conversão entre as modulações. 

Essa filtragem é feita com um filtro passa baixas, seguido de um decimador e por fim um filtro passa altas. Naturalmente, para essa aplicação de áudio, escolhemos escolhemos as frequências de corte nos valores em que o ouvido humano é imcapaz de detectar. Foi escolhida a frequência inferior de 10Hz e de 22kHz para o limite superior da banda.

Uma vez realizada essa conversão para a codificação PCM, os dados encontram-se num formato adequado para operações matemáticas da biblioteca **CMSIS** da ST.

Todos os passos suprecitados foram empregados utilizando os recursos de FreeRTOS da "Real Time Engineers". Cada passo foi implementado como uma tarefa que é agendada pelo sistema operacional em tempo real, o que possibilita uma menor economia de energia e eficiência no processamento. 

 Um teste interessante que pode ser feito, mas que ainda não foi realizado por falta dos meios necessários, é escolher uma frequência superior e utilizar um apito para cães, ou então um dispositivo ultrasônico, e verificar o funcionamento do sistema nesses casos. Isso também leva a possíveis aplicações de tecnologia de ultrassom para deficiente visuais por exemplo. 
### Documentação das Funções
    MP45DT02_config: configuração do microfone da comunicação I2S
    fftConfig: configuração da arquiteruda de DSP utilizada na FFT
    Init_LEDS: configura a inicialização dos LEDS de GPIO
    LEDS_OFF: desliga os LEDS
    AudioRecStart: Inicializa a comunicação I2S para o microfone
    AudioRecStop: Desliga a comunicação I2S, na finalização 
    ButtonReader: função da tarefa de leitura do botão
    PDM_Filter: função da tarefa de conversão PDM para PCM
    Store_Analyse: função da tarefa de análise espectral
    

### Referências

  - Application note AN3997 : "Audio Playback and Recording using ST32F4Discovery", encontrado no site da ST;  
  - Application note AN3998 : "PDM audio software decoding on STM32 microcontrollers", encontrado no site da ST; 
  - Application note AN4841 : "Digital signal processing for STM32 microcontrollers using CMSIS", encontrado no site da ST; 
  - Reference manual RM0090 - Manual de Referência da Discovery
  - A implementação das bibliotecas foi possível devido à ajuda do professor Luiz Renault. 

### Fluxograma 

### Diagrama em Blocos

### Pinagem














