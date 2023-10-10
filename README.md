# LoRaWAN_Radioenge_STM32L412KB
Placa Núcleo STM32L412KKB se comunicando com módulo LoRaWAN da Radioenge. 

Este é um exemplo de como fazer uma comunicação AT via UART via um STM32L412KB. Neste caso acabei utilizando o módulo da Radioenge por já ter domínio sobre ela, além também de ter refeito a biblioteca dele para Arduino: https://github.com/elcereza/LoRaWAN

Existem pontos de melhorias que da para serem feitos no código que montei, afinal não sou tão bom em C ANSI e foi um estudo para mim. De qualquer forma esse código pode te ajudar a compreender melhor a infraestrutura do STM32 para suas próprias aplicações. 

É importante considerar que a estrutura do STM32CubeIDE trabalha com comentários do tipo:
```
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

```
Essas estruturas são extremamentes importantes para o código montado dentro da IDE, pois há um autogerador de código dentro da IDE e caso você não respeite isso, poderá perder o código que foi montado. Se por exemplo você cria seus '#defines' fora desses comentários, é certo que perderá seu código a qualquer atualização do autogerador...

O diretório do código fica em: Core/Src/main.c
