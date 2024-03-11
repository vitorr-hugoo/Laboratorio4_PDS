#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define pinSensor 32

const float referenciaADC = 3.3; // Tensão de referência do ADC
const int resolucaoADC = 12;     // Resolução do ADC (bits)

const int bufferSize = 128;
float buffer[bufferSize];
float *ponteiroBuffer = buffer; // Inicializa o ponteiro para o início do array

float leituraParaTensao(int leituraADC)
{
    return (leituraADC / (float)(1 << resolucaoADC)) * referenciaADC;
}

void imprimirBuffer()
{
    Serial.println("Valores no Buffer Circular:");
    for (int i = 0; i < bufferSize; i++)
    {
        Serial.printf("%.4f\n", buffer[i]);
    }
}

void adicionarAoBuffer(float valor)
{
    // Adiciona o valor ao buffer
    *ponteiroBuffer = valor;
    ponteiroBuffer++;

    // Verifica se o ponteiro ultrapassou o final do array e reinicia se necessário
    if (ponteiroBuffer >= buffer + bufferSize)
    {
        ponteiroBuffer = buffer;
    }
}

void tarefaLeitura(void *parametros)
{
    while (1)
    {
        int leituraADC = analogRead(pinSensor);
        float tensao = leituraParaTensao(leituraADC);
        adicionarAoBuffer(tensao);

        vTaskDelay(pdMS_TO_TICKS(1)); // Atraso de 1 ms
    }
}

void tarefaImprimirBuffer(void *parametros)
{
    while (1)
    {
        // Imprime o buffer periodicamente
        imprimirBuffer();

        vTaskDelay(pdMS_TO_TICKS(5000)); // Atraso de 5 segundos
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(pinSensor, INPUT);

    xTaskCreatePinnedToCore(tarefaLeitura, "TarefaLeitura", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tarefaImprimirBuffer, "TarefaImprimirBuffer", 2048, NULL, 1, NULL, 0);
}

void loop()
{
    // loop vazio, a execução ocorre nas tarefas do FreeRTOS
}
