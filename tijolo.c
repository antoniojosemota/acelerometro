#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

// Definições de GPIOs e endereço do MPU6050
#define MPU6050_ADDR 0x68
#define SDA_PIN 2
#define SCL_PIN 3
#define LED_VERDE 12  // Inclinação < 15°
#define LED_VERMELHO 13 // Inclinação > 30°
#define LED_AZUL 11  // 15° <= Inclinação <= 30°
#define GRAVITY 9.81
#define ACEL 3.6
#define THRESHOLD 0.05 

float velocidade = 0.0;
absolute_time_t timer_back;
float ax_g = 0;
float ay_g = 0;
float az_g = 0;
float ax2ms, ay2ms, az2ms;
int16_t ax, ay, az;

void inicializar_mpu6050() {
    uint8_t reset[2] = {0x6B, 0x00}; // Endereço do registrador Power Management 1
    i2c_write_blocking(i2c1, MPU6050_ADDR, reset, 2, false);
}

void ler_acelerometro(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buffer[6];
    i2c_write_blocking(i2c1, MPU6050_ADDR, (uint8_t[]){0x3B}, 1, true); // Registrador do acelerômetro
    i2c_read_blocking(i2c1, MPU6050_ADDR, buffer, 6, false);

    *ax = (buffer[0] << 8) | buffer[1];
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
}

float calcular_inclinacao(int16_t ax, int16_t ay, int16_t az) {
    ax_g = ax / 16384.0; // Converte para g
    ay_g = ay / 16384.0;
    az_g = az / 16384.0;

    float inclinacao = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0 / M_PI);
    return inclinacao;
}

void inicializar_leds() {
    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_put(LED_VERDE, 0);

    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);
    gpio_put(LED_VERMELHO, 0);

    gpio_init(LED_AZUL);
    gpio_set_dir(LED_AZUL, GPIO_OUT);
    gpio_put(LED_AZUL, 0);
}

float threshold_acelerometer(int samples){
    float sum = 0;
    float midrain;

    for (int i = 0; i < samples; i++){
        ler_acelerometro(&ax, &ay, &az);
        calcular_inclinacao(ax, ay, az);  // Lê os valores do acelerômetro
        ax_g = ax_g *9.81f;  // Converte os valores para g e para m/s²
        ay_g = ay_g *9.81f;
        az_g = az_g *9.81f;
        midrain = sqrt(ax_g*ax_g + az_g*az_g);
        sum += midrain;
        sleep_us(1000);
    }
    
    float mean = sum / samples;

    return mean;
}

void acionar_led(float inclinacao) {
    if (inclinacao > 30.0) {
        gpio_put(LED_VERMELHO, 1);  // Liga o LED vermelho
        gpio_put(LED_AZUL, 0);
        gpio_put(LED_VERDE, 0);
        printf("LED VERMELHO: Inclinação acima de 30°\n");
    } else if (inclinacao >= 15.0 && inclinacao <= 30.0) {
        gpio_put(LED_VERMELHO, 0);
        gpio_put(LED_AZUL, 1);  // Liga o LED azul
        gpio_put(LED_VERDE, 0);
        printf("LED AZUL: Inclinação entre 15° e 30°\n");
    } else {
        gpio_put(LED_VERMELHO, 0);
        gpio_put(LED_AZUL, 0);
        gpio_put(LED_VERDE, 1);  // Liga o LED verde
        printf("LED VERDE: Inclinação abaixo de 15°\n");
    }
}

float acel_linear(){
    ax_g = (ax / 16384.0)*9.81f;  // Converte os valores para g e para m/s²
    az_g = (az / 16384.0)*9.81f;

    float result = sqrt(ax_g * ax_g + ay_g * ay_g);
    printf("%.2f\n", result);
    return result;
}

int main() {
    stdio_init_all();
    i2c_init(i2c1, 400*1000); // Inicializa I2C
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    inicializar_mpu6050();
    inicializar_leds();

    float callibrateOFFset = threshold_acelerometer(100);

    sleep_ms(5000);

    printf("Linha de tolerância = %.2f\n", callibrateOFFset);

    while (true) {
        timer_back = get_absolute_time();

        ler_acelerometro(&ax, &ay, &az);

        absolute_time_t actual_time = get_absolute_time();
        int64_t delta_us = absolute_time_diff_us(timer_back, actual_time);
        float delta = delta_us / 1e6;
        timer_back = actual_time;

        float acel_vetor = acel_linear();

        if (acel_vetor > callibrateOFFset){
            printf("Lançamento rapido\n");
        } 

        float inclinacao = calcular_inclinacao(ax, ay, az);

        printf("Acelerômetro: X=%d, Y=%d, Z=%d\n", ax, ay, az);
        printf("Inclinação no eixo X: %.2f graus\n", inclinacao);
        printf("Aceleração em X: %.2f m/s², Y: %.2f m/s², Z: %.2f m/s²", ax_g, ay_g, az_g);

        acionar_led(inclinacao);

        sleep_ms(1000); // Atualiza a cada 1 segundo
    }
}
