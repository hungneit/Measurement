#include "stm32f1xx.h"
#include <stdio.h>

#define DHT11_PIN     9 
#define RAIN_PIN      8 
#define SERVO_PIN     8  
#define UART_TX_PIN   9  


void SystemInit72MHz(void);
void uart_init(void);
void uart_send(char *s);
void timer_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void gpio_input_pb(uint8_t pin);
void dht11_output_mode(void);
void servo_pwm_init(void);
void servo_set_angle(int angle);
uint8_t dht11_read(uint8_t *temperature, uint8_t *humidity);

void uart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (0x0B << 4);
    USART1->BRR = 72000000 / 9600;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_send(char *s) {
    while (*s) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *s++;
    }
}

void SystemInit72MHz(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint32_t us) {
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) delay_us(1000);
}

void gpio_input_pb(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    if (pin >= 8) {
        GPIOB->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOB->CRH |=  (0x4 << ((pin - 8) * 4));
    }
}

void dht11_output_mode(void) {
    GPIOB->CRH &= ~(0xF << 4); 
    GPIOB->CRH |= (0x3 << 4);  
}

void servo_pwm_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;
    GPIOA->CRH &= ~(0xF << 0);
    GPIOA->CRH |= (0x0B << 0);

    TIM1->PSC = 72 - 1;
    TIM1->ARR = 20000 - 1;
    TIM1->CCR1 = 500;
    TIM1->CCMR1 |= (6 << 4);
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void servo_set_angle(int angle) {
    uint16_t pulse = 500 + (angle * 2000) / 180;
    TIM1->CCR1 = pulse;
}

uint8_t dht11_read(uint8_t *temperature, uint8_t *humidity) {
    uint8_t data[5] = {0};
    uint32_t timeout;

    dht11_output_mode();
    GPIOB->ODR &= ~(1 << DHT11_PIN);
    delay_ms(18);
    GPIOB->ODR |= (1 << DHT11_PIN);
    delay_us(30);
    gpio_input_pb(DHT11_PIN);

    timeout = 10000;
    while (!(GPIOB->IDR & (1 << DHT11_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;
    timeout = 10000;
    while ((GPIOB->IDR & (1 << DHT11_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;

    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            timeout = 10000;
            while (!(GPIOB->IDR & (1 << DHT11_PIN)) && timeout--) delay_us(1);
            delay_us(40);
            data[i] <<= 1;
            if (GPIOB->IDR & (1 << DHT11_PIN)) data[i] |= 1;
            timeout = 10000;
            while ((GPIOB->IDR & (1 << DHT11_PIN)) && timeout--) delay_us(1);
        }
    }

    if (data[4] != (data[0] + data[1] + data[2] + data[3])) return 1;
    *humidity = data[0];
    *temperature = data[2];
    return 0;
}


int main(void) {
    SystemInit72MHz();
    uart_init();
    timer_init();
    servo_pwm_init();
    gpio_input_pb(DHT11_PIN);
    gpio_input_pb(RAIN_PIN);

    uart_send("Khoi dong OK\r\n");

    uint8_t temp = 0, humi = 0;
    char buffer[64];

    while (1) {
        uint8_t dht_ok = dht11_read(&temp, &humi);
        uint8_t is_raining = !(GPIOB->IDR & (1 << RAIN_PIN));

        if (dht_ok == 0) {
            sprintf(buffer, "Nhiet do: %d C, Do am: %d%%\r\n", temp, humi);
            uart_send(buffer);
        } else {
            uart_send("Doc DHT11 that bai\r\n");
        }

        if (is_raining || temp >= 35 || humi >= 80) {
            uart_send("DIEU KIEN KICH HOAT: Dong mai (Servo 90 do)\r\n");
            servo_set_angle(90);
        } else if (temp <= 30 && humi <= 75) {
            uart_send("DIEU KIEN CHO PHEP: Mo mai (Servo 0 do)\r\n");
            servo_set_angle(0);
        }

        delay_ms(2000);
    }
}
