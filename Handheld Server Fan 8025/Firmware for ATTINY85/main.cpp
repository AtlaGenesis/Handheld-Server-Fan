// ATTINY85 PWM generator on PB1 (OC1A) at 25 kHz
// Potentiometer on PB2 (ADC1), mapped 25%..100% duty
// Startup behavior per spec with soft ramps

#define F_CPU 8000000UL  // CPU clock for delay timing; set to your fuse clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Timer1 runs from PLL (64 MHz) with prescaler /16 for 25 kHz
// fPWM = 64 MHz / (16 * (TOP + 1)) => TOP = 159 for 25 kHz
#define PWM_TOP 159

#define ADC_MIN_THRESHOLD 8   // treat <= ~0.8% as "minimum" (noise guard)
#define RAMP_STEP_MS 5         // ramp granularity

static void pwm_init(void);
static void adc_init(void);
static uint16_t adc_read(void);
static uint16_t adc_read_avg(uint8_t samples);
static uint8_t adc_to_duty(uint16_t adc);
static void pwm_set_duty(uint8_t duty_percent);
static void ramp_duty(uint8_t start, uint8_t end, uint16_t duration_ms);

int main(void) {
    pwm_init();
    adc_init();

    _delay_ms(10);       // allow ADC to settle
    (void)adc_read();    // discard first conversion

    uint16_t adc_start = adc_read_avg(8);
    uint8_t target = adc_to_duty(adc_start);

    if (adc_start <= ADC_MIN_THRESHOLD) {
        pwm_set_duty(0);
        ramp_duty(0, 25, 750);   // 0% -> 25% in 0.75 s
        target = 25;
    } else {
        pwm_set_duty(25);
        ramp_duty(25, target, 1000); // 25% -> target in 1 s
    }

    uint8_t current = target;

    while (1) {
        uint16_t adc = adc_read_avg(4);
        uint8_t duty = adc_to_duty(adc);

        if (duty != current) {
            pwm_set_duty(duty);
            current = duty;
        }

        _delay_ms(5);  // modest update rate to reduce ADC jitter
    }
}

// ------------------------- PWM -------------------------

static void pwm_init(void) {
    DDRB |= (1 << PB1);   // PB1 as output (OC1A)
    PORTB &= ~(1 << PB1); // ensure low at start

    // Enable PLL (64 MHz) and route to Timer1
    PLLCSR = (1 << PLLE);
    while (!(PLLCSR & (1 << PLOCK))) {
        // wait for PLL lock
    }
    PLLCSR |= (1 << PCKE);

    // Set PWM top and initialize duty
    OCR1C = PWM_TOP;
    OCR1A = 0;

    // Fast PWM on OC1A, non-inverting, prescaler /16
    // Prescaler bits for /16: CS12 and CS10 set (per ATtiny85 Timer1)
    TCCR1 = (1 << PWM1A) | (1 << COM1A1) | (1 << CS12) | (1 << CS10);
}

static void pwm_set_duty(uint8_t duty_percent) {
    if (duty_percent == 0) {
        // Disconnect OC1A and drive low for true 0%
        TCCR1 &= ~((1 << COM1A1) | (1 << COM1A0));
        PORTB &= ~(1 << PB1);
        OCR1A = 0;
        return;
    }

    // Ensure OC1A connected in non-inverting mode
    TCCR1 = (TCCR1 & ~((1 << COM1A1) | (1 << COM1A0))) | (1 << COM1A1);

    uint16_t compare = ((uint32_t)(PWM_TOP + 1) * duty_percent) / 100;
    if (compare == 0) compare = 1;
    if (compare > PWM_TOP) compare = PWM_TOP;

    OCR1A = (uint8_t)compare;
}

// ------------------------- ADC -------------------------

static void adc_init(void) {
    // ADC1 (PB2), VCC as reference
    ADMUX = (1 << MUX0);  // MUX=001 for ADC1
    ADCSRA = (1 << ADEN)  // enable ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler /128
    DIDR0 |= (1 << ADC1D); // disable digital input on ADC1
}

static uint16_t adc_read(void) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {
        // wait for conversion
    }
    return ADC;
}

static uint16_t adc_read_avg(uint8_t samples) {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < samples; i++) {
        acc += adc_read();
    }
    return (uint16_t)(acc / samples);
}

static uint8_t adc_to_duty(uint16_t adc) {
    // Map 0..1023 -> 25..100
    uint32_t duty = 25 + ((uint32_t)adc * 75) / 1023;
    if (duty > 100) duty = 100;
    return (uint8_t)duty;
}

// ------------------------- Ramping -------------------------

static void ramp_duty(uint8_t start, uint8_t end, uint16_t duration_ms) {
    if (duration_ms == 0 || start == end) {
        pwm_set_duty(end);
        return;
    }

    uint16_t steps = duration_ms / RAMP_STEP_MS;
    if (steps == 0) steps = 1;

    int16_t delta = (int16_t)end - (int16_t)start;

    for (uint16_t i = 0; i < steps; i++) {
        uint8_t duty = (uint8_t)(start + ((int32_t)delta * i) / steps);
        pwm_set_duty(duty);
        _delay_ms(RAMP_STEP_MS);
    }

    pwm_set_duty(end);
}
