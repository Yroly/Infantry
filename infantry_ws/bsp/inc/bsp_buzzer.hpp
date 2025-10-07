#pragma once
#include "stdint.h"
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern void buzzer_note(uint16_t note,float volume);

