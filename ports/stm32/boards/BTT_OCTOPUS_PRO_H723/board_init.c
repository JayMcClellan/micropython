#include "py/mphal.h"
#include "timer.h"
#include "motion.h"

// Experiment: TIM24 is unused by MicroPython on this MCU, so it's free to
// drive directly. It's configured to count at 100kHz and fire an update
// interrupt on every tick; toggling PE15 each tick gives a 50kHz square
// wave on the pin, to confirm the timer is running.
void TIM24_IRQHandler(void) {
    TIM24->SR = ~TIM_SR_UIF;
    pin_E15->gpio->ODR ^= pin_E15->pin_mask;
}

void BTT_OCTOPUS_PRO_H723_board_early_init(void) {
    motion_init();

    mp_hal_pin_output(pin_E15);
    mp_hal_pin_low(pin_E15);
    mp_hal_pin_high(pin_E15);
    mp_hal_pin_low(pin_E15);

    __HAL_RCC_TIM24_CLK_ENABLE();

    // Run the counter at its full source rate and use ARR to set the
    // 100kHz update period, rather than prescaling down to 100kHz and
    // using ARR=0 -- with ARR=0 the counter reloads every clock edge and
    // never holds a nonzero value, so CNT is useless as a liveness check.
    TIM24->PSC = 0;
    TIM24->ARR = (timer_get_source_freq(24) / 100000) - 1;
    TIM24->EGR = TIM_EGR_UG; // load PSC/ARR, reset CNT
    TIM24->SR = ~TIM_SR_UIF; // clear UIF set by the forced update above
    TIM24->DIER = TIM_DIER_UIE;
    TIM24->CR1 = TIM_CR1_CEN;

    NVIC_SetPriority(TIM24_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 2, 0));
    HAL_NVIC_EnableIRQ(TIM24_IRQn);
}
