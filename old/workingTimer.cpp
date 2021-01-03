#include <Arduino.h>

volatile int flag = 0;

extern "C"
{
   void TIMER4_IRQHandler_v()
  {
    if (NRF_TIMER4->EVENTS_COMPARE[0] == 1)
    {   
        flag++;
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    }
  }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Configuring timer");
 
    NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER4->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos;
    NRF_TIMER4->CC[0] = 10000;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

    NVIC_EnableIRQ(TIMER4_IRQn);
    NRF_TIMER4->TASKS_START = 1;
}

void loop() {
    Serial.println(flag);
}