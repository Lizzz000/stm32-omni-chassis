#ifndef CONTROL_TIMING_H
#define CONTROL_TIMING_H

/* TIM7: APB1 timer clock 84MHz / (83+1) = 1MHz counter. */
#define CHASSIS_CONTROL_PERIOD_MS 10U
#define CHASSIS_CONTROL_TIMER_ARR (CHASSIS_CONTROL_PERIOD_MS * 1000U - 1U)

/* Keep the existing 100/200/250ms debounce windows exactly representable. */
#if (100U % CHASSIS_CONTROL_PERIOD_MS) || (200U % CHASSIS_CONTROL_PERIOD_MS) || (250U % CHASSIS_CONTROL_PERIOD_MS)
#error "Control period must divide protection and settling windows"
#endif

#endif
