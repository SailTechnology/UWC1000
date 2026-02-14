#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

typedef struct {
  float temperature_c;
  float depth_dbar;
  float pressure_mv;
  float temperature_resistance_ohm;
  uint32_t last_update_ms;
  uint32_t success_count;
  uint32_t error_count;
  uint8_t last_error_code;
  bool valid;
} SensorData_t;

extern volatile SensorData_t g_sensor_latest;

void SensorTask_Run(void);
bool SensorTask_GetData(SensorData_t *out);

#ifdef __cplusplus
}
#endif

#endif
