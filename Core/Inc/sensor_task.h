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
  /* 最近一次解析得到的温度（摄氏度） */
  float temperature_c;
  /* 最近一次解析得到的深度（dbar） */
  float depth_dbar;
  /* 压力传感器电压（mV），用于联调校验 */
  float pressure_mv;
  /* 温度电阻（欧姆），用于传感器诊断 */
  float temperature_resistance_ohm;
  /* 最近一次成功更新时间戳（ms） */
  uint32_t last_update_ms;
  /* 成功采样累计次数 */
  uint32_t success_count;
  /* 失败采样累计次数 */
  uint32_t error_count;
  /* 最近一次错误码（对应 sensor_task.c 的 SensorError_t） */
  uint8_t last_error_code;
  /* 当前缓存是否有效（至少成功采样过一次） */
  bool valid;
} SensorData_t;

/* 无锁快照（volatile），适合快速观测；严格并发读取建议用 SensorTask_GetData() */
extern volatile SensorData_t g_sensor_latest;

/* 传感器任务主函数（由 RTOS 线程入口调用） */
void SensorTask_Run(void);
/* 线程安全读取传感器缓存 */
bool SensorTask_GetData(SensorData_t *out);

#ifdef __cplusplus
}
#endif

#endif
