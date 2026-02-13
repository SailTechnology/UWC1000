#ifndef MAVLINK_TASK_H
#define MAVLINK_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define MAVLINK_UART_HANDLE (&huart1)

// For an onboard companion device, it's typical to use the *same* SYSID as the vehicle
// and a distinct COMPID. In this project we learn the vehicle SYSID from heartbeat and
// mirror it automatically. This is only the startup fallback before we know the vehicle SYSID.
#define MAVLINK_LOCAL_SYS_ID_FALLBACK 1u
// 200 is a safe, distinct component id for an onboard controller.
#define MAVLINK_LOCAL_COMP_ID 200u

#define MAVLINK_TARGET_SYS_ID 1u
#define MAVLINK_TARGET_COMP_ID 1u

typedef struct {
  uint8_t sysid;
  uint8_t compid;
  uint8_t type;
  uint8_t autopilot;
  uint8_t base_mode;
  uint32_t custom_mode;
  uint8_t system_status;
  uint8_t mavlink_version;

  uint16_t battery_voltage_mv;
  int16_t battery_current_ca;
  int8_t battery_remaining;

  float roll;
  float pitch;
  float yaw;
  float rollspeed;
  float pitchspeed;
  float yawspeed;

  int32_t lat;
  int32_t lon;
  int32_t alt_mm;
  int32_t relative_alt_mm;
  int16_t vx;
  int16_t vy;
  int16_t vz;
  uint16_t heading_cdeg;

  uint16_t last_command_ack;
  uint8_t last_command_result;

  uint32_t last_heartbeat_ms;
  uint32_t last_sys_status_ms;
  uint32_t last_attitude_ms;
  uint32_t last_position_ms;
  uint32_t last_command_ack_ms;
} MavlinkTelemetry_t;

void MavlinkTask_Run(void);
bool Mavlink_GetTelemetry(MavlinkTelemetry_t *out);
bool Mavlink_SendCommandLong(uint16_t command,
                             float param1,
                             float param2,
                             float param3,
                             float param4,
                             float param5,
                             float param6,
                             float param7,
                             uint8_t target_system,
                             uint8_t target_component,
                             uint8_t confirmation);
bool Mavlink_RequestMessageInterval(uint32_t message_id, uint32_t interval_us);
bool Mavlink_SendStatustext(const char *text, uint8_t severity);
uint8_t Mavlink_GetTargetSystem(void);
uint8_t Mavlink_GetTargetComponent(void);

#ifdef __cplusplus
}
#endif

#endif
