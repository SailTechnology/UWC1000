#include "sensor_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define SENSOR_MODBUS_SLAVE_ID 1u
#define SENSOR_MODBUS_FUNC_READ_INPUT 0x04u
#define SENSOR_MODBUS_FUNC_READ_HOLDING 0x03u
#define SENSOR_MODBUS_FUNC_WRITE_SINGLE 0x06u

#define SENSOR_REG_START 0x0010u
#define SENSOR_REG_COUNT 0x000Eu
#define SENSOR_REG_FILE_COUNT 0x0006u
#define SENSOR_REG_WORK_MODE 0x0033u
#define SENSOR_MODE_DEBUG 0x0001u
#define SENSOR_MODE_AUTO 0x0002u

#define SENSOR_POLL_PERIOD_MS 2000u
#define SENSOR_STARTUP_DEBUG_WAIT_MS 2000u
#define SENSOR_MODE_SET_RETRY 3u
#define SENSOR_TX_TIMEOUT_MS 100u
#define SENSOR_RX_FIRST_TIMEOUT_MS 1500u
#define SENSOR_RX_TIMEOUT_MS 100u
#define SENSOR_RX_DRAIN_BYTES 96u
#define SENSOR_FRAME_MAX_LEN 96u
#define SENSOR_UART3_TX_TIMEOUT_MS 100u
#define SENSOR_UART3_OUTPUT_ENABLE 1u

#define SENSOR_OFFSET_TEMP_RES 0u
#define SENSOR_OFFSET_TEMP 8u
#define SENSOR_OFFSET_PRESSURE_MV 16u
#define SENSOR_OFFSET_DEPTH 24u

typedef enum {
  SENSOR_ERR_NONE = 0u,
  SENSOR_ERR_TX = 1u,
  SENSOR_ERR_RX_TIMEOUT = 2u,
  SENSOR_ERR_BAD_ADDR = 3u,
  SENSOR_ERR_BAD_FUNC = 4u,
  SENSOR_ERR_EXCEPTION = 5u,
  SENSOR_ERR_BAD_LEN = 6u,
  SENSOR_ERR_BAD_CRC = 7u
} SensorError_t;

static osMutexId_t g_sensor_mutex = NULL;
static SensorData_t g_sensor_data;
volatile SensorData_t g_sensor_latest;
static char g_sensor_line[128];

static void sensor_uart3_print(const char *s)
{
  if (SENSOR_UART3_OUTPUT_ENABLE == 0u || s == NULL) {
    return;
  }
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), SENSOR_UART3_TX_TIMEOUT_MS);
}

static uint16_t sensor_modbus_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0u; i < len; ++i) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0u; b < 8u; ++b) {
      if ((crc & 0x0001u) != 0u) {
        crc = (uint16_t)((crc >> 1) ^ 0xA001u);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void sensor_uart2_drain_rx(void)
{
  uint8_t byte = 0u;
  for (uint16_t i = 0u; i < SENSOR_RX_DRAIN_BYTES; ++i) {
    if (HAL_UART_Receive(&huart2, &byte, 1u, 2u) != HAL_OK) {
      break;
    }
  }
}

static float sensor_get_f32_be(const uint8_t *p)
{
  uint32_t raw = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) |
                 (uint32_t)p[3];
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static bool sensor_read_input_registers(uint16_t reg_start,
                                        uint16_t reg_count,
                                        uint8_t *payload,
                                        uint8_t *payload_len,
                                        SensorError_t *error_code)
{
  uint8_t req[8];
  req[0] = SENSOR_MODBUS_SLAVE_ID;
  req[1] = SENSOR_MODBUS_FUNC_READ_INPUT;
  req[2] = (uint8_t)((reg_start >> 8) & 0xFFu);
  req[3] = (uint8_t)(reg_start & 0xFFu);
  req[4] = (uint8_t)((reg_count >> 8) & 0xFFu);
  req[5] = (uint8_t)(reg_count & 0xFFu);
  uint16_t req_crc = sensor_modbus_crc16(req, 6u);
  req[6] = (uint8_t)(req_crc & 0xFFu);
  req[7] = (uint8_t)((req_crc >> 8) & 0xFFu);

  sensor_uart2_drain_rx();

  if (HAL_UART_Transmit(&huart2, req, sizeof(req), SENSOR_TX_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_TX;
    return false;
  }

  uint8_t frame[SENSOR_FRAME_MAX_LEN];
  if (HAL_UART_Receive(&huart2, &frame[0], 1u, SENSOR_RX_FIRST_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_RX_TIMEOUT;
    return false;
  }
  if (HAL_UART_Receive(&huart2, &frame[1], 2u, SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_RX_TIMEOUT;
    return false;
  }

  if (frame[0] != SENSOR_MODBUS_SLAVE_ID) {
    *error_code = SENSOR_ERR_BAD_ADDR;
    return false;
  }

  if ((frame[1] & 0x80u) != 0u) {
    if (HAL_UART_Receive(&huart2, &frame[3], 3u, SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
      *error_code = SENSOR_ERR_RX_TIMEOUT;
      return false;
    }
    uint16_t crc_expect = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    uint16_t crc_calc = sensor_modbus_crc16(frame, 4u);
    if (crc_calc != crc_expect) {
      *error_code = SENSOR_ERR_BAD_CRC;
      return false;
    }
    *error_code = SENSOR_ERR_EXCEPTION;
    return false;
  }

  if (frame[1] != SENSOR_MODBUS_FUNC_READ_INPUT && frame[1] != SENSOR_MODBUS_FUNC_READ_HOLDING) {
    *error_code = SENSOR_ERR_BAD_FUNC;
    return false;
  }

  uint8_t bytes = frame[2];
  if (bytes == 0u || (uint16_t)(bytes + 5u) > SENSOR_FRAME_MAX_LEN) {
    *error_code = SENSOR_ERR_BAD_LEN;
    return false;
  }

  if (HAL_UART_Receive(&huart2, &frame[3], (uint16_t)(bytes + 2u), SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_RX_TIMEOUT;
    return false;
  }

  uint16_t frame_len = (uint16_t)(3u + bytes + 2u);
  uint16_t crc_expect = (uint16_t)frame[frame_len - 2u] | ((uint16_t)frame[frame_len - 1u] << 8);
  uint16_t crc_calc = sensor_modbus_crc16(frame, (uint16_t)(frame_len - 2u));
  if (crc_calc != crc_expect) {
    *error_code = SENSOR_ERR_BAD_CRC;
    return false;
  }

  memcpy(payload, &frame[3], bytes);
  *payload_len = bytes;
  *error_code = SENSOR_ERR_NONE;
  return true;
}

static bool sensor_write_single_holding_register(uint16_t reg_addr,
                                                 uint16_t reg_value,
                                                 SensorError_t *error_code)
{
  uint8_t req[8];
  req[0] = SENSOR_MODBUS_SLAVE_ID;
  req[1] = SENSOR_MODBUS_FUNC_WRITE_SINGLE;
  req[2] = (uint8_t)((reg_addr >> 8) & 0xFFu);
  req[3] = (uint8_t)(reg_addr & 0xFFu);
  req[4] = (uint8_t)((reg_value >> 8) & 0xFFu);
  req[5] = (uint8_t)(reg_value & 0xFFu);
  uint16_t req_crc = sensor_modbus_crc16(req, 6u);
  req[6] = (uint8_t)(req_crc & 0xFFu);
  req[7] = (uint8_t)((req_crc >> 8) & 0xFFu);

  sensor_uart2_drain_rx();

  if (HAL_UART_Transmit(&huart2, req, sizeof(req), SENSOR_TX_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_TX;
    return false;
  }

  uint8_t rsp[8];
  if (HAL_UART_Receive(&huart2, &rsp[0], 1u, SENSOR_RX_FIRST_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_RX_TIMEOUT;
    return false;
  }
  if (HAL_UART_Receive(&huart2, &rsp[1], 7u, SENSOR_RX_TIMEOUT_MS) != HAL_OK) {
    *error_code = SENSOR_ERR_RX_TIMEOUT;
    return false;
  }

  if (rsp[0] != SENSOR_MODBUS_SLAVE_ID) {
    *error_code = SENSOR_ERR_BAD_ADDR;
    return false;
  }
  if (rsp[1] != SENSOR_MODBUS_FUNC_WRITE_SINGLE) {
    *error_code = SENSOR_ERR_BAD_FUNC;
    return false;
  }
  if (rsp[2] != req[2] || rsp[3] != req[3] || rsp[4] != req[4] || rsp[5] != req[5]) {
    *error_code = SENSOR_ERR_BAD_LEN;
    return false;
  }

  uint16_t crc_expect = (uint16_t)rsp[6] | ((uint16_t)rsp[7] << 8);
  uint16_t crc_calc = sensor_modbus_crc16(rsp, 6u);
  if (crc_calc != crc_expect) {
    *error_code = SENSOR_ERR_BAD_CRC;
    return false;
  }

  *error_code = SENSOR_ERR_NONE;
  return true;
}

static bool sensor_set_work_mode_with_retry(uint16_t mode_value, SensorError_t *error_code)
{
  for (uint8_t i = 0u; i < SENSOR_MODE_SET_RETRY; ++i) {
    SensorError_t err = SENSOR_ERR_NONE;
    if (sensor_write_single_holding_register(SENSOR_REG_WORK_MODE, mode_value, &err)) {
      *error_code = SENSOR_ERR_NONE;
      return true;
    }
    *error_code = err;
    osDelay(100u);
  }
  return false;
}

static bool sensor_read_file_count(uint16_t *count, SensorError_t *error_code)
{
  uint8_t payload[SENSOR_FRAME_MAX_LEN];
  uint8_t payload_len = 0u;
  if (!sensor_read_input_registers(SENSOR_REG_FILE_COUNT, 1u, payload, &payload_len, error_code)) {
    return false;
  }
  if (payload_len < 2u) {
    *error_code = SENSOR_ERR_BAD_LEN;
    return false;
  }
  *count = ((uint16_t)payload[0] << 8) | (uint16_t)payload[1];
  *error_code = SENSOR_ERR_NONE;
  return true;
}

static void sensor_startup_mode_sequence(void)
{
  SensorError_t err = SENSOR_ERR_NONE;
  uint16_t file_count = 0u;

  if (sensor_read_file_count(&file_count, &err)) {
    (void)snprintf(g_sensor_line, sizeof(g_sensor_line), "SENSOR FILE COUNT:%u\r\n", (unsigned)file_count);
    sensor_uart3_print(g_sensor_line);
  } else {
    (void)snprintf(g_sensor_line, sizeof(g_sensor_line), "SENSOR FILE COUNT ERR:%u\r\n", (unsigned)err);
    sensor_uart3_print(g_sensor_line);
  }

  bool debug_ok = sensor_set_work_mode_with_retry(SENSOR_MODE_DEBUG, &err);
  if (debug_ok) {
    sensor_uart3_print("SENSOR MODE: DEBUG\r\n");
  } else {
    (void)snprintf(g_sensor_line, sizeof(g_sensor_line), "SENSOR MODE DEBUG ERR:%u\r\n", (unsigned)err);
    sensor_uart3_print(g_sensor_line);
  }

  osDelay(SENSOR_STARTUP_DEBUG_WAIT_MS);

  bool auto_ok = sensor_set_work_mode_with_retry(SENSOR_MODE_AUTO, &err);
  if (auto_ok) {
    sensor_uart3_print("SENSOR MODE: AUTO\r\n");
  } else {
    (void)snprintf(g_sensor_line, sizeof(g_sensor_line), "SENSOR MODE AUTO ERR:%u\r\n", (unsigned)err);
    sensor_uart3_print(g_sensor_line);
  }
}

static void sensor_store_data(const SensorData_t *data)
{
  if (g_sensor_mutex == NULL) {
    return;
  }
  if (osMutexAcquire(g_sensor_mutex, 0u) != osOK) {
    return;
  }
  g_sensor_data = *data;
  osMutexRelease(g_sensor_mutex);
}

static void sensor_task_init(void)
{
  if (g_sensor_mutex == NULL) {
    osMutexAttr_t attr = { .name = "sensorMutex" };
    g_sensor_mutex = osMutexNew(&attr);
  }
  memset(&g_sensor_data, 0, sizeof(g_sensor_data));
  memset((void *)&g_sensor_latest, 0, sizeof(g_sensor_latest));
}

void SensorTask_Run(void)
{
  sensor_task_init();
  sensor_startup_mode_sequence();

  uint32_t last_poll = 0u;
  uint8_t payload[SENSOR_FRAME_MAX_LEN];
  uint8_t payload_len = 0u;

  for (;;) {
    uint32_t now = osKernelGetTickCount();
    if ((now - last_poll) >= SENSOR_POLL_PERIOD_MS) {
      SensorData_t data = g_sensor_data;
      SensorError_t err = SENSOR_ERR_NONE;
      bool ok = sensor_read_input_registers(SENSOR_REG_START,
                                            SENSOR_REG_COUNT,
                                            payload,
                                            &payload_len,
                                            &err);
      if (ok && payload_len >= 28u) {
        data.temperature_resistance_ohm = sensor_get_f32_be(&payload[SENSOR_OFFSET_TEMP_RES]);
        data.temperature_c = sensor_get_f32_be(&payload[SENSOR_OFFSET_TEMP]);
        data.pressure_mv = sensor_get_f32_be(&payload[SENSOR_OFFSET_PRESSURE_MV]);
        data.depth_dbar = sensor_get_f32_be(&payload[SENSOR_OFFSET_DEPTH]);
        data.last_update_ms = now;
        data.success_count++;
        data.last_error_code = SENSOR_ERR_NONE;
        data.valid = true;
        (void)snprintf(g_sensor_line,
                       sizeof(g_sensor_line),
                       "SENSOR T:%.3f C D:%.3f dbar\r\n",
                       (double)data.temperature_c,
                       (double)data.depth_dbar);
        sensor_uart3_print(g_sensor_line);
      } else {
        if (ok && payload_len < 28u) {
          err = SENSOR_ERR_BAD_LEN;
        }
        data.error_count++;
        data.last_error_code = (uint8_t)err;
        (void)snprintf(g_sensor_line,
                       sizeof(g_sensor_line),
                       "SENSOR ERR:%u\r\n",
                       (unsigned)data.last_error_code);
        sensor_uart3_print(g_sensor_line);
      }
      g_sensor_latest = data;
      sensor_store_data(&data);
      last_poll = now;
    }

    osDelay(5u);
  }
}

bool SensorTask_GetData(SensorData_t *out)
{
  if (out == NULL || g_sensor_mutex == NULL) {
    return false;
  }
  if (osMutexAcquire(g_sensor_mutex, 0u) != osOK) {
    return false;
  }
  *out = g_sensor_data;
  osMutexRelease(g_sensor_mutex);
  return true;
}
