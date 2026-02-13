#include "mavlink_task.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define MAVLINK_STX_V1 0xFEu
#define MAVLINK_STX_V2 0xFDu
#define MAVLINK_IFLAG_SIGNED 0x01u

#define MAVLINK_MAX_PAYLOAD_LEN 255u
#define MAVLINK_MAX_PACKET_LEN 280u

#define MAVLINK_MSG_ID_HEARTBEAT 0u
#define MAVLINK_MSG_ID_SYS_STATUS 1u
#define MAVLINK_MSG_ID_ATTITUDE 30u
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33u
#define MAVLINK_MSG_ID_RC_CHANNELS 65u
#define MAVLINK_MSG_ID_COMMAND_LONG 76u
#define MAVLINK_MSG_ID_COMMAND_ACK 77u
#define MAVLINK_MSG_ID_STATUSTEXT 253u

#define MAVLINK_CRC_EXTRA_HEARTBEAT 50u
#define MAVLINK_CRC_EXTRA_SYS_STATUS 124u
#define MAVLINK_CRC_EXTRA_ATTITUDE 39u
#define MAVLINK_CRC_EXTRA_GLOBAL_POSITION_INT 104u
#define MAVLINK_CRC_EXTRA_RC_CHANNELS 118u
#define MAVLINK_CRC_EXTRA_COMMAND_LONG 152u
#define MAVLINK_CRC_EXTRA_COMMAND_ACK 143u
#define MAVLINK_CRC_EXTRA_STATUSTEXT 83u

#define MAV_CMD_SET_MESSAGE_INTERVAL 511u

#define MAVLINK_RX_TIMEOUT_MS 2u
#define MAVLINK_TX_TIMEOUT_MS 50u
#define MAVLINK_HEARTBEAT_PERIOD_MS 1000u

#define MAVLINK_RATE_SYS_STATUS_US 500000u
#define MAVLINK_RATE_ATTITUDE_US 100000u
#define MAVLINK_RATE_GLOBAL_POSITION_US 200000u

#define MAVLINK_MAV_TYPE_GCS 6u
#define MAVLINK_MAV_AUTOPILOT_INVALID 8u
#define MAVLINK_MAV_STATE_ACTIVE 4u
#define MAVLINK_VERSION 3u

#define MAVLINK_SEVERITY_INFO 6u
#define MAVLINK_STATUSTEXT_PERIOD_MS 2000u
// When enabled, raw MAVLink bytes received from the flight controller (USART1)
// are forwarded to the PC debug port (USART3). This will look like gibberish in
// an ASCII terminal and can mask readable debug prints below.
#define MAVLINK_DEBUG_BRIDGE 0u
#define MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS 10u
#define MAVLINK_DEBUG_MSGID_ASCII 1u
#define MAVLINK_DEBUG_PRINT_UNKNOWN_FRAMES 0u

#define MAV_MODE_FLAG_SAFETY_ARMED 0x80u

#define MAVLINK_RX_RING_SIZE 1024u
#define MAVLINK_RX_RING_MASK (MAVLINK_RX_RING_SIZE - 1u)
#if ((MAVLINK_RX_RING_SIZE & MAVLINK_RX_RING_MASK) != 0u)
#error "MAVLINK_RX_RING_SIZE must be a power of two"
#endif

typedef struct {
  uint16_t command;
  float params[7];
  uint8_t target_system;
  uint8_t target_component;
  uint8_t confirmation;
} MavlinkCommandLongItem;

typedef struct {
  uint8_t magic;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t index;
  uint16_t expected_len;
} MavlinkParser;

typedef struct {
  uint8_t magic;
  uint8_t len;
  uint8_t incompat_flags;
  uint8_t compat_flags;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  uint32_t msgid;
  const uint8_t *payload;
  bool crc_ok;
} MavlinkFrame;

static const char *ardupilot_copter_mode_name(uint32_t custom_mode)
{
  // https://ardupilot.org/copter/docs/flight-modes.html (keep minimal, fallback to "MODE").
  switch (custom_mode) {
    case 0u:
      return "STABILIZE";
    case 1u:
      return "ACRO";
    case 2u:
      return "ALT_HOLD";
    case 3u:
      return "AUTO";
    case 4u:
      return "GUIDED";
    case 5u:
      return "LOITER";
    case 6u:
      return "RTL";
    case 9u:
      return "LAND";
    case 13u:
      return "SPORT";
    case 16u:
      return "POSHOLD";
    case 17u:
      return "BRAKE";
    default:
      return "MODE";
  }
}

static bool mav_type_is_copter(uint8_t mav_type)
{
  // Common Copter types.
  return (mav_type == 2u) || (mav_type == 3u) || (mav_type == 4u) || (mav_type == 13u) ||
         (mav_type == 14u) || (mav_type == 15u) || (mav_type == 16u) || (mav_type == 17u);
}

static osMutexId_t g_mavlink_mutex = NULL;
static osMessageQueueId_t g_mavlink_tx_queue = NULL;
static MavlinkTelemetry_t g_telemetry;
static uint8_t g_target_sysid = MAVLINK_TARGET_SYS_ID;
static uint8_t g_target_compid = MAVLINK_TARGET_COMP_ID;
static uint8_t g_local_sysid = MAVLINK_LOCAL_SYS_ID_FALLBACK;
static uint8_t g_tx_seq = 0u;
static bool g_link_up = false;
static MavlinkParser g_parser;

static volatile uint16_t g_rx_head = 0u;
static volatile uint16_t g_rx_tail = 0u;
static uint8_t g_rx_ring[MAVLINK_RX_RING_SIZE];
static uint8_t g_rx_it_byte = 0u;
static volatile uint32_t g_rx_bytes = 0u;
static volatile uint32_t g_rx_drops = 0u;
static volatile uint32_t g_rx_stx_v1 = 0u;
static volatile uint32_t g_rx_stx_v2 = 0u;
static volatile uint32_t g_frames_ok = 0u;
static volatile uint32_t g_crc_fail = 0u;
static volatile uint32_t g_unknown_crc_extra = 0u;
static volatile uint32_t g_last_crc_fail_msgid = 0u;
static volatile uint32_t g_last_unknown_msgid = 0u;
static volatile uint8_t g_last_unknown_len = 0u;
static volatile uint8_t g_last_unknown_sysid = 0u;
static volatile uint8_t g_last_unknown_compid = 0u;
static volatile uint32_t g_last_ok_msgid = 0u;
static volatile uint8_t g_last_ok_sysid = 0u;
static volatile uint8_t g_last_ok_compid = 0u;
static volatile uint8_t g_last_crc_fail_len = 0u;
static volatile uint8_t g_last_crc_fail_sysid = 0u;
static volatile uint8_t g_last_crc_fail_compid = 0u;
static volatile uint16_t g_last_hb_crc_expected = 0u;
static volatile uint16_t g_last_hb_crc_calculated = 0u;
static volatile uint8_t g_last_hb_incompat = 0u;

// Heartbeat sniffer statistics (MAVLink1 heartbeat only, for debugging/telemetry).
static volatile uint32_t g_hb_hdr = 0u;
static volatile uint32_t g_hb_ok = 0u;
static volatile uint32_t g_hb_bad = 0u;
static volatile uint32_t g_hb2_hdr = 0u;
static volatile uint32_t g_hb2_ok = 0u;
static volatile uint32_t g_hb2_bad = 0u;

static char g_dbg_line[320];
static char g_fc_line[160];

typedef struct {
  uint8_t buf[32];
  uint8_t idx;
  uint8_t expect;
  bool active;
} MavlinkHbSniffer;

typedef struct {
  uint8_t buf[48];
  uint8_t idx;
  uint8_t expect;
  bool active;
} MavlinkHbSnifferV2;

static void mavlink_parser_reset(MavlinkParser *parser)
{
  parser->magic = 0u;
  parser->index = 0u;
  parser->expected_len = 0u;
}

static void mavlink_crc_accumulate(uint8_t data, uint16_t *crc)
{
  // MAVLink uses X.25 CRC with 8-bit tmp (wrap-around matters).
  uint8_t tmp = (uint8_t)(data ^ (uint8_t)(*crc & 0xFFu));
  tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
  *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

static uint16_t mavlink_crc_calculate(const uint8_t *buffer, uint16_t length)
{
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < length; ++i) {
    mavlink_crc_accumulate(buffer[i], &crc);
  }
  return crc;
}

static uint8_t mavlink_get_crc_extra(uint32_t msgid)
{
  switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      return MAVLINK_CRC_EXTRA_HEARTBEAT;
    case MAVLINK_MSG_ID_SYS_STATUS:
      return MAVLINK_CRC_EXTRA_SYS_STATUS;
    case MAVLINK_MSG_ID_ATTITUDE:
      return MAVLINK_CRC_EXTRA_ATTITUDE;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      return MAVLINK_CRC_EXTRA_GLOBAL_POSITION_INT;
    case MAVLINK_MSG_ID_RC_CHANNELS:
      return MAVLINK_CRC_EXTRA_RC_CHANNELS;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      return MAVLINK_CRC_EXTRA_COMMAND_LONG;
    case MAVLINK_MSG_ID_COMMAND_ACK:
      return MAVLINK_CRC_EXTRA_COMMAND_ACK;
    case MAVLINK_MSG_ID_STATUSTEXT:
      return MAVLINK_CRC_EXTRA_STATUSTEXT;
    default:
      return 0u;
  }
}

static uint16_t mavlink_get_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t mavlink_get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int16_t mavlink_get_i16(const uint8_t *p)
{
  return (int16_t)mavlink_get_u16(p);
}

static int32_t mavlink_get_i32(const uint8_t *p)
{
  return (int32_t)mavlink_get_u32(p);
}

static float mavlink_get_f32(const uint8_t *p)
{
  float value;
  memcpy(&value, p, sizeof(value));
  return value;
}

static void mavlink_put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void mavlink_put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void mavlink_put_f32(uint8_t *p, float v)
{
  memcpy(p, &v, sizeof(v));
}

static bool mavlink_send_message(uint32_t msgid,
                                 const uint8_t *payload,
                                 uint8_t payload_len,
                                 uint8_t crc_extra)
{
  if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
    return false;
  }

  static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  // Transmit MAVLink2 (0xFD). ArduPilot uses MAVLink2 on the configured port and this ensures
  // proper routing/forwarding in typical setups.
  buffer[0] = MAVLINK_STX_V2;
  buffer[1] = payload_len;
  buffer[2] = 0u;
  buffer[3] = 0u;
  buffer[4] = g_tx_seq++;
  buffer[5] = g_local_sysid;
  buffer[6] = MAVLINK_LOCAL_COMP_ID;
  buffer[7] = (uint8_t)(msgid & 0xFFu);
  buffer[8] = (uint8_t)((msgid >> 8) & 0xFFu);
  buffer[9] = (uint8_t)((msgid >> 16) & 0xFFu);

  if (payload_len > 0u && payload != NULL) {
    memcpy(&buffer[10], payload, payload_len);
  }

  uint16_t crc = mavlink_crc_calculate(&buffer[1], (uint16_t)(9u + payload_len));
  mavlink_crc_accumulate(crc_extra, &crc);
  buffer[10 + payload_len] = (uint8_t)(crc & 0xFFu);
  buffer[11 + payload_len] = (uint8_t)((crc >> 8) & 0xFFu);

  uint16_t frame_len = (uint16_t)(12u + payload_len);
  return (HAL_UART_Transmit(MAVLINK_UART_HANDLE, buffer, frame_len, MAVLINK_TX_TIMEOUT_MS) ==
          HAL_OK);
}

static bool mavlink_send_heartbeat(void)
{
  uint8_t payload[9];
  mavlink_put_u32(&payload[0], 0u);
  payload[4] = MAVLINK_MAV_TYPE_GCS;
  payload[5] = MAVLINK_MAV_AUTOPILOT_INVALID;
  payload[6] = 0u;
  payload[7] = MAVLINK_MAV_STATE_ACTIVE;
  payload[8] = MAVLINK_VERSION;

  return mavlink_send_message(MAVLINK_MSG_ID_HEARTBEAT,
                              payload,
                              (uint8_t)sizeof(payload),
                              MAVLINK_CRC_EXTRA_HEARTBEAT);
}

static bool mavlink_send_command_long(const MavlinkCommandLongItem *item)
{
  uint8_t payload[33];
  mavlink_put_f32(&payload[0], item->params[0]);
  mavlink_put_f32(&payload[4], item->params[1]);
  mavlink_put_f32(&payload[8], item->params[2]);
  mavlink_put_f32(&payload[12], item->params[3]);
  mavlink_put_f32(&payload[16], item->params[4]);
  mavlink_put_f32(&payload[20], item->params[5]);
  mavlink_put_f32(&payload[24], item->params[6]);
  mavlink_put_u16(&payload[28], item->command);
  payload[30] = item->target_system;
  payload[31] = item->target_component;
  payload[32] = item->confirmation;

  return mavlink_send_message(MAVLINK_MSG_ID_COMMAND_LONG,
                              payload,
                              (uint8_t)sizeof(payload),
                              MAVLINK_CRC_EXTRA_COMMAND_LONG);
}

static void mavlink_store_telemetry(const MavlinkTelemetry_t *telemetry)
{
  if (g_mavlink_mutex == NULL) {
    return;
  }
  if (osMutexAcquire(g_mavlink_mutex, 0) != osOK) {
    return;
  }
  g_telemetry = *telemetry;
  osMutexRelease(g_mavlink_mutex);
}

static bool mavlink_parse_frame(MavlinkParser *parser, MavlinkFrame *frame)
{
  if (parser->magic == MAVLINK_STX_V1) {
    uint8_t payload_len = parser->buffer[0];
    if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
      return false;
    }
    uint16_t crc_offset = (uint16_t)(5u + payload_len);
    uint16_t crc_expected = (uint16_t)parser->buffer[crc_offset] |
                            ((uint16_t)parser->buffer[crc_offset + 1u] << 8);
    uint8_t crc_extra = mavlink_get_crc_extra(parser->buffer[4]);

    frame->magic = parser->magic;
    frame->len = payload_len;
    frame->incompat_flags = 0u;
    frame->compat_flags = 0u;
    frame->seq = parser->buffer[1];
    frame->sysid = parser->buffer[2];
    frame->compid = parser->buffer[3];
    frame->msgid = parser->buffer[4];
    frame->payload = &parser->buffer[5];
    frame->crc_ok = false;

    if (crc_extra == 0u) {
      g_unknown_crc_extra++;
      g_last_unknown_msgid = parser->buffer[4];
      g_last_unknown_len = payload_len;
      g_last_unknown_sysid = parser->buffer[2];
      g_last_unknown_compid = parser->buffer[3];
      return true; // still forward for debug/target detection
    }

    uint16_t crc = mavlink_crc_calculate(parser->buffer, (uint16_t)(5u + payload_len));
    mavlink_crc_accumulate(crc_extra, &crc);
    if (crc != crc_expected) {
      g_crc_fail++;
      g_last_crc_fail_msgid = parser->buffer[4];
      g_last_crc_fail_len = payload_len;
      g_last_crc_fail_sysid = parser->buffer[2];
      g_last_crc_fail_compid = parser->buffer[3];
      return false;
    }

    frame->crc_ok = true;
    return true;
  }

  if (parser->magic == MAVLINK_STX_V2) {
    uint8_t payload_len = parser->buffer[0];
    if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
      return false;
    }
    uint16_t crc_offset = (uint16_t)(9u + payload_len);
    uint16_t crc_expected = (uint16_t)parser->buffer[crc_offset] |
                            ((uint16_t)parser->buffer[crc_offset + 1u] << 8);
    uint32_t msgid = (uint32_t)parser->buffer[6] |
                     ((uint32_t)parser->buffer[7] << 8) |
                     ((uint32_t)parser->buffer[8] << 16);
    uint8_t crc_extra = mavlink_get_crc_extra(msgid);

    frame->magic = parser->magic;
    frame->len = payload_len;
    frame->incompat_flags = parser->buffer[1];
    frame->compat_flags = parser->buffer[2];
    frame->seq = parser->buffer[3];
    frame->sysid = parser->buffer[4];
    frame->compid = parser->buffer[5];
    frame->msgid = msgid;
    frame->payload = &parser->buffer[9];
    frame->crc_ok = false;

    if (crc_extra == 0u) {
      g_unknown_crc_extra++;
      g_last_unknown_msgid = msgid;
      g_last_unknown_len = payload_len;
      g_last_unknown_sysid = parser->buffer[4];
      g_last_unknown_compid = parser->buffer[5];
      return true; // still forward for debug/target detection
    }

    uint16_t crc = mavlink_crc_calculate(parser->buffer, (uint16_t)(9u + payload_len));
    mavlink_crc_accumulate(crc_extra, &crc);
    if (crc != crc_expected) {
      g_crc_fail++;
      g_last_crc_fail_msgid = msgid;
      g_last_crc_fail_len = payload_len;
      g_last_crc_fail_sysid = parser->buffer[4];
      g_last_crc_fail_compid = parser->buffer[5];
      if (msgid == MAVLINK_MSG_ID_HEARTBEAT && payload_len == 9u) {
        g_last_hb_crc_expected = crc_expected;
        g_last_hb_crc_calculated = crc;
        g_last_hb_incompat = parser->buffer[1];
      }
      return false;
    }

    frame->crc_ok = true;
    return true;
  }

  return false;
}

static bool mavlink_parse_byte(MavlinkParser *parser, uint8_t byte, MavlinkFrame *frame)
{
  if (parser->magic == 0u) {
    if (byte == MAVLINK_STX_V1 || byte == MAVLINK_STX_V2) {
      parser->magic = byte;
      parser->index = 0u;
      parser->expected_len = 0u;
    }
    return false;
  }

  if (parser->index >= MAVLINK_MAX_PACKET_LEN) {
    mavlink_parser_reset(parser);
    return false;
  }

  parser->buffer[parser->index++] = byte;

  if (parser->magic == MAVLINK_STX_V1) {
    if (parser->index == 1u) {
      uint8_t payload_len = parser->buffer[0];
      if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
        mavlink_parser_reset(parser);
        return false;
      }
      parser->expected_len = (uint16_t)(5u + payload_len + 2u);
    }
  } else if (parser->magic == MAVLINK_STX_V2) {
    if (parser->index == 1u) {
      uint8_t payload_len = parser->buffer[0];
      if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
        mavlink_parser_reset(parser);
        return false;
      }
    } else if (parser->index == 2u) {
      uint8_t payload_len = parser->buffer[0];
      uint8_t incompat_flags = parser->buffer[1];
      uint16_t signature_len = (incompat_flags & MAVLINK_IFLAG_SIGNED) ? 13u : 0u;
      parser->expected_len = (uint16_t)(9u + payload_len + 2u + signature_len);
    }
  }

  if (parser->expected_len != 0u && parser->index >= parser->expected_len) {
    bool ok = mavlink_parse_frame(parser, frame);
    mavlink_parser_reset(parser);
    return ok;
  }

  return false;
}

static void mavlink_hb_sniffer_reset(MavlinkHbSniffer *s)
{
  s->idx = 0u;
  s->expect = 0u;
  s->active = false;
}

static void mavlink_hb_sniffer_feed(MavlinkHbSniffer *s, uint8_t byte, uint32_t now_ms)
{
  if (!s->active) {
    if (byte == MAVLINK_STX_V1) {
      s->active = true;
      s->idx = 0u;
      s->buf[s->idx++] = byte;
      s->expect = 0u;
    }
    return;
  }

  if (s->idx >= sizeof(s->buf)) {
    mavlink_hb_sniffer_reset(s);
    return;
  }

  s->buf[s->idx++] = byte;

  // Once LEN is captured, decide if this is a heartbeat-sized MAVLink1 frame.
  if (s->idx == 2u) {
    uint8_t len = s->buf[1];
    if (len != 9u) {
      mavlink_hb_sniffer_reset(s);
      return;
    }
    s->expect = (uint8_t)(8u + len); // total frame length including STX
  }

  if (s->expect != 0u && s->idx >= s->expect) {
    // Header check: STX(0) LEN(1) SEQ(2) SYS(3) COMP(4) MSGID(5)
    if (s->buf[0] == MAVLINK_STX_V1 && s->buf[1] == 9u && s->buf[5] == 0u) {
      g_hb_hdr++;

      const uint8_t *frame_wo_stx = &s->buf[1];
      uint16_t crc_expected = (uint16_t)s->buf[15] | ((uint16_t)s->buf[16] << 8);
      uint16_t crc = mavlink_crc_calculate(frame_wo_stx, 14u); // LEN..payload (5+9=14)
      mavlink_crc_accumulate(MAVLINK_CRC_EXTRA_HEARTBEAT, &crc);
      if (crc == crc_expected) {
        g_hb_ok++;

        // Decode heartbeat payload (starts at offset 6).
        const uint8_t *pl = &s->buf[6];
        MavlinkTelemetry_t telemetry = g_telemetry;
        telemetry.sysid = s->buf[3];
        telemetry.compid = s->buf[4];
        telemetry.custom_mode = mavlink_get_u32(&pl[0]);
        telemetry.type = pl[4];
        telemetry.autopilot = pl[5];
        telemetry.base_mode = pl[6];
        telemetry.system_status = pl[7];
        telemetry.mavlink_version = pl[8];
        telemetry.last_heartbeat_ms = now_ms;
        mavlink_store_telemetry(&telemetry);

        g_target_sysid = telemetry.sysid;
        g_target_compid = telemetry.compid;
        g_link_up = true;
      } else {
        g_hb_bad++;
      }
    }

    mavlink_hb_sniffer_reset(s);
  }
}

static void mavlink_hb_sniffer_v2_reset(MavlinkHbSnifferV2 *s)
{
  s->idx = 0u;
  s->expect = 0u;
  s->active = false;
}

static void mavlink_hb_sniffer_v2_feed(MavlinkHbSnifferV2 *s, uint8_t byte, uint32_t now_ms)
{
  if (!s->active) {
    if (byte == MAVLINK_STX_V2) {
      s->active = true;
      s->idx = 0u;
      s->buf[s->idx++] = byte;
      s->expect = 0u;
    }
    return;
  }

  if (s->idx >= sizeof(s->buf)) {
    mavlink_hb_sniffer_v2_reset(s);
    return;
  }

  s->buf[s->idx++] = byte;

  // MAVLink2 heartbeat: STX LEN(=9) INC COMPAT SEQ SYS COMP MSGID[3] PAYLOAD(9) CRC[2] SIG[0/13]
  if (s->idx == 2u) {
    if (s->buf[1] != 9u) {
      mavlink_hb_sniffer_v2_reset(s);
      return;
    }
  } else if (s->idx == 3u) {
    uint8_t incompat = s->buf[2];
    uint8_t sig_len = (incompat & MAVLINK_IFLAG_SIGNED) ? 13u : 0u;
    s->expect = (uint8_t)(10u + 9u + 2u + sig_len);
  }

  if (s->expect != 0u && s->idx >= s->expect) {
    // Heartbeat msgid == 0
    if (s->buf[1] == 9u && s->buf[7] == 0u && s->buf[8] == 0u && s->buf[9] == 0u) {
      g_hb2_hdr++;

      uint16_t crc_expected = (uint16_t)s->buf[19] | ((uint16_t)s->buf[20] << 8);
      uint16_t crc = mavlink_crc_calculate(&s->buf[1], 18u); // LEN..payload (9 header bytes + 9 payload bytes)
      mavlink_crc_accumulate(MAVLINK_CRC_EXTRA_HEARTBEAT, &crc);
      if (crc == crc_expected) {
        g_hb2_ok++;

        const uint8_t *pl = &s->buf[10];
        MavlinkTelemetry_t telemetry = g_telemetry;
        telemetry.sysid = s->buf[5];
        telemetry.compid = s->buf[6];
        telemetry.custom_mode = mavlink_get_u32(&pl[0]);
        telemetry.type = pl[4];
        telemetry.autopilot = pl[5];
        telemetry.base_mode = pl[6];
        telemetry.system_status = pl[7];
        telemetry.mavlink_version = pl[8];
        telemetry.last_heartbeat_ms = now_ms;
        mavlink_store_telemetry(&telemetry);

        g_target_sysid = telemetry.sysid;
        g_target_compid = telemetry.compid;
        g_local_sysid = telemetry.sysid; // mirror vehicle SYSID (companion style)
        g_link_up = true;
      } else {
        g_hb2_bad++;
      }
    }

    mavlink_hb_sniffer_v2_reset(s);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance != USART1) {
    return;
  }

  uint16_t head = g_rx_head;
  uint16_t next = (uint16_t)((head + 1u) & MAVLINK_RX_RING_MASK);
  if (next != g_rx_tail) {
    g_rx_ring[head] = g_rx_it_byte;
    g_rx_head = next;
  } else {
    g_rx_drops++;
  }
  g_rx_bytes++;

  (void)HAL_UART_Receive_IT(huart, &g_rx_it_byte, 1u);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance != USART1) {
    return;
  }

  // Recover from potential overrun and keep RX running.
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)HAL_UART_Receive_IT(huart, &g_rx_it_byte, 1u);
}

static void mavlink_handle_frame(const MavlinkFrame *frame, uint32_t now_ms)
{
  // Keep target IDs updated even if we can't validate CRC for some dialect-specific messages.
  if (frame->sysid != 0u && frame->sysid != g_local_sysid) {
    g_target_sysid = frame->sysid;
    g_target_compid = frame->compid;
  }

  if (MAVLINK_DEBUG_MSGID_ASCII != 0u) {
    const char *tag = NULL;
    if (frame->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      tag = "[HB]\r\n";
    } else if (frame->msgid == MAVLINK_MSG_ID_SYS_STATUS) {
      tag = "[SYS]\r\n";
    } else if (frame->msgid == MAVLINK_MSG_ID_ATTITUDE) {
      tag = "[ATT]\r\n";
    } else if (frame->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
      tag = "[GPS]\r\n";
    } else if (frame->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      tag = "[ACK]\r\n";
    } else if (frame->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
      tag = "[STAT]\r\n";
    } else if (!frame->crc_ok && MAVLINK_DEBUG_PRINT_UNKNOWN_FRAMES != 0u) {
      tag = "[UNK]\r\n";
    }

    if (tag != NULL) {
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)tag, (uint16_t)strlen(tag), MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);
    }
  }

  if (!frame->crc_ok) {
    return;
  }

  if (frame->msgid == MAVLINK_MSG_ID_HEARTBEAT && frame->len >= 9u) {
    MavlinkTelemetry_t telemetry = g_telemetry;
    telemetry.sysid = frame->sysid;
    telemetry.compid = frame->compid;
    telemetry.custom_mode = mavlink_get_u32(&frame->payload[0]);
    telemetry.type = frame->payload[4];
    telemetry.autopilot = frame->payload[5];
    telemetry.base_mode = frame->payload[6];
    telemetry.system_status = frame->payload[7];
    telemetry.mavlink_version = frame->payload[8];
    telemetry.last_heartbeat_ms = now_ms;
    mavlink_store_telemetry(&telemetry);

    g_target_sysid = frame->sysid;
    g_target_compid = frame->compid;
    g_link_up = true;
    return;
  }

  if (frame->msgid == MAVLINK_MSG_ID_SYS_STATUS && frame->len >= 31u) {
    MavlinkTelemetry_t telemetry = g_telemetry;
    telemetry.battery_voltage_mv = mavlink_get_u16(&frame->payload[14]);
    telemetry.battery_current_ca = mavlink_get_i16(&frame->payload[16]);
    telemetry.battery_remaining = (int8_t)frame->payload[18];
    telemetry.last_sys_status_ms = now_ms;
    mavlink_store_telemetry(&telemetry);
    return;
  }

  if (frame->msgid == MAVLINK_MSG_ID_ATTITUDE && frame->len >= 28u) {
    MavlinkTelemetry_t telemetry = g_telemetry;
    telemetry.roll = mavlink_get_f32(&frame->payload[4]);
    telemetry.pitch = mavlink_get_f32(&frame->payload[8]);
    telemetry.yaw = mavlink_get_f32(&frame->payload[12]);
    telemetry.rollspeed = mavlink_get_f32(&frame->payload[16]);
    telemetry.pitchspeed = mavlink_get_f32(&frame->payload[20]);
    telemetry.yawspeed = mavlink_get_f32(&frame->payload[24]);
    telemetry.last_attitude_ms = now_ms;
    mavlink_store_telemetry(&telemetry);
    return;
  }

  if (frame->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && frame->len >= 28u) {
    MavlinkTelemetry_t telemetry = g_telemetry;
    telemetry.lat = mavlink_get_i32(&frame->payload[4]);
    telemetry.lon = mavlink_get_i32(&frame->payload[8]);
    telemetry.alt_mm = mavlink_get_i32(&frame->payload[12]);
    telemetry.relative_alt_mm = mavlink_get_i32(&frame->payload[16]);
    telemetry.vx = mavlink_get_i16(&frame->payload[20]);
    telemetry.vy = mavlink_get_i16(&frame->payload[22]);
    telemetry.vz = mavlink_get_i16(&frame->payload[24]);
    telemetry.heading_cdeg = mavlink_get_u16(&frame->payload[26]);
    telemetry.last_position_ms = now_ms;
    mavlink_store_telemetry(&telemetry);
    return;
  }

  if (frame->msgid == MAVLINK_MSG_ID_COMMAND_ACK && frame->len >= 3u) {
    MavlinkTelemetry_t telemetry = g_telemetry;
    telemetry.last_command_ack = mavlink_get_u16(&frame->payload[0]);
    telemetry.last_command_result = frame->payload[2];
    telemetry.last_command_ack_ms = now_ms;
    mavlink_store_telemetry(&telemetry);
    return;
  }
}

static void mavlink_task_init(void)
{
  if (g_mavlink_mutex == NULL) {
    osMutexAttr_t attr = { .name = "mavlinkMutex" };
    g_mavlink_mutex = osMutexNew(&attr);
  }

  if (g_mavlink_tx_queue == NULL) {
    osMessageQueueAttr_t attr = { .name = "mavlinkTxQueue" };
    g_mavlink_tx_queue = osMessageQueueNew(8u, sizeof(MavlinkCommandLongItem), &attr);
  }

  memset(&g_telemetry, 0, sizeof(g_telemetry));

  const char *banner = "MAVTASK\r\n";
  (void)HAL_UART_Transmit(&huart3,
                          (uint8_t *)banner,
                          (uint16_t)strlen(banner),
                          MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);

  // Start interrupt-driven RX so we don't drop bytes during TX or RTOS delays.
  (void)HAL_UART_Receive_IT(MAVLINK_UART_HANDLE, &g_rx_it_byte, 1u);
}

void MavlinkTask_Run(void)
{
  mavlink_task_init();

  mavlink_parser_reset(&g_parser);
  MavlinkHbSniffer hb;
  mavlink_hb_sniffer_reset(&hb);
  MavlinkHbSnifferV2 hb2;
  mavlink_hb_sniffer_v2_reset(&hb2);

  uint32_t last_heartbeat_tx = 0u;
  uint32_t last_statustext_tx = 0u;
  uint32_t last_rx_watchdog = 0u;
  uint32_t last_rx_bytes = 0u;
  uint32_t last_fc_status = 0u;
  bool requested_rates = false;

  for (;;) {
    uint32_t now = osKernelGetTickCount();
    if ((now - last_heartbeat_tx) >= MAVLINK_HEARTBEAT_PERIOD_MS) {
      (void)mavlink_send_heartbeat();
      last_heartbeat_tx = now;
    }

    if (g_link_up && !requested_rates) {
      (void)Mavlink_RequestMessageInterval(MAVLINK_MSG_ID_SYS_STATUS,
                                           MAVLINK_RATE_SYS_STATUS_US);
      (void)Mavlink_RequestMessageInterval(MAVLINK_MSG_ID_ATTITUDE,
                                           MAVLINK_RATE_ATTITUDE_US);
      (void)Mavlink_RequestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                           MAVLINK_RATE_GLOBAL_POSITION_US);
      requested_rates = true;
    }

    if ((now - last_statustext_tx) >= MAVLINK_STATUSTEXT_PERIOD_MS) {
      (void)Mavlink_SendStatustext("MCU link OK", MAVLINK_SEVERITY_INFO);
      last_statustext_tx = now;
    }

    if (g_mavlink_tx_queue != NULL) {
      MavlinkCommandLongItem item;
      while (osMessageQueueGet(g_mavlink_tx_queue, &item, NULL, 0u) == osOK) {
        (void)mavlink_send_command_long(&item);
      }
    }

    while (g_rx_tail != g_rx_head) {
      uint8_t byte = g_rx_ring[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1u) & MAVLINK_RX_RING_MASK);

      mavlink_hb_sniffer_feed(&hb, byte, now);
      mavlink_hb_sniffer_v2_feed(&hb2, byte, now);

      if (byte == MAVLINK_STX_V1) {
        g_rx_stx_v1++;
      } else if (byte == MAVLINK_STX_V2) {
        g_rx_stx_v2++;
      }

      if (MAVLINK_DEBUG_BRIDGE != 0u) {
        (void)HAL_UART_Transmit(&huart3, &byte, 1u, MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);
      }

      MavlinkFrame frame;
      if (mavlink_parse_byte(&g_parser, byte, &frame)) {
        g_frames_ok++;
        if (frame.crc_ok) {
          g_last_ok_msgid = frame.msgid;
          g_last_ok_sysid = frame.sysid;
          g_last_ok_compid = frame.compid;
        }
        mavlink_handle_frame(&frame, now);
      }
    }

    // Simple RX watchdog so we know if bytes are arriving at all.
    if ((now - last_rx_watchdog) >= 1000u) {
      uint32_t rx = g_rx_bytes;
      uint32_t stx1 = g_rx_stx_v1;
      uint32_t stx2 = g_rx_stx_v2;
      uint32_t ok = g_frames_ok;
      uint32_t crc = g_crc_fail;
      uint32_t unk = g_unknown_crc_extra;
      uint32_t bad_id = g_last_crc_fail_msgid;
      uint32_t bad_len = g_last_crc_fail_len;
      uint32_t bad_sys = g_last_crc_fail_sysid;
      uint32_t bad_comp = g_last_crc_fail_compid;
      uint32_t unk_id = g_last_unknown_msgid;
      uint32_t unk_len = g_last_unknown_len;
      uint32_t unk_sys = g_last_unknown_sysid;
      uint32_t unk_comp = g_last_unknown_compid;
      uint32_t ok_id = g_last_ok_msgid;
      uint32_t ok_sys = g_last_ok_sysid;
      uint32_t ok_comp = g_last_ok_compid;
      uint32_t drop = g_rx_drops;
      uint32_t hb_ok = g_hb_ok;
      uint32_t hb_bad = g_hb_bad;
      uint32_t hb_hdr = g_hb_hdr;
      uint32_t hb2_ok = g_hb2_ok;
      uint32_t hb2_bad = g_hb2_bad;
      uint32_t hb2_hdr = g_hb2_hdr;

      uint32_t rx_delta = rx - last_rx_bytes;
      static uint32_t last_stx1 = 0u;
      static uint32_t last_stx2 = 0u;
      static uint32_t last_ok = 0u;
      static uint32_t last_crc = 0u;
      static uint32_t last_unk = 0u;
      static uint32_t last_drop = 0u;
      static uint32_t last_hb_ok = 0u;
      static uint32_t last_hb_bad = 0u;
      static uint32_t last_hb_hdr = 0u;
      static uint32_t last_hb2_ok = 0u;
      static uint32_t last_hb2_bad = 0u;
      static uint32_t last_hb2_hdr = 0u;

      uint32_t stx1_delta = stx1 - last_stx1;
      uint32_t stx2_delta = stx2 - last_stx2;
      uint32_t ok_delta = ok - last_ok;
      uint32_t crc_delta = crc - last_crc;
      uint32_t unk_delta = unk - last_unk;
      uint32_t drop_delta = drop - last_drop;
      uint32_t hb2_ok_delta = hb2_ok - last_hb2_ok;
      uint32_t hb2_bad_delta = hb2_bad - last_hb2_bad;
      uint32_t hb2_hdr_delta = hb2_hdr - last_hb2_hdr;

      last_stx1 = stx1;
      last_stx2 = stx2;
      last_ok = ok;
      last_crc = crc;
      last_unk = unk;
      last_drop = drop;
      last_hb_ok = hb_ok;
      last_hb_bad = hb_bad;
      last_hb_hdr = hb_hdr;
      last_hb2_ok = hb2_ok;
      last_hb2_bad = hb2_bad;
      last_hb2_hdr = hb2_hdr;

      int n = 0;
      if (rx_delta == 0u) {
        n = snprintf(g_dbg_line,
                     sizeof(g_dbg_line),
                     "[NO RX] drop+%lu\r\n",
                     (unsigned long)drop_delta);
      } else {
        n = snprintf(g_dbg_line,
                     sizeof(g_dbg_line),
                     "[RX] b+%lu stx1+%lu stx2+%lu ok+%lu crcf+%lu(id:%lu len:%lu s:%lu c:%lu) unk+%lu(id:%lu len:%lu s:%lu c:%lu) HB2 hdr+%lu ok+%lu bad+%lu hbcrc exp:%04X calc:%04X inc:%02X drop+%lu\r\n",
                     (unsigned long)rx_delta,
                     (unsigned long)stx1_delta,
                     (unsigned long)stx2_delta,
                     (unsigned long)ok_delta,
                     (unsigned long)crc_delta,
                     (unsigned long)bad_id,
                     (unsigned long)bad_len,
                     (unsigned long)bad_sys,
                     (unsigned long)bad_comp,
                     (unsigned long)unk_delta,
                     (unsigned long)unk_id,
                     (unsigned long)unk_len,
                     (unsigned long)unk_sys,
                     (unsigned long)unk_comp,
                     (unsigned long)hb2_hdr_delta,
                     (unsigned long)hb2_ok_delta,
                     (unsigned long)hb2_bad_delta,
                     (unsigned)g_last_hb_crc_expected,
                     (unsigned)g_last_hb_crc_calculated,
                     (unsigned)g_last_hb_incompat,
                     (unsigned long)drop_delta);
      }
      if (n > 0) {
        // Ensure line ends with CRLF even if snprintf truncates.
        size_t L = strlen(g_dbg_line);
        if (L < 2u || g_dbg_line[L - 1u] != '\n') {
          const char *crlf = "\r\n";
          (void)crlf;
          if (L + 2u < sizeof(g_dbg_line)) {
            g_dbg_line[L] = '\r';
            g_dbg_line[L + 1u] = '\n';
            g_dbg_line[L + 2u] = '\0';
          } else {
            // Force-terminate with CRLF in last two positions.
            g_dbg_line[sizeof(g_dbg_line) - 3u] = '\r';
            g_dbg_line[sizeof(g_dbg_line) - 2u] = '\n';
            g_dbg_line[sizeof(g_dbg_line) - 1u] = '\0';
          }
        }
        (void)HAL_UART_Transmit(&huart3,
                                (uint8_t *)g_dbg_line,
                                (uint16_t)strlen(g_dbg_line),
                                MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);
      }
      last_rx_bytes = rx;
      last_rx_watchdog = now;
    }

    // Periodic flight controller status on USART3.
    if ((now - last_fc_status) >= 1000u) {
      MavlinkTelemetry_t t;
      if (Mavlink_GetTelemetry(&t) && t.last_heartbeat_ms != 0u) {
        bool armed = ((t.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0u);
        const char *mode_name = "MODE";
        if (t.autopilot == 3u && mav_type_is_copter(t.type)) {
          mode_name = ardupilot_copter_mode_name(t.custom_mode);
        }

        (void)snprintf(g_fc_line,
                       sizeof(g_fc_line),
                       "FC sys:%u comp:%u %s base:0x%02X cmode:%lu (%s)\r\n",
                       (unsigned)t.sysid,
                       (unsigned)t.compid,
                       armed ? "ARMED" : "DISARM",
                       (unsigned)t.base_mode,
                       (unsigned long)t.custom_mode,
                       mode_name);
        (void)HAL_UART_Transmit(&huart3,
                                (uint8_t *)g_fc_line,
                                (uint16_t)strlen(g_fc_line),
                                MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);
      } else {
        const char *s = "FC: no heartbeat\r\n";
        (void)HAL_UART_Transmit(&huart3,
                                (uint8_t *)s,
                                (uint16_t)strlen(s),
                                MAVLINK_DEBUG_BRIDGE_TIMEOUT_MS);
      }
      last_fc_status = now;
    }

    osDelay(1u);
  }
}

bool Mavlink_GetTelemetry(MavlinkTelemetry_t *out)
{
  if (out == NULL || g_mavlink_mutex == NULL) {
    return false;
  }
  if (osMutexAcquire(g_mavlink_mutex, 0u) != osOK) {
    return false;
  }
  *out = g_telemetry;
  osMutexRelease(g_mavlink_mutex);
  return true;
}

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
                             uint8_t confirmation)
{
  if (g_mavlink_tx_queue == NULL) {
    return false;
  }

  MavlinkCommandLongItem item;
  item.command = command;
  item.params[0] = param1;
  item.params[1] = param2;
  item.params[2] = param3;
  item.params[3] = param4;
  item.params[4] = param5;
  item.params[5] = param6;
  item.params[6] = param7;
  item.target_system = target_system;
  item.target_component = target_component;
  item.confirmation = confirmation;

  return (osMessageQueuePut(g_mavlink_tx_queue, &item, 0u, 0u) == osOK);
}

bool Mavlink_RequestMessageInterval(uint32_t message_id, uint32_t interval_us)
{
  float msg_id = (float)message_id;
  float interval = (float)interval_us;
  return Mavlink_SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL,
                                 msg_id,
                                 interval,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 g_target_sysid,
                                 g_target_compid,
                                 0u);
}

bool Mavlink_SendStatustext(const char *text, uint8_t severity)
{
  if (text == NULL) {
    return false;
  }

  uint8_t payload[51];
  payload[0] = severity;
  memset(&payload[1], 0, 50u);

  size_t len = strlen(text);
  if (len > 50u) {
    len = 50u;
  }
  memcpy(&payload[1], text, len);

  return mavlink_send_message(MAVLINK_MSG_ID_STATUSTEXT,
                              payload,
                              (uint8_t)sizeof(payload),
                              MAVLINK_CRC_EXTRA_STATUSTEXT);
}

uint8_t Mavlink_GetTargetSystem(void)
{
  return g_target_sysid;
}

uint8_t Mavlink_GetTargetComponent(void)
{
  return g_target_compid;
}
