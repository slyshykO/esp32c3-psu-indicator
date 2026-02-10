/*
https://endless-sphere.com/sphere/threads/emerson-vertiv-r48-series-can-programming.114785/post-1875324

As you did help me i will share my code. You can find a lot of new information there.

Intresting is specially the format of the 29-bit Message idenfifier, the structure of the data, status bits and possible some new valuetypes...

For getting my code running you will have make some few changes as you will miss some libraries... It should support multiple devices on the same can bus.

It written for espressif 5.5
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <soc/gpio_num.h>

#define R48_MAX_DEVICE_COUNT 2
#define R48_TEMPERATURE_OFFSET 6.2f  // Gemessene Temperatur +5°C (Kalibrierung)
#define R48_MIN_STATUS_UPDATE_INTERVAL_MS 2000UL // Minimaler Abstand zwischen Status-Updates

#define R48_RATED_CURRENT_A (62.5f)         
#define R48_MIN_VOLTAGE_V   (41.0f)
#define R48_MAX_VOLTAGE_V   (58.0f)

#define R48_DEFAULT_OFFLINE_CURRENT_A (0.0f) // Default offline current to set on startup if stored value not found
#define R48_DEFAULT_OFFLINE_VOLTAGE_V (48.0f) // Default offline voltage to set on startup if stored value not found



//Allowed range 0xF0..0xF8
//This is a special internal address that is coded into the CAN IDs for the R48 protocol
#define R48_THIS_DEVICE_ADDRESS (0xF0) // Address of this controller
#define R48_BROADCAST_ADDRESS   (0xFF) // Broadcast address for all devices

//Type definitions
#define R48_DATA_ERRTYPE_NONE          0xF0
#define R48_DATA_ERRTYPE_INVALID_ADR   0xF1
#define R48_DATA_ERRTYPE_INVALID_CMD   0xF2
#define R48_DATA_ERRTYPE_INVALID_DATA  0xF3
#define R48_DATA_ERRTYPE_ADR_IDENT_RUNNING 0xF4


//MSGTYPE definitions
#define R48_MSGTYPE_REQUEST_ALL 0x00

#define R48_MSGTYPE_REQUEST_BYTE_DATA 0x01
#define R48_MSGTYPE_RESPONSE_BYTE_DATA 0x41

#define R48_MSGTYPE_REQUEST_BIT_DATA 0x02
#define R48_MSGTYPE_RESPONSE_BIT_DATA 0x42

#define R48_MSGTYPE_WRITE_DATA 0x03
#define R48_MSGTYPE_WRITE_DATA_ACK 0x43

#define R48_PROTO_MODULE_CONTROLLER_COMM 0x060

#define R48_IOUT_OFFLINE_NVS_KEY "r48IoutOff"
#define R48_VOUT_OFFLINE_NVS_KEY "r48VoutOff"

#ifdef __cplusplus
extern "C" {
#endif

// Address of a R48 module
typedef uint8_t r48_module_adr_t;

typedef struct r48_can_id_t
{
    //Adress is in this format:
    /*
  
    • Bits 28-20: Protokollnummer (PROTNO, 9 bits)
    • Bit 19: PTP (Point-to-Point Indikator)
    • Bits 18-11: Zieladresse (DSTADDR, 8 bits)
    • Bits 10-3: Quelladresse (SRCADDR, 8 bits)
    • Bits 2-0: CNT, RES1, RES2 (Zähler / Reserviert)
        */
    union
    {
        struct
        {
            uint32_t direct;
        } u32;
        struct
        {
            uint32_t res2:1; //allways 1
            uint32_t res1:1; //allways 1
            uint32_t cnt:1; //0 if last frame from this source, 1 if more frames are comming
            uint32_t src_addr:8; // Source Address range 0x00-0xFE 0x00-0x7f are from modules, 0xF0-0xF8 for master controllers
            uint32_t dst_addr:8; //Same as src_addr
            uint32_t ptp:1; // Point-to-Point indicator if broadcast 0 than dest_addr is 0xFF
            uint32_t proto:9; // allways 0x060 for R48
        } bits;
    };
} r48_can_id_t;

typedef enum r48_valuetype_e
{
    R48_VALUETYPE_VOUT = 0x01,
    R48_VALUETYPE_IOUT = 0x02,
    R48_VALUETYPE_ILIMIT = 0x03,
    R48_VALUETYPE_TEMPERATURE = 0x04,
    R48_VALUETYPE_VIN = 0x05,
    R48_VALUETYPE_INVALID = 0x06,
    R48_VALUETYPE_DISPLAY_CURRENT = 0x07,
    R48_VALUETYPE_ROOM_TEMPERATURE = 0x0B,
    R48_VALUETYPE_IOUT_OFFLINE = 0x19,
    R48_VALUETYPE_AC_INPUT_ILIMIT = 0x1A, // "Diesel power limit" = AC input current limit
    R48_VALUETYPE_VOUT_ONLINE = 0x21,
    R48_VALUETYPE_IOUT_ONLINE = 0x22,
    R48_VALUETYPE_VOUT_OFFLINE = 0x24,
    R48_VALUETYPE_WALKIN_TIME = 0x29, // Ramp-up time (sec)
    R48_VALUETYPE_REMOTE_OFF = 0x30,
    R48_VALUETYPE_WALKIN_ENABLE        = 0x32, // Walk-in on/off
    R48_VALUETYPE_FAN_CONTROL = 0x33, // 0x00 = auto, 0x01 = full speed
    R48_VALUETYPE_TURN_OFF_MODULE = 0x35,
    R48_VALUETYPE_RESTART_AFTER_OV     = 0x39, // Restart after overvoltage (on/off)
    R48_VALUETYPE_BIT_STATUS = 0x40,
    R48_VALUETYPE_RESERVED_1 = 0x54,
    R48_VALUETYPE_RESERVED_2 = 0x58,
    R48_VALUETYPE_ALL = 0x80,
} r48_valuetype_e;

typedef struct r48_data_t
{
    uint8_t msgtype:7;
    uint8_t err:1;
    uint8_t errType;
    union
    {
        uint8_t all_bytes[6];
        struct {
            uint8_t valuetype0_allways0; // At least i think it is always 0x00
            uint8_t valuetype1; // r48_get_request_e
            uint8_t content[4];
        } byte_wise;
    } data;
} r48_data_t;

typedef struct r48_device_mem_t
{
    float v_out_set;      // Gesetzte Ausgangsspannung
    int64_t v_out_set_timestamp;

    float i_out_set;      // Gesetzter Ausgangsstrom
    int64_t i_out_set_timestamp;

    //Gemessene Werte
    float v_out;      // Ausgangsspannung
    int64_t v_out_timestamp;

    float i_out;      // Ausgangsstrom
    int64_t i_out_timestamp;

    float temp_C;     // Temperatur in °C
    float room_temp_C;     // Temperatur in °C
    float v_in;       // Eingangsspannung

    float i_limit;   // Wrong data

    float v_out_offline; // Letzte gesetzte offline Spannung
    float i_out_offline; // Letzter gesetzter offline Strom

    union {
        struct {
            uint8_t byte3;
            uint8_t byte2;
            uint8_t byte1;
            uint8_t byte0;
        };
        struct {
            //Data Byte 4
            uint8_t temp_limited_power:1;
            uint8_t ac_power_limitation:1;
            uint8_t eeprom_module_error:1;
            uint8_t fan_failure:1;
            uint8_t module_protection:1;
            uint8_t module_fault:1;
            uint8_t overtemperature:1;
            uint8_t overvoltage:1;

            //Data Byte 5
            uint8_t can_error_status:1;
            uint8_t module_power_balance_error:1;
            uint8_t module_identification:1;
            uint8_t overvoltage_shutdown_relay:1;
            uint8_t walk_in_function:1;
            uint8_t fan_full_speed:1;
            uint8_t module_switch:1;
            uint8_t module_power_limit:1;

            //Data Byte 6
            uint8_t module_pfc_error:1;
            uint8_t module_ac_overvoltage:1;
            uint8_t duplicate_module_id:1;
            uint8_t severe_current_imbalance:1;
            uint8_t comm_phase_loss:1;
            uint8_t comm_imbalance:1;
            uint8_t module_ac_undervoltage_alarm:1;
            uint8_t sequential_start_function:1;

            //Data Byte 7
            uint8_t reserved0:1;
            uint8_t reserved1:1;
            uint8_t fuse_fault_output:1;
            uint8_t internal_comm_error:1;
            uint8_t undervoltage_output:1;
            uint8_t overload_alarm:1;
            uint8_t pnp_alarm:1;
            uint8_t slight_current_imbalance:1;
        } bits;
        uint8_t bytes[4];
        uint32_t u32;;
    } bit_status; // Status-Bits

    bool remote_off; // Ob das Gerät per Remote-Off abgeschaltet ist

    // int64_t last_online_update; // TickCount des letzten Updates
    // int64_t last_request_time;   // Zeit des letzten Requests
} r48_device_mem_t;


typedef struct r48_device_mem_t* r48_handle_t;

typedef struct r48_mem_t
{
    int64_t last_status_broadcast_req_time;
    r48_handle_t devices[R48_MAX_DEVICE_COUNT];
} r48_mem_t;

extern r48_mem_t r48_mem;

bool r48_init(gpio_num_t tx_gpio, gpio_num_t rx_gpio);

void r48_worker(void);

bool r48_set_current_offline(r48_module_adr_t module_adr, float amps);
bool r48_set_current_online(r48_module_adr_t module_adr, float amps);
bool r48_set_voltage_offline(r48_module_adr_t module_adr, float volts);
bool r48_set_voltage_online(r48_module_adr_t module_adr, float volts);

bool r48_fan_set_fullspeed(r48_module_adr_t module_adr);
bool r48_fan_set_auto(r48_module_adr_t module_adr);

bool r48_remote_on(r48_module_adr_t module_adr);
bool r48_remote_off(r48_module_adr_t module_adr);

bool r48_turn_on_off_module(bool on);



bool r48_send_status_request(r48_module_adr_t dest_adr, r48_valuetype_e valuetype);


#ifdef __cplusplus
}
#endif