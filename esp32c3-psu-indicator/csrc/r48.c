#include "R48.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>
#include <string.h>
#include <math.h>
#include <driver/twai.h>   // legacy TWAI API (OK in IDF 5.5)
#include <esp_check.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <bareco_database.h>

_Static_assert(sizeof(r48_data_t) == 8, "r48_data_t must be exactly 8 bytes");
_Static_assert(sizeof(((r48_device_mem_t*)0)->bit_status) == 4, "bit_status must be 4 bytes");

const static char TAG[] = "R48";

r48_mem_t r48_mem = {0};

static inline float be_to_float(const uint8_t in[4]);
static inline float fraction_to_amps(float frac);

static void r48_log_frame(const twai_message_t *msg)
{
    char buf[3 * 8 + 1] = {0};
    int n = 0;

    for(int i = 0; i < msg->data_length_code && i < 8; ++i)
    {
        n += snprintf(&buf[n], sizeof(buf) - n, "%02X ", msg->data[i]);
    }

    ESP_LOGI(TAG,
                    "RX %s ID=0x%08X DLC=%d%s%s data=[%s]",
                    msg->extd ? "EXT" : "STD",
                    msg->identifier,
                    msg->data_length_code,
                    msg->rtr ? " RTR" : "",
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
                    msg->self ? " SELF" : "",
#else
                    "",
#endif
                    buf);
}

void r48_log_msg_full_details(const twai_message_t *msg)
{
    ESP_LOGI(TAG, "R48 Message Details:");
    const r48_can_id_t *can_id = (const r48_can_id_t *)&msg->identifier;
    ESP_LOGI(TAG, "  Protocol: 0x%03X", can_id->bits.proto);
    ESP_LOGI(TAG, "  PTP: %u", can_id->bits.ptp);
    ESP_LOGI(TAG, "  Dest Addr: 0x%02X", can_id->bits.dst_addr);
    ESP_LOGI(TAG, "  Src Addr: 0x%02X", can_id->bits.src_addr);
    ESP_LOGI(TAG, "  CNT: %u", can_id->bits.cnt);
    if(!can_id->bits.res1 || !can_id->bits.res2)
    {
        ESP_LOGW(TAG, "  Warning: RES1 or RES2 bits are not set to 1 as expected!");
    }
    const r48_data_t *data = (const r48_data_t *)msg->data;
    ESP_LOGI(TAG, "  Message Type: 0x%02X", data->msgtype);
    ESP_LOGI(TAG, "  Error Flag: %u", data->err);
    ESP_LOGI(TAG, "  Error Type: 0x%02X", data->errType);
    char buffer[4 * 6 + 1] = {0};
    for(int i = 0; i < 4; i++)
    {
        sprintf(&buffer[i * 5], "0x%02X ", data->data.byte_wise.content[i]);
    }
    ESP_LOGI(TAG, "  Content Bytes: %s", buffer);
    ESP_LOGI(TAG, "  Content as Float: %.3f", be_to_float(data->data.byte_wise.content));
}

static void print_all_status_flag_set(r48_module_adr_t addr)
{
    r48_handle_t handle = r48_mem.devices[addr];
    if(handle == NULL || addr >= R48_MAX_DEVICE_COUNT)
    {
        ESP_LOGW(TAG, "print_all_status_flag_set: No handle for device index 0x%02X", addr);
        return;
    }
    const char *flags_names[32] = {
        "TEMP_LIMITED_POWER",
        "AC_POWER_LIMITATION",
        "EEPROM_MODULE_ERROR",
        "FAN_FAILURE",
        "MODULE_PROTECTION",
        "MODULE_FAULT",
        "OVERTEMPERATURE",
        "OVERVOLTAGE",
        "CAN_ERROR_STATUS",
        "MODULE_POWER_BALANCE_ERROR",
        "MODULE_IDENTIFICATION",
        "OVERVOLTAGE_SHUTDOWN_RELAY",
        "WALK_IN_FUNCTION",
        "FAN_FULL_SPEED",
        "MODULE_SWITCH",
        "MODULE_POWER_LIMIT",
        "MODULE_PFC_ERROR",
        "MODULE_AC_OVERVOLTAGE",
        "DUPLICATE_MODULE_ID",
        "SEVERE_CURRENT_IMBALANCE",
        "COMM_PHASE_LOSS",
        "COMM_IMBALANCE",
        "MODULE_AC_UNDERVOLTAGE_ALARM",
        "SEQUENTIAL_START_FUNCTION",
        "RESERVED0",
        "RESERVED1",
        "FUSE_FAULT_OUTPUT",
        "INTERNAL_COMM_ERROR",
        "UNDERVOLTAGE_OUTPUT",
        "OVERLOAD_ALARM",
        "PNP_ALARM",
        "SLIGHT_CURRENT_IMBALANCE"
    };
    for(int i = 0; i < 32; i++)
    {
        if(handle->bit_status.u32 & (1 << i))
        {
            ESP_LOGI(TAG, "R48[0x%02X]: Status Flag set: %s", addr, flags_names[i]);
        }
    }
    ESP_LOGI(TAG, "MODULE_SWITCH=%s", handle->bit_status.bits.module_switch ? "ON" : "OFF");
}

static inline float be_to_float(const uint8_t in[4])
{
    union { float f; uint8_t b[4]; } u;
    u.b[3] = in[0];
    u.b[2] = in[1];
    u.b[1] = in[2];
    u.b[0] = in[3];
    return u.f;
}

static inline void float_to_be(float f, uint8_t out[4])
{
    union { float f; uint8_t b[4]; } u;
    u.f = f;
    out[0] = u.b[3];
    out[1] = u.b[2];
    out[2] = u.b[1];
    out[3] = u.b[0];
}


bool r48_init(gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{
    // General config
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    g_config.alerts_enabled = 0; // optional: TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_BUS_OFF ...

    // 125 kbps timing + accept all filter
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }
    ret = twai_start();
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "TWAI started @125kbps, ext frames");
    // for(uint8_t i = 0; i < R48_MAX_DEVICE_COUNT; i++)
    // {
    //     if(r48_devices[i] == NULL)
    //     {
    //         r48_device_mem_t *mem = heap_caps_calloc(1, sizeof(r48_device_mem_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
    //         if(mem == NULL)
    //         {
    //             ESP_LOGE(TAG, "malloc failed");
    //             return NULL;
    //         }
    //         mem->device_index = i;
    //         r48_devices[i] = mem;
    //         return mem;
    //     }
    // }
    ESP_LOGI(TAG, "R48 initialized");
    return true;
}

static bool r48_parse_status_message(const twai_message_t *msg)
{

    r48_data_t data;
    memcpy(&data, msg->data, sizeof(data));
    if(data.errType != R48_DATA_ERRTYPE_NONE)
    {
        ESP_LOGW(TAG, "R48: Error in status message, errType=0x%02X", data.errType);
        r48_log_frame(msg);
        return false;
    }

    if(data.msgtype != R48_MSGTYPE_RESPONSE_BYTE_DATA && data.msgtype != R48_MSGTYPE_RESPONSE_BIT_DATA && data.msgtype != R48_MSGTYPE_WRITE_DATA_ACK)
    {
        ESP_LOGW(TAG, "R48: Unknown message type 0x%02X in status message", data.msgtype);
        //On module on/off:
        /*
            W (18563) R48: R48: Unknown message type 0x43 in status message
            I (18563) R48: RX EXT ID=0x060F8003 DLC=8 data=[43 F0 00 35 00 00 00 00 ]
        */
        r48_log_frame(msg);
        return false;
    }

    r48_can_id_t addr = {.u32.direct = msg->identifier};
    r48_handle_t handle = r48_mem.devices[addr.bits.src_addr];
    if(handle == NULL)
    {
        ESP_LOGW(TAG, "r48_parse_status_message: No handle for device index 0x%02X", msg->identifier & 0xFF);
        return false;
    }

    float float_value = be_to_float(data.data.byte_wise.content);
    switch(data.data.byte_wise.valuetype1)
    {
    case R48_VALUETYPE_VOUT: // VOUT
        handle->v_out = float_value;
        handle->v_out_timestamp = esp_timer_get_time();
        ESP_LOGI(TAG, "R48: VOUT = %.2f V", handle->v_out);
        return true;
    case R48_VALUETYPE_IOUT: // IOUT
        handle->i_out = float_value;
        handle->i_out_timestamp = esp_timer_get_time();
        ESP_LOGI(TAG, "R48: IOUT = %.2f A", handle->i_out);
        return true;
        case R48_VALUETYPE_ILIMIT: // ILIMIT
        // Currently ignored
        ESP_LOGI(TAG, "R48: ILIMIT message received: %f A", float_value);
        return true;
    case R48_VALUETYPE_TEMPERATURE: // TEMPERATURE
        handle->temp_C = float_value + R48_TEMPERATURE_OFFSET;
        ESP_LOGI(TAG, "R48: TEMPERATURE = %.2f °C", handle->temp_C);
        return true;
    case R48_VALUETYPE_VIN: // VIN
        handle->v_in = float_value;
        ESP_LOGI(TAG, "R48: VIN = %.2f V", handle->v_in);
        return true;
    case R48_VALUETYPE_BIT_STATUS: // BIT STATUS
        memcpy(handle->bit_status.bytes, msg->data + 4, 4);
        ESP_LOGI(TAG, "R48: BIT STATUS = 0x%08X", handle->bit_status.u32);
        print_all_status_flag_set(addr.bits.src_addr);
        return true;
    case R48_VALUETYPE_IOUT_OFFLINE:
    {
        float amps = fraction_to_amps(float_value);
        handle->i_out_offline = amps;
        char key[16];
        snprintf(key, sizeof(key), "%s%02X", R48_IOUT_OFFLINE_NVS_KEY, addr.bits.src_addr);
        esp_err_t ret = bareco_database_set_float(key, amps);
        if(ret != ESP_OK)
        {
            ESP_LOGW(TAG, "r48_set_current_offline: Failed to store offline current in NVS for module 0x%02X: %s", addr.bits.src_addr, esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "R48: IOUT_OFFLINE = %.2f A", handle->i_out_offline);
        return true;
    }
    case R48_VALUETYPE_VOUT_OFFLINE:
    {
        float volts = float_value; // fraction_to_volts(float_value);
        handle->v_out_offline = volts;
        char key[16];
        snprintf(key, sizeof(key), "%s%02X", R48_VOUT_OFFLINE_NVS_KEY, addr.bits.src_addr);
        esp_err_t ret = bareco_database_set_float(key, volts);
        if(ret != ESP_OK)
        {
            ESP_LOGW(TAG, "r48_set_voltage_offline: Failed to store offline voltage in NVS for module 0x%02X: %s", addr.bits.src_addr, esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "R48: VOUT_OFFLINE = %.2f V", handle->v_out_offline);
        return true;
    }
    case R48_VALUETYPE_IOUT_ONLINE:
    {
        float amps = fraction_to_amps(float_value);
        handle->i_out_set = amps;
        handle->i_out_set_timestamp = esp_timer_get_time();
        handle->remote_off = false; //Setting current will enable the output
        ESP_LOGI(TAG, "R48: IOUT SET ONLINE = %.2f A", handle->i_out_set);
        return true;
    }
    case R48_VALUETYPE_VOUT_ONLINE:
    {
        handle->v_out_set = float_value;
        handle->v_out_set_timestamp = esp_timer_get_time();
        handle->remote_off = false; //Setting voltage will enable the output
        ESP_LOGI(TAG, "R48: VOUT SET ONLINE = %.2f V", handle->v_out_set);
        return true;
    }
    case 0x54: //fallthroug // reserved and ignored
    case 0x58:
        //Currently ignored
        return true;
    case 0x07: // DISPLAY_CURRENT
        // Currently ignored
        return true;
    case 0x0B: // ROOM_TEMPERATURE
        handle->room_temp_C = float_value;
        ESP_LOGI(TAG, "R48: ROOM_TEMPERATURE = %.2f °C", handle->room_temp_C);
        return true;
    default:
        ESP_LOGW(TAG, "R48: Unknown valuetype 0x%02X in status message", data.data.byte_wise.valuetype1);
        ESP_LOGW(TAG, "R48: Value = %.2f", float_value);
        // I (81181) R48: RX EXT ID=0x060F8007 DLC=8 SELF data=[41 F0 00 07 00 00 00 00 ]
        r48_log_frame(msg);
        return false;
    }
}

static bool r48_route_message(const twai_message_t *msg)
{
    if(msg->data_length_code != 8)
        return false; // Invalid message

    r48_can_id_t addr = {.u32.direct = msg->identifier};
    if(addr.bits.proto != 0x070 && addr.bits.proto != 0x060)
    {
        //lets ignore non-R48 messages
        return false;
    }

    uint8_t src_addr = addr.bits.src_addr;
    if(src_addr >= R48_MAX_DEVICE_COUNT)
    {
        ESP_LOGW(TAG, "r48_route_message: Source address 0x%02X out of range", src_addr);
        return false;
    }

    //But we should register a device for this source address if it not exists yet
    r48_handle_t handle = r48_mem.devices[src_addr];;
    if(handle == NULL)
    {
        ESP_LOGW(TAG, "r48_route_message: New device detected at address 0x%02X, allocating memory", src_addr);
        r48_mem.devices[src_addr] = heap_caps_calloc(1, sizeof(r48_device_mem_t), MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL);
        if(r48_mem.devices[src_addr] == NULL)
        {
            ESP_LOGE(TAG, "r48_route_message: malloc failed for device index 0x%02X", src_addr);
            return false;
        }
        char key[16];
        snprintf(key, sizeof(key), "%s%02X", R48_IOUT_OFFLINE_NVS_KEY, src_addr);
        esp_err_t ret1 = bareco_database_get_float(key, &r48_mem.devices[src_addr]->i_out_offline);
        snprintf(key, sizeof(key), "%s%02X", R48_VOUT_OFFLINE_NVS_KEY, src_addr);
        esp_err_t ret2 = bareco_database_get_float(key, &r48_mem.devices[src_addr]->v_out_offline);
        if(ret1 != ESP_OK || ret2 != ESP_OK)
        {
            ESP_LOGW(TAG, "r48_route_message: No stored offline values for device 0x%02X, using defaults", src_addr);
            r48_set_current_offline(src_addr, R48_DEFAULT_OFFLINE_CURRENT_A);
            r48_set_voltage_offline(src_addr, R48_DEFAULT_OFFLINE_VOLTAGE_V);
        }
        else
        {
            ESP_LOGI(TAG, "r48_route_message: Loaded stored offline values for device 0x%02X: Iout=%.2f A, Vout=%.2f V", src_addr,
                r48_mem.devices[src_addr]->i_out_offline,
                r48_mem.devices[src_addr]->v_out_offline);
            r48_set_current_offline(src_addr, r48_mem.devices[src_addr]->i_out_offline);
            r48_set_voltage_offline(src_addr, r48_mem.devices[src_addr]->v_out_offline);
        }
          
    }



    uint8_t dst_addr = addr.bits.dst_addr;
    if(dst_addr != 0xFF && dst_addr != R48_THIS_DEVICE_ADDRESS)
    {
        //Not for us
        ESP_LOGI(TAG, "r48_route_message: Message not for us (dst=0x%02X)", dst_addr);
        return false;
    }

    if(addr.bits.proto == 0x070)
    {
        // device to device message, are ignored
        return false;
    }

    return r48_parse_status_message(msg);


    // // Process message
    // switch(msg->identifier)
    // {
    // case 0x0707F803: // Status broadcast
    //     // ESP_LOGI(TAG, "R48 Status message received");
    //     // Currently ignored
    //     return true;
    // case 0x060F8007: //fallthrough
    // case 0x060F8003:
    // {
    //     return r48_parse_status_message(handle, msg);
    // }
    // default:
    //     r48_log_frame(msg);
    //     return false; // Unknown message
    // }
}

bool r48_send_can_message(const twai_message_t *msg, TickType_t ticks_to_wait)
{
    esp_err_t ret = twai_transmit(msg, ticks_to_wait);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "r48_send_can_message: twai_transmit failed: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

//Dest address 0xFF = broadcast
bool r48_send_status_request(r48_module_adr_t dest_adr, r48_valuetype_e request)
{
    ESP_LOGI(TAG, "r48_send_status_request: Sending request for %02X", request); //, request_text[request] ? request_text[request] : "UNKNOWN");
    if(dest_adr != R48_BROADCAST_ADDRESS && dest_adr >= R48_MAX_DEVICE_COUNT)
    {
        ESP_LOGW(TAG, "r48_send_status_request: Destination address 0x%02X out of range", dest_adr);
        return false;
    }

    r48_can_id_t can_id = {0};
    can_id.bits.proto = 0x060; // R48 protocol
    can_id.bits.dst_addr = dest_adr;
    can_id.bits.src_addr = R48_THIS_DEVICE_ADDRESS;
    can_id.bits.ptp = (dest_adr != R48_BROADCAST_ADDRESS) ? 1 : 0;
    can_id.bits.cnt = 0;
    can_id.bits.res1 = 1;
    can_id.bits.res2 = 1;

    twai_message_t msg = {0};
    msg.identifier = can_id.u32.direct;
    msg.extd = 1;                // 29-bit ID
    msg.rtr = 0;
    msg.data_length_code = 8;

    switch (request)
    {
    case R48_VALUETYPE_ILIMIT: //fallthrough
    case R48_VALUETYPE_VOUT: //fallthrough
    case R48_VALUETYPE_IOUT: //fallthrough
    case R48_VALUETYPE_TEMPERATURE: //fallthrough
    case R48_VALUETYPE_VIN: //fallthrough
    case R48_VALUETYPE_ROOM_TEMPERATURE: //fallthrough
    case R48_VALUETYPE_BIT_STATUS: //fallthrough
        {
            r48_data_t data =
            {
                .err = 0,
                .msgtype = R48_MSGTYPE_REQUEST_BYTE_DATA,
                .errType = R48_DATA_ERRTYPE_NONE,
                .data.byte_wise = {
                .valuetype0_allways0 = 0x00,
                .valuetype1 = (uint8_t)request,
                .content = {0x00, 0x00, 0x00, 0x00}
            }};
            memcpy(msg.data, &data, 8);
            return r48_send_can_message(&msg, 2); //Should not block long time
        }
    case R48_VALUETYPE_ALL:
        {
            r48_data_t data =
            {
                .err = 0,
                .msgtype = R48_MSGTYPE_REQUEST_ALL,
                .errType = R48_DATA_ERRTYPE_NONE,
                .data.byte_wise = {
                .valuetype0_allways0 = 0x00,
                .valuetype1 = 0x80, // all
                .content = {0x44, 0x7A, 0x00, 0x00} //From example, works also with 0x00 0x00 0x00 0x00
            }};
            memcpy(msg.data, &data, 8);
            return r48_send_can_message(&msg, 2); //Should not block long time
        }
          
    default:
        ESP_LOGW(TAG, "r48_send_status_request: Unsupported request %" PRIu8, request);
        // r48_log_msg_full_details(&msg);
        return false;
    }
}

void r48_worker(void)
{
    twai_message_t msg;
    while(twai_receive(&msg, 2) == ESP_OK)
    {
        r48_route_message(&msg);
    }
      
    //     // Check for timeouts
        int64_t now = esp_timer_get_time();
        // if((handle->v_out_timestamp == 0 || now - handle->v_out_timestamp > (R48_STATUS_UPDATE_INTERVAL_MS * 1000UL)))
        // {
            // Value too old
            // #define STOP_STATUS_BROADCAST_REQUESTS 1
            if((now - r48_mem.last_status_broadcast_req_time) >= (R48_MIN_STATUS_UPDATE_INTERVAL_MS * 1000UL))
            {
                #ifndef STOP_STATUS_BROADCAST_REQUESTS
                for(uint8_t dev = 0; dev < R48_MAX_DEVICE_COUNT; dev++)
                {
                    while(twai_receive(&msg, 0) == ESP_OK)
                    {
                        r48_route_message(&msg);
                    }
                    if(r48_mem.devices[dev] == NULL)
                    {
                        continue;
                    }
                    r48_send_status_request(dev, R48_VALUETYPE_BIT_STATUS);
                    r48_send_status_request(dev, R48_VALUETYPE_IOUT);
                    r48_send_status_request(dev, R48_VALUETYPE_VOUT);
                    r48_send_status_request(dev, R48_VALUETYPE_VIN);
                    r48_send_status_request(dev, R48_VALUETYPE_ROOM_TEMPERATURE);
                    r48_send_status_request(dev, R48_VALUETYPE_TEMPERATURE);
                }
                // if(r48_mem.devices[0] != NULL)
                // {
                //     if(!r48_mem.devices[0]->remote_off)
                //         r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_ALL);
                // }
                //Braodcast requests are not working like this
                // r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_ROOM_TEMPERATURE);
                // r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_BIT_STATUS);
                // r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_IOUT);
                // r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_VOUT);
                // r48_send_status_request(R48_BROADCAST_ADDRESS, R48_VALUETYPE_VIN);
                r48_mem.last_status_broadcast_req_time = now;
                #endif
            }
        // }
    // }
}

static bool r48_send_u8_f32(r48_module_adr_t adr, uint8_t reg, float value)
{
    if(adr >= R48_MAX_DEVICE_COUNT && adr != R48_BROADCAST_ADDRESS)
    {
        ESP_LOGE(TAG, "r48_send_u8_f32: module_adr 0x%02X out of range", adr);
        return false;
    }
    twai_message_t msg = {0};
    r48_can_id_t *can_id = (r48_can_id_t *)&msg.identifier;
    can_id->bits.proto = R48_PROTO_MODULE_CONTROLLER_COMM;
    can_id->bits.dst_addr = adr;
    can_id->bits.src_addr = R48_THIS_DEVICE_ADDRESS;
    can_id->bits.ptp = (adr != R48_BROADCAST_ADDRESS) ? 1 : 0;
    // can_id->bits.cnt = 0;
    can_id->bits.res1 = 1;
    can_id->bits.res2 = 1;
    msg.extd = 1;                  // Extended 29-bit
    // msg.rtr = 0;
    msg.data_length_code = 8;
    r48_data_t *data = (r48_data_t *)msg.data;
    // data->err = 0;
    data->msgtype = R48_MSGTYPE_WRITE_DATA;
    data->errType = R48_DATA_ERRTYPE_NONE;
    // data->data.byte_wise.valuetype0_allways0 = 0x00;
    data->data.byte_wise.valuetype1 = reg;
    float_to_be(value, data->data.byte_wise.content);
    return (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);
}

static inline float amps_to_fraction(float amps)
{
    if(amps < 0.f) amps = 0.f;
    float frac = amps / R48_RATED_CURRENT_A;     // 1.00 = 100% Nennstrom
    // Viele Firmwares akzeptieren 0.10..1.21; 0.00 (abschalten) klappt nicht immer. :contentReference[oaicite:3]{index=3}
    if(frac > 1.21f) frac = 1.21f;
    return frac;
}

static inline float fraction_to_amps(float frac)
{
    float amps = frac * R48_RATED_CURRENT_A;
    return amps;
}

bool r48_set_current_offline(r48_module_adr_t module_adr, float amps)
{
    float frac = amps_to_fraction(amps);
    if(!r48_send_u8_f32(module_adr, R48_VALUETYPE_IOUT_OFFLINE, frac))
        return false;
  
    ESP_LOGI(TAG, "R48: Set offline current to %.2f A", amps);

    //TODO: Check if this is needed when broadcasting or if we get feedback from the module
    //As we parsing the current from the acknowledgement message, but if we broadcast, we possibly don't get a response
    // if(module_adr != R48_BROADCAST_ADDRESS && r48_mem.devices[module_adr] != NULL)
    // {
    //     r48_handle_t handle = r48_mem.devices[module_adr];
    //     handle->i_out_set = amps;
    //     handle->i_out_set_timestamp = esp_timer_get_time();
    // }
    return true;
}


bool r48_set_voltage_offline(r48_module_adr_t module_adr, float volts)
{
    if(volts > R48_MAX_VOLTAGE_V)
    {
        ESP_LOGW(TAG, "R48: Requested voltage %.2f V exceeds maximum of %.2f V, clamping", volts, R48_MAX_VOLTAGE_V);
        volts = R48_MAX_VOLTAGE_V;
    }

    ESP_LOGI(TAG, "R48: Set offline voltage to %.2f V", volts);
    if(!r48_send_u8_f32(module_adr, R48_VALUETYPE_VOUT_OFFLINE, volts))
        return false;

    return true;
}

bool r48_set_current_online(r48_module_adr_t module_adr, float amps)
{
    ESP_LOGI(TAG, "R48: Set online current to %.2f A", amps);
    float frac = amps_to_fraction(amps);
    if(!r48_send_u8_f32(module_adr, R48_VALUETYPE_IOUT_ONLINE, frac))
    {
        return false;
    }
    return true;
}

bool r48_set_voltage_online(r48_module_adr_t module_adr, float volts)
{

    if(volts > R48_MAX_VOLTAGE_V)
    {
        ESP_LOGW(TAG, "R48: Requested voltage %.2f V exceeds maximum of %.2f V, clamping", volts, R48_MAX_VOLTAGE_V);
        volts = R48_MAX_VOLTAGE_V;
    }


    ESP_LOGI(TAG, "R48: Set online voltage to %.2f V", volts);
    if(!r48_send_u8_f32(module_adr, R48_VALUETYPE_VOUT_ONLINE, volts))
    {
        return false;
    }

    return true;
}

bool _r48_send_write_data(r48_module_adr_t module_adr, uint8_t valuetype, const uint8_t content[4])
{
    twai_message_t msg = {0};
    msg.extd = 1;                 // extended frame
    msg.rtr = 0;                  // data frame
    msg.data_length_code = 8;
    r48_data_t *data = (r48_data_t *)msg.data;

    r48_can_id_t *can_id = (r48_can_id_t *)&msg.identifier;
    can_id->bits.proto = R48_PROTO_MODULE_CONTROLLER_COMM;
    can_id->bits.dst_addr = module_adr;
    can_id->bits.src_addr = R48_THIS_DEVICE_ADDRESS;
    can_id->bits.ptp = (module_adr != R48_BROADCAST_ADDRESS) ? 1 : 0;
    can_id->bits.cnt = 0;
    can_id->bits.res1 = 1;
    can_id->bits.res2 = 1;

    data->msgtype = R48_MSGTYPE_WRITE_DATA;
    // data->err = 0;
    data->errType = R48_DATA_ERRTYPE_NONE;
    // data->data.byte_wise.valuetype0_allways0 = 0x00;
    data->data.byte_wise.valuetype1 = valuetype;
    data->data.byte_wise.content[0] = content[0];
    data->data.byte_wise.content[1] = content[1];
    data->data.byte_wise.content[2] = content[2];
    data->data.byte_wise.content[3] = content[3];

    esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if(res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send write data command: %s", esp_err_to_name(res));
        return false;
    }
    return true;
}

bool _r48_remote_on_off(r48_module_adr_t module_adr, bool remote_on)
{
    uint8_t content[4] = {0};
    content[1] = remote_on ? 0x00 : 0x01; // 1 = remote off, 0 = remote on
  

    if(!_r48_send_write_data(module_adr, R48_VALUETYPE_REMOTE_OFF, content))
    {
        ESP_LOGE(TAG, "Failed to send remote command: remote %s", remote_on ? "on" : "off");
        return false;
    }

    if(module_adr != R48_BROADCAST_ADDRESS && r48_mem.devices[module_adr] != NULL)
    {
        r48_handle_t handle = r48_mem.devices[module_adr];
        handle->remote_off = !remote_on;
        if(remote_on)
        {
            //When enabling remote on, reset IOUT to last set value
            handle->i_out = handle->i_out_set;
        }
    }
    return true;
}

bool r48_remote_off(r48_module_adr_t module_adr)
{
    return _r48_remote_on_off(module_adr, false);
}

bool r48_remote_on(r48_module_adr_t module_adr)
{
    return _r48_remote_on_off(module_adr, true);
}

bool r48_fan_set_fullspeed(r48_module_adr_t module_adr)
{
    uint8_t content[4] = {0x00, 0x01, 0x00, 0x00}; // 1 = full speed, 0 = auto
    return _r48_send_write_data(module_adr, R48_VALUETYPE_FAN_CONTROL, content);
    return true;
}

bool r48_fan_set_auto(r48_module_adr_t module_adr)
{
    uint8_t content[4] = {0x00, 0x00, 0x00, 0x00}; // 1 = full speed, 0 = auto
    return _r48_send_write_data(module_adr, R48_VALUETYPE_FAN_CONTROL, content);
}

bool r48_turn_on_off_module(bool on)
{
    uint8_t content[4] = {0x00, on ? 0x00 : 0x01, 0x00, 0x00};
    ESP_LOGI(TAG, "r48_turn_on_off_module: Turning module %s", on ? "ON" : "OFF");
    return _r48_send_write_data(R48_BROADCAST_ADDRESS, R48_VALUETYPE_TURN_OFF_MODULE, content);
}