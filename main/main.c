#include <stdio.h>
#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "app_settings.h"
#include "c6_prototyper_core.h"
#include "core_utils.h"
#include "ap_server.h"
#include "data_logger.h"
#include "app_logger.h"

#include "sfm3003.h"

const char* compiled_at = __DATE__ " @ " __TIME__;

static const char* TAG = "Main";


/* Data Logging - Structure and Functions */
// unit of logging struct. For neatness (at least) make this so that a whole number of units fills a 4k page.
struct data_unit_s {  // 16 bytes
    uint32_t timestamp;
    float flow_slm;
    float flow_mps;
    float temp;
} typedef data_unit_t;
uint8_t data_unit_size = sizeof(data_unit_t);
typedef union {
    data_unit_t struct_rep;
    uint8_t raw_rep[16];
} data_unit;

// Take the reading, compose the log record and log it. Callback from core_activity()
// The return error is only for the write_record; sensor activity should have produced its own logs
esp_err_t do_work_fn(time_t ts){
    data_unit_t record = {.timestamp=(uint32_t) ts};  // use wake-up time for clean record. No appreciable diff in this case but maybe sometimes.

    // float temp;
    // float flow_slm;
    sfm_read_oneshot(&record.flow_slm, &record.temp);
    record.flow_mps = compute_flow_mps(record.flow_slm);

    return write_record(&record);
}

// For CSV emitter in HTTP response. This is a callback to format one record (as raw bytes) to one row of csv
// NB: column order MUST be consistent across heading and data
size_t format_record(uint8_t bytes[], char* formatted, size_t buff_size, bool as_csv){
    // TODO checks for -MAX_FLT for all floats
    size_t str_len;
    if (bytes == NULL){
        // heading
        str_len = snprintf(formatted, buff_size, "timestamp,flow_sfm,flow_mps,temp_c\n");
    } else {
        data_unit record;
        memcpy(&record.raw_rep, bytes, data_unit_size);
        
        // take account of possible missing values (and apply dp formatting)
        char s_flow_slm[8];
        char s_flow_mps[8];
        char s_temp[8];
        const char* missing_val = (as_csv)?"":"NA";  // "" for missing data if CSV else "NA"
        float_to_string_guarded(s_flow_slm, 8, record.struct_rep.flow_slm, "%.2f", missing_val);
        float_to_string_guarded(s_flow_mps, 8, record.struct_rep.flow_mps, "%.2f", missing_val);
        float_to_string_guarded(s_temp, 8, record.struct_rep.temp, "%.2f", missing_val);

        if (as_csv){
            str_len = snprintf(formatted, buff_size, "%lu,%s,%s,%s\n", record.struct_rep.timestamp, s_flow_slm, s_flow_mps, s_temp);
        } else {
            // Version for user-facing format of one record from datalog
            str_len = snprintf(formatted, buff_size, "Timestamp=%lu, Flow=%sSLM=%sm/s, T=%sC", record.struct_rep.timestamp, s_flow_slm, s_flow_mps, s_temp);
        }
    }
    return (str_len + 1 < buff_size)?str_len:buff_size;
}

// Extract the timestamp (return value) and generate a formatted version from the raw record bytes. Timestamp usually the first 4 bytes but this is not assumed.
uint32_t time_of_record(uint8_t bytes[], char* formatted, size_t buff_size, bool for_filename){
    data_unit record;
    memcpy(&record.raw_rep, bytes, data_unit_size);
    time_t ts = record.struct_rep.timestamp;

    return format_timestamp(ts, formatted, buff_size, for_filename);
}

// Create formatted text for a live reading.
// Returns void; sensor failure should be reported in the formatted strings and should have been logged by each sensor read fn
void live_reading(char *msgbuff, size_t msgbuff_len, bool for_html){
    const char* prefix = for_html?"<p>":"";
    const char* suffix = for_html?"</p>":"";

    size_t index = snprintf(msgbuff, msgbuff_len, "%sSN: %llu%s\n", prefix, sfm_serial_number, suffix);

    float temp;
    float flow_slm;
    esp_err_t err = sfm_read_oneshot(&flow_slm, &temp);
    float flow_mps = compute_flow_mps(flow_slm);

    if (err == ESP_OK){
        // take account of possible missing values (and apply dp formatting).
        // this is likely to be redundant fuss as it is unlikely that individual data items will be missing and err still OK
        char s_flow_slm[8];
        char s_flow_mps[8];
        char s_temp[8];
        float_to_string_guarded(s_flow_slm, 8, flow_slm, "%.2f", "NA");
        float_to_string_guarded(s_flow_mps, 8, flow_mps, "%.2f", "NA");
        float_to_string_guarded(s_temp, 8, temp, "%.2f", "NA");

        snprintf(&msgbuff[index], (index < msgbuff_len)?msgbuff_len-index:0, "%sFlow = %sslm = %sm/s Temp = %sC%s", prefix, s_flow_slm, s_flow_mps, s_temp, suffix);
    } else {
        snprintf(&msgbuff[index], (index < msgbuff_len)?msgbuff_len-index:0, "%sFlow & Temp: %s%s", prefix, esp_err_to_name(err), suffix);
    }
    
}

/* Settings for web access */
// First stuff for main
// List of storage keys. Max 15 chars

// TODO add burst vs one-shot control settings and logic
const char* main_settings_available[] = {};  // see .n_settings, below

// fn to load local variables from NVS or default
void main_load_settings(){
    ESP_LOGD(TAG, "Reading Settings (none)");
}
// fn to get a string version of the local value and the original (aka default) - for web server
void main_setting_get_str(const char* key, char* current, char* original){
    // current = NULL;
    // original = NULL;
}
// fn to take string form of setting from webserver and store to local variable and NVS
esp_err_t main_setting_store_str(const char* key, char * value){
    esp_err_t err = ESP_ERR_INVALID_ARG;  // for if no case is matched
    return err;
}
app_settings_source_t main_ass = {
    .source_code="MAIN",
    .source_name="Main",
    .settings_available_ptr=main_settings_available,
    .n_settings=0,
    .settings_get_str_fn=main_setting_get_str,
    .settings_store_str_fn=main_setting_store_str
};

#define APP_SETTINGS_N_COMPONENTS 2
app_settings_source_t app_settings_sources[APP_SETTINGS_N_COMPONENTS];  // A bit hacky? Use extern in http_server.c to get this.
uint8_t app_settings_sources_size = APP_SETTINGS_N_COMPONENTS;  // sizeof(app_settings_sources) / sizeof(app_settings_source_t);

void app_main(void)
{
    // Logging levels - Generic set in LOG_DEFAULT_LEVEL is expected to be WARN and LOG_MAXIMUM_LEVEL to INFO (the latter allows runtime mutation to INFO).
    // C6 prototyper (ie not ESP-IDF) components set overrides according to KConfig flag in their xxxx_init()
    #ifdef CONFIG_MAIN_LOG_INFO
    esp_log_level_set(TAG, ESP_LOG_INFO);
    #else
    esp_log_level_set(TAG, ESP_LOG_WARN);
    #endif

    // gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    core_init();
    
    main_load_settings();

    init_data_logger(data_unit_size);
    http_server_attach_data_interface(format_record, time_of_record, live_reading);  // callbacks to these functions in data logger component from web server

    // Configurable settings (via web server). Ordering here -> UI order.
    app_settings_sources[0] = main_ass;
    app_settings_sources[1] = core_ass;
    // None for the SFM3003

    // setup interface and scan for DS18B20 devices
    sfm_init();  // logs its own errors and leaves state as SFM_MISSING on fail, so no need to say more or take further action.

    app_logger_store();

    core_activity(do_work_fn);  // pass work callback
}