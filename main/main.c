#include <stdio.h>
#include "string.h"
#include "float.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "app_settings.h"
#include "c6_prototyper_core.h"
#include "core_utils.h"
#include "ap_server.h"
#include "data_logger.h"
#include "app_logger.h"

#include "sfm3003.h"

static const char* TAG = "Main";

// Settings local variables. Dont expect to need a 16 bit int ever, but not worth defining settings fns for 8 bit (+ size checks)
uint16_t sfm_burst_len;
uint16_t sfm_bperiod_s;
bool sfm_use_lp_core;  // if true then all recorded values come from LP Core sampling, and the burst settings are ignored (no burst or singleton). NB "live" readings not affected.

/* Data Logging - Structure and Functions */
// For neatness (at least) make the slot size so that a whole number of units fills a 4k page.
#define DATA_SLOT_SIZE 32
// The logging struct may be shorter than the slot size
struct data_unit_s {
    uint32_t timestamp;
    float flow_slm;
    float flow_mps;
    float temp;
    float flow_slm_sd;  // only for LP Core use
    // three more slots not currently used
} typedef data_unit_t;
typedef union {
    data_unit_t struct_rep;
    uint8_t raw_rep[DATA_SLOT_SIZE];
} data_unit;

uint8_t data_unit_size = sizeof(data_unit_t);  // No of bytes written/read per record - may be less than DATA_SLOT_SIZE
uint8_t data_slot_size = DATA_SLOT_SIZE;  // Unit of increment of address per record.

// Take the reading(s), compose the log record and log it. Callback from core_activity()
// The return error is only for the write_record; sensor activity should have produced its own logs
esp_err_t do_work_fn(time_t ts){
    data_unit_t record = {.timestamp=(uint32_t) ts, .flow_slm_sd=FLOAT_NA};  // use wake-up time for clean record. No appreciable diff in this case but maybe sometimes.

    // NOTE : 3 way choice of behaviour here, and each has its own return statement

    // If LP Core in use then all we need to do is get the mean of stored samples
    if (sfm_use_lp_core) {
        esp_err_t err = lp_core_readings(&record.temp, &record.flow_slm, &record.flow_slm_sd);
        app_logger_store();  // essential if debug logging in sfm3003 component
        // Skip logging if the LP Core was not ready and buffer filled
        // ESP_ERR_INVALID_SIZE occurs if not enough samples, ESP_ERR_NOT_FOUND if the LP Core was not running
        if ((err != ESP_ERR_INVALID_SIZE) && (err != ESP_ERR_NOT_FOUND)) {
            record.flow_mps = compute_flow_mps(record.flow_slm);  // NB not doing the same for the SD because the conversion should be done before SD calc. Calib fns not valid for SD.
            return write_record(&record);
        }
        return err;
    }

    // HP Core in use - SFM3003 under direct control
    if (sfm_burst_len == 0) {  // Single reading per wake. The temp is read before full warm-up
        sfm_read_oneshot(&record.flow_slm, &record.temp, true);  // apply offset
        record.flow_mps = compute_flow_mps(record.flow_slm);
        return write_record(&record);

    } else {
        // Multiple readings with SFM in measurement mode throughout.
        // The temp is read when each flow is. This includes the sfm_burst_len == 1 case (hence it is different from case when == 0).
        // The timestamp is fudged to be neat and the delay is not computed to us or ms accuracy as it is N seconds long.        
        if (sfm_get_state() == SFM_ASLEEP) {  // shouldn't happen in well-written main functions
            ESP_LOGW(TAG, "SFM3003 was asleep; waking. This should only happen with sleep hold-off.");
            sfm_wake();
        }
        esp_err_t err = sfm_to_measurement(true);
        if (err == ESP_OK){
            for (uint16_t i = 0; i < sfm_burst_len; i++){
                if (sfm_take_reading((i == 0)?50:0, &record.flow_slm, &record.temp, true) != ESP_OK) break;
                record.flow_mps = compute_flow_mps(record.flow_slm);
                if (write_record(&record) != ESP_OK) break;
                vTaskDelay(sfm_bperiod_s * 1000 / portTICK_PERIOD_MS);
                record.timestamp += sfm_bperiod_s;
                app_logger_store();  // essential if debug logging in sfm3003 component
            }
        }

        // idle and to sleep, ready for ESP32 deep sleep. Try these even if errors above.
        // Each will log an error but the return value from this function depends ONLY on what happens taking the reading
        sfm_to_idle();
        sfm_to_sleep();
        
        app_logger_store();

        return err;

    }
}

// For CSV emitter in HTTP response. This is a callback to format one record (as raw bytes) to one row of csv
// NB: column order MUST be consistent across heading and data
size_t format_record(uint8_t bytes[], char* formatted, size_t buff_size, bool as_csv){
    size_t str_len;
    if (bytes == NULL){
        // heading
        str_len = snprintf(formatted, buff_size, "timestamp,flow_slm,flow_mps,temp_c,flow_slm_sd\n");
    } else {
        data_unit record;
        memcpy(&record.raw_rep, bytes, data_unit_size);
        
        // take account of possible missing values - signalled by -MAX_FLT - (and apply dp formatting)
        char s_flow_slm[8];
        char s_flow_mps[8];
        char s_temp[8];
        char s_flow_slm_sd[8];
        const char* missing_val = (as_csv)?"":"?";  // "" for missing data if CSV else "?"
        float_to_string_guarded(s_flow_slm, 8, record.struct_rep.flow_slm, "%.2f", missing_val);
        float_to_string_guarded(s_flow_mps, 8, record.struct_rep.flow_mps, "%.2f", missing_val);
        float_to_string_guarded(s_temp, 8, record.struct_rep.temp, "%.2f", missing_val);
        float_to_string_guarded(s_flow_slm_sd, 8, record.struct_rep.flow_slm_sd, "%.3f", missing_val);

        if (as_csv){
            str_len = snprintf(formatted, buff_size, "%lu,%s,%s,%s,%s\n", record.struct_rep.timestamp, s_flow_slm, s_flow_mps, s_temp, s_flow_slm_sd);
        } else {
            // Version for user-facing format of one record from datalog
            if (record.struct_rep.flow_slm_sd != FLOAT_NA) {
                // LP Core sampling - have a SD
                str_len = snprintf(formatted, buff_size, "Timestamp=%lu, Mean flow=%sslm=%sm/s (sd=%sslm), T=%sC", record.struct_rep.timestamp, s_flow_slm, s_flow_mps, s_flow_slm_sd, s_temp);
            } else {
                str_len = snprintf(formatted, buff_size, "Timestamp=%lu, Flow=%sslm=%sm/s, T=%sC", record.struct_rep.timestamp, s_flow_slm, s_flow_mps, s_temp);
            }
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

    float temp;
    float flow_slm;
    esp_err_t err = sfm_read_oneshot(&flow_slm, &temp, true);  // apply offset
    float flow_mps = compute_flow_mps(flow_slm);

    size_t index = 0;
    if (for_html) {  // include the serial number for web page
        index = snprintf(msgbuff, msgbuff_len, "%sSN: %llu%s\n", prefix, sfm_serial_number, suffix);
    }

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
#define SFM_N_SETTINGS 3
const char* main_settings_available[SFM_N_SETTINGS] = {"SFM_BURST_LEN", "SFM_BPERIOD_S", "SFM_USE_LP_CORE"};  // see .n_settings, below

#ifdef CONFIG_DEF_SFM_USE_LP_CORE
bool def_sfm_use_core = true;
#else
bool def_sfm_use_core = false;
#endif

// fn to load local variables from NVS or default
void main_load_settings(){
    ESP_LOGD(TAG, "Reading Settings");
    setting_get_uint16("SFM_BURST_LEN", &sfm_burst_len, CONFIG_DEF_SFM_BURST_LEN);  // default is no burst
    setting_get_uint16("SFM_BPERIOD_S", &sfm_bperiod_s, CONFIG_DEF_SFM_BPERIOD_S);
    setting_get_bool("SFM_USE_LP_CORE", &sfm_use_lp_core, def_sfm_use_core);
}
// fn to get a string version of the local value and the original (aka default) - for web server
void main_setting_get_str(const char* key, char* current, char* original){
    if (strcmp(key, "SFM_BURST_LEN") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%u", sfm_burst_len);
        snprintf(original, SETTINGS_CO_BUFF_LEN, "%u", CONFIG_DEF_SFM_BURST_LEN);
    } else if (strcmp(key, "SFM_BPERIOD_S") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%u", sfm_bperiod_s);
        snprintf(original, SETTINGS_CO_BUFF_LEN, "%u", CONFIG_DEF_SFM_BPERIOD_S);
    } else if (strcmp(key, "SFM_USE_LP_CORE") == 0){
        strcpy(current, sfm_use_lp_core?"y":"n");
        strcpy(original, def_sfm_use_core?"y":"n");
    } else {
        current = NULL;
        original = NULL;
    }
}
// fn to take string form of setting from webserver and store to local variable and NVS
esp_err_t main_setting_store_str(const char* key, char * value){
    esp_err_t err = ESP_ERR_INVALID_ARG;  // for if no case is matched
    if (strcmp(key, "SFM_BURST_LEN") == 0) {
        err = setting_store_uint16(key, value, &sfm_burst_len);  // updates local value with parsed result irrespective of whether NVS storage worked
    } else if (strcmp(key, "SFM_BPERIOD_S") == 0) {
        err = setting_store_uint16(key, value, &sfm_bperiod_s);
    } else if (strcmp(key, "SFM_USE_LP_CORE") == 0) {
        err = setting_store_bool(key, value, &sfm_use_lp_core);
    }
    return err;
}
app_settings_source_t main_ass = {
    .source_code="MAIN",
    .source_name="Main",
    .settings_available_ptr=main_settings_available,
    .n_settings=SFM_N_SETTINGS,
    .settings_get_str_fn=main_setting_get_str,
    .settings_store_str_fn=main_setting_store_str
};

#define APP_SETTINGS_N_COMPONENTS 3
app_settings_source_t app_settings_sources[APP_SETTINGS_N_COMPONENTS];  // A bit hacky? Use extern in http_server.c to get this.
uint8_t app_settings_sources_size = APP_SETTINGS_N_COMPONENTS;  // sizeof(app_settings_sources) / sizeof(app_settings_source_t);

// set the I2C interface and SFM3003 state for use from HP Core, specifically from the WS interactions.
// Only used when the LP Core is being used to take measurements, called as call-back
esp_err_t switch_sfm_to_hp(){
    lp_core_stop();
    return ESP_OK;
}

// Used to revert to LP Core access. Not used in initial setup, called as call-back
esp_err_t switch_sfm_to_lp(){
    return lp_core_start(true);
}

void app_main(void)
{
    // Logging levels - Generic set in LOG_DEFAULT_LEVEL is expected to be WARN and LOG_MAXIMUM_LEVEL to DEBUG (the latter allows runtime mutation to DEBUG).
    // C6 prototyper (ie not ESP-IDF) components set overrides according to KConfig in their xxxx_init()
    #ifdef CONFIG_MAIN_LOG_LEVEL
    esp_log_level_set(TAG, CONFIG_MAIN_LOG_LEVEL);
    #else
    esp_log_level_set(TAG, ESP_LOG_WARN);
    #endif

    // gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);

    int wake_cause = core_init();  // actually esp_sleep_wakeup_cause_t return type
    
    main_load_settings();

    init_data_logger(data_unit_size, data_slot_size);
    //http_server_attach_data_interface(format_record, time_of_record, live_reading);  // callbacks to these functions in data logger component from web server

    // Configurable settings (via web server). Ordering here -> UI order.
    app_settings_sources[0] = core_ass;
    app_settings_sources[1] = main_ass;
    app_settings_sources[2] = sfm3003_ass;

    // setup i2c and read serial number. Include a wake interaction since that will almost always be required when app_main is run, since SFM put to sleep before ESP32 sleeps.
    sfm_init(sfm_use_lp_core, true, wake_cause);  // logs its own errors and leaves state as SFM_MISSING on fail, so no need to say more or take further action.

    app_logger_store();

    // set callbacks before triggering the core activity loop
    core_set_data_callbacks(do_work_fn, format_record, time_of_record, live_reading);
    if (sfm_use_lp_core){
        core_set_ws_callbacks(switch_sfm_to_hp, switch_sfm_to_lp);
    }
    core_activity();
}