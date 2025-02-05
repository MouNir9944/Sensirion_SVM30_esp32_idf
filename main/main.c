#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define VERSION "1.2 / August 2020";
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define TEST_I2C_PORT I2C_NUM_0
const char * SVM30_VERSION = VERSION;
static const uint8_t SHTC1_ADDRESS = 0x70;
static const uint8_t SGP30_I2C_ADDRESS = 0x58;
const uint8_t REST_I2C_ADDRESS=0x0;
static const uint16_t SHTC3_CMD_WAKEUP = 0x3517;
#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32
#define SGP30_CMD_GET_FEATURESET 0x202f
#define SGP30_CMD_GET_FEATURESET_DURATION_US 10000
#define SGP30_PRODUCT_TYPE 0
#define SGP30_ERR_INVALID_PRODUCT_TYPE (-12)
#define SGP30_CMD_GET_FEATURESET_WORDS 1
#define SGP30_ERR_UNSUPPORTED_FEATURE_SET (-10)
#define SGP30_CMD_IAQ_INIT 0x2003
#define SGP30_CMD_IAQ_INIT_DURATION_US 10     // 10000 == 10ms
#define SHTC1_CMD_MEASURE_HPM 0x7866
#define CRC8_LEN 1
#define SGP30_CMD_SET_ABSOLUTE_HUMIDITY 0x2061
#define SGP30_CMD_IAQ_MEASURE_DURATION_US 12  // 12000 == 12ms
#define SGP30_CMD_SET_ABSOLUTE_HUMIDITY_DURATION_US 10    // 10000==10ms
#define SGP30_CMD_GET_IAQ_BASELINE_WORDS 2
#define SGP30_CMD_GET_IAQ_BASELINE 0x2015
#define SGP30_CMD_IAQ_MEASURE_WORDS 2
#define SGP30_CMD_GET_IAQ_BASELINE_DURATION_US 10    //10000==10ms
#define SGP30_CMD_IAQ_MEASURE 0x2008
#define SHTC1_MEASUREMENT_DURATION_USEC  14   //    14400== 14ms
static uint16_t shtc1_cmd_measure = SHTC1_CMD_MEASURE_HPM;
static const uint16_t SHTC1_CMD_DURATION_USEC = 1;// 1000us; == 1ms
#define SGP30_Init_Air_Quality      0x2003
#define RESET_CMD       0x0006
#define SGP30_Read_ID               0x3682
#define SHTC1_Read_ID                   0xEFC8
#define SGP30_Get_Feature_Set       0x202F

/************************ */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif /* ARRAY_SIZE */
esp_err_t sgp30_probe() ;
/* T_LO and T_HI parametrize the first and last temperature step of the absolute
 * humidity lookup table (at 100%RH). The lookup table entries have to be
 * linearly spaced. We provide a python script to generate look up tables based
 * on customizable T_LO/T_HI and number of steps. */

/**
 * T_LO - Towest temperature sampling point in the lookup table.
 * Temperature value in milli-degrees Centigrade
 */
#define T_LO (-20000)

/**
 * T_HI - Towest temperature sampling point in the lookup table.
 * Temperature value in milli-degrees Celsius (Centigrade)
 */
#define T_HI (70000)

/**
 * Lookup table for linearly spaced temperature points between T_LO and T_HI
 * Absolute Humidity value in mg/m^3.
 */
static const uint32_t AH_LUT_100RH[] = {1078,  2364,  4849,  9383,   17243,
                                        30264, 50983, 82785, 130048, 198277};
/**
 * T_STEP is the temperature step between the sampling points in the lookup
 * table. It is determined by T_HI, T_LO and the number of entries in
 * AH_LUT_100RH and does not have to be adapted when changing the paramters.
 */
static const uint32_t T_STEP = (T_HI - T_LO) / (ARRAY_SIZE(AH_LUT_100RH) - 1);
/*************************************** */



i2c_master_dev_handle_t scht1_handle;
i2c_master_dev_handle_t svm30_handle;
i2c_master_dev_handle_t rest_handle;

void i2c_init(){
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg_SHTC1 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC1_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_device_config_t dev_cfg_spg30 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP30_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_device_config_t REST_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = REST_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_SHTC1, &scht1_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_spg30, &svm30_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &REST_cfg, &rest_handle));

}


uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

uint16_t sensirion_fill_cmd_send_buf(uint8_t* buf, uint16_t cmd,
                                     const uint16_t* args, uint8_t num_args) {

    uint8_t crc;
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = sensirion_common_generate_crc((uint8_t*)&buf[idx - 2],
                                            SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}


esp_err_t sensirion_i2c_write_cmd(i2c_master_dev_handle_t i2c_handler,uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, command, NULL, 0);

    return i2c_master_transmit(i2c_handler, &buf, SENSIRION_COMMAND_SIZE,-1);
}

esp_err_t sensirion_i2c_write_cmd_with_args(i2c_master_dev_handle_t i2c_handler, uint16_t command,
                                          const uint16_t* data_words,
                                          uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    uint16_t buf_size;

    buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
     return i2c_master_transmit(i2c_handler, &buf, buf_size,-1);
}



esp_err_t sensirion_i2c_delayed_read_cmd(i2c_master_dev_handle_t i2c_handler, uint16_t cmd,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words) {
    esp_err_t ret;
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, cmd, NULL, 0);
    ret = i2c_master_transmit(i2c_handler, &buf, SENSIRION_COMMAND_SIZE,-1);
    if (ret != ESP_OK)
        {return ret;}

    if (delay_us)
       {vTaskDelay(pdMS_TO_TICKS(delay_us));} 

    return i2c_master_receive(i2c_handler, data_words, num_words,-1);
}


esp_err_t shtc1_wake_up(void) {

    return sensirion_i2c_write_cmd(scht1_handle, SHTC3_CMD_WAKEUP);
}

esp_err_t shtc1_read_serial(uint32_t* serial) {
    esp_err_t ret;
    const uint16_t tx_words[] = {0x007B};
    uint16_t serial_words[SENSIRION_NUM_WORDS(*serial)];
  

    ret = sensirion_i2c_write_cmd_with_args(scht1_handle, 0xC595, tx_words,
                                            SENSIRION_NUM_WORDS(tx_words));
    if (ret)
      {return ret;}

    vTaskDelay(pdMS_TO_TICKS(SHTC1_CMD_DURATION_USEC));

    ret = sensirion_i2c_delayed_read_cmd(scht1_handle, 0xC7F7, SHTC1_CMD_DURATION_USEC, &serial_words[0], 1);
    if (ret)
      {return ret;}

    ret = sensirion_i2c_delayed_read_cmd(scht1_handle, 0xC7F7, SHTC1_CMD_DURATION_USEC, &serial_words[1], 1);
    if (ret)
      {return ret;}

    *serial = ((uint32_t)serial_words[0] << 16) | serial_words[1];
    printf("serial: %lu\n", (unsigned long)serial);
    return ESP_OK;
}



esp_err_t shtc1_probe(void) {
    uint32_t serial;

    shtc1_wake_up(); /* Try to wake up the sensor, ignore return value */
    return shtc1_read_serial(&serial);
}


esp_err_t svm_probe() {
    esp_err_t err;

    err = shtc1_probe();
    if (err != ESP_OK)
        {return err;}

    return sgp30_probe();
}


/*******************************/
esp_err_t sgp30_get_feature_set_version(uint16_t* feature_set_version,
                                      uint8_t* product_type) {
    esp_err_t ret;
    uint16_t words[SGP30_CMD_GET_FEATURESET_WORDS];

    ret = sensirion_i2c_delayed_read_cmd(svm30_handle,
                                         SGP30_CMD_GET_FEATURESET,
                                         SGP30_CMD_GET_FEATURESET_DURATION_US,
                                         words, SGP30_CMD_GET_FEATURESET_WORDS);

    if (ret != ESP_OK)
        {return ret;}

    *feature_set_version = words[0] & 0x00FF;
    *product_type = (uint8_t)((words[0] & 0xF000) >> 12);

    return ESP_OK;
}


static int16_t sgp30_check_featureset(uint16_t needed_fs) {
    int16_t ret;
    uint16_t fs_version;
    uint8_t product_type;

    ret = sgp30_get_feature_set_version(&fs_version, &product_type);
    if (ret != ESP_OK)
    { return ret;}

    if (product_type != SGP30_PRODUCT_TYPE)
       { return SGP30_ERR_INVALID_PRODUCT_TYPE;}

    if (fs_version < needed_fs)
        {return SGP30_ERR_UNSUPPORTED_FEATURE_SET;}

    return ESP_OK;
}

esp_err_t sgp30_iaq_init() {
    esp_err_t ret =sensirion_i2c_write_cmd(svm30_handle, SGP30_CMD_IAQ_INIT);
    vTaskDelay(pdMS_TO_TICKS(SGP30_CMD_IAQ_INIT_DURATION_US));
    return ret;
}

esp_err_t sgp30_probe() {
    esp_err_t ret = sgp30_check_featureset(0x20);

    if (ret != ESP_OK)
       { return ret;}

    return sgp30_iaq_init();
}

/************************************************** */

int8_t sensirion_common_check_crc(const uint8_t* data, uint16_t count,
                                  uint8_t checksum) {
    if (sensirion_common_generate_crc(data, count) != checksum)
        return ESP_FAIL;
    return ESP_OK;
}

esp_err_t sensirion_i2c_read_words_as_bytes(i2c_master_dev_handle_t *i2c_handler, uint8_t* data,
                                          uint16_t num_words) {
    esp_err_t ret;
    uint16_t i, j;
    uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
    uint8_t* const buf8 = (uint8_t*)word_buf;

    ret = i2c_master_receive(i2c_handler, buf8, size,-1);
    if (ret != ESP_OK)
        return ret;

    /* check the CRC for each word */
    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

        ret = sensirion_common_check_crc(&buf8[i], SENSIRION_WORD_SIZE,
                                         buf8[i + SENSIRION_WORD_SIZE]);
        if (ret != ESP_OK)
            return ret;

        data[j++] = buf8[i];
        data[j++] = buf8[i + 1];
    }

    return ESP_OK;
}


esp_err_t sensirion_i2c_read_words(i2c_master_dev_handle_t i2c_handler, uint16_t* data_words,
                                 uint16_t num_words) {
    esp_err_t ret;
    uint8_t i;
    const uint8_t* word_bytes;

    ret = sensirion_i2c_read_words_as_bytes(i2c_handler, (uint8_t*)data_words,
                                            num_words);
    if (ret != ESP_OK)
        return ret;

    for (i = 0; i < num_words; ++i) {
        word_bytes = (uint8_t*)&data_words[i];
        data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
    }

    return ESP_OK;
}



esp_err_t shtc1_measure(void) {
    return sensirion_i2c_write_cmd(scht1_handle, shtc1_cmd_measure);
}
int16_t shtc1_read(int32_t* temperature, int32_t* humidity) {
    uint16_t words[2];
    int16_t ret = sensirion_i2c_read_words(scht1_handle, words,
                                           SENSIRION_NUM_WORDS(words));
    /**
     * formulas for conversion of the sensor signals, optimized for fixed point
     * algebra:
     * Temperature = 175 * S_T / 2^16 - 45
     * Relative Humidity = 100 * S_RH / 2^16
     */
    *temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;
    *humidity = ((12500 * (int32_t)words[1]) >> 13);

    return ret;
}


esp_err_t shtc1_measure_blocking_read(int32_t* temperature, int32_t* humidity) {
    esp_err_t ret;

    ret = shtc1_measure();
    if (ret)
        return ret;
#if !defined(USE_SENSIRION_CLOCK_STRETCHING) || !USE_SENSIRION_CLOCK_STRETCHING
    vTaskDelay(pdTICKS_TO_MS(SHTC1_MEASUREMENT_DURATION_USEC));
#endif /* USE_SENSIRION_CLOCK_STRETCHING */
    return shtc1_read(temperature, humidity);
}


uint32_t sensirion_calc_absolute_humidity(int32_t temperature_milli_celsius,
                                          int32_t humidity_milli_percent) {
    uint32_t t, i, rem, ret;

    if (humidity_milli_percent <= 0)
        return 0;

    if (temperature_milli_celsius < T_LO)
        t = 0;
    else
        t = (uint32_t)(temperature_milli_celsius - T_LO);

    i = t / T_STEP;
    rem = t % T_STEP;

    if (i >= ARRAY_SIZE(AH_LUT_100RH) - 1) {
        ret = AH_LUT_100RH[ARRAY_SIZE(AH_LUT_100RH) - 1];

    } else if (rem == 0) {
        ret = AH_LUT_100RH[i];

    } else {
        ret = (AH_LUT_100RH[i] +
               ((AH_LUT_100RH[i + 1] - AH_LUT_100RH[i]) * rem / T_STEP));
    }

    // Code is mathematically (but not numerically) equivalent to
    //    return (ret * (humidity_milli_percent)) / 100000;
    // Maximum ret = 198277 (Or last entry from AH_LUT_100RH)
    // Maximum humidity_milli_percent = 119000 (theoretical maximum)
    // Multiplication might overflow with a maximum of 3 digits
    // Trick: ((ret >> 3) * (uint32_t)humidity_milli_percent) never overflows
    // Now we only need to divide by 12500, as the tripple righ shift
    // divides by 8

    return ((ret >> 3) * (uint32_t)(humidity_milli_percent)) / 12500;
}

esp_err_t sgp30_set_absolute_humidity(uint32_t absolute_humidity) {
    esp_err_t ret;
    uint16_t ah_scaled;

    if (absolute_humidity > 256000)
        return ESP_FAIL;

    /* ah_scaled = (absolute_humidity / 1000) * 256 */
    ah_scaled = (uint16_t)((absolute_humidity * 16777) >> 16);

    ret = sensirion_i2c_write_cmd_with_args(svm30_handle, SGP30_CMD_SET_ABSOLUTE_HUMIDITY, &ah_scaled,
        SENSIRION_NUM_WORDS(ah_scaled));

    
    vTaskDelay(pdTICKS_TO_MS(SGP30_CMD_SET_ABSOLUTE_HUMIDITY_DURATION_US));

    return ret;
}


esp_err_t svm_set_humidity(const int32_t* temperature,
                                const int32_t* humidity) {
    uint32_t absolute_humidity;

    absolute_humidity =
        sensirion_calc_absolute_humidity(*temperature, *humidity);

    if (absolute_humidity == 0)
        absolute_humidity = 1; /* avoid disabling humidity compensation */

    return sgp30_set_absolute_humidity(absolute_humidity);
}


esp_err_t sgp30_read_iaq(uint16_t* tvoc_ppb, uint16_t* co2_eq_ppm) {
    esp_err_t ret;
    uint16_t words[SGP30_CMD_IAQ_MEASURE_WORDS];

    ret = sensirion_i2c_read_words(svm30_handle, words,
                                   SGP30_CMD_IAQ_MEASURE_WORDS);

    *tvoc_ppb = words[1];
    *co2_eq_ppm = words[0];

    return ret;
}

int16_t sgp30_measure_iaq() {
    return sensirion_i2c_write_cmd(svm30_handle, SGP30_CMD_IAQ_MEASURE);
}
esp_err_t sgp30_measure_iaq_blocking_read(uint16_t* tvoc_ppb,
                                        uint16_t* co2_eq_ppm) {
    esp_err_t ret;

    ret = sgp30_measure_iaq();
    if (ret != ESP_OK)
        return ret;

   
    vTaskDelay(pdTICKS_TO_MS(SGP30_CMD_IAQ_MEASURE_DURATION_US));

    return sgp30_read_iaq(tvoc_ppb, co2_eq_ppm);
}

static void svm_compensate_rht(int32_t* temperature, int32_t* humidity) {
    *temperature = ((*temperature * 8225) >> 13) - 500;
    *humidity = (*humidity * 8397) >> 13;
}

esp_err_t svm_measure_iaq_blocking_read(uint16_t* tvoc_ppb, uint16_t* co2_eq_ppm,
                                      int32_t* temperature, int32_t* humidity) {
    esp_err_t err;

    err = shtc1_measure_blocking_read(temperature, humidity);
    if (err != ESP_OK)
        return err;

    err = svm_set_humidity(temperature, humidity);
    if (err != ESP_OK)
        return err;

    svm_compensate_rht(temperature, humidity);

    err = sgp30_measure_iaq_blocking_read(tvoc_ppb, co2_eq_ppm);
    if (err != ESP_OK)
        return err;

    return ESP_OK;
}

esp_err_t sgp30_get_iaq_baseline(uint32_t* baseline) {
    esp_err_t ret;
    uint16_t words[SGP30_CMD_GET_IAQ_BASELINE_WORDS];

    ret =sensirion_i2c_write_cmd(svm30_handle, SGP30_CMD_GET_IAQ_BASELINE);

    if (ret != ESP_OK)
        return ret;


    vTaskDelay(pdTICKS_TO_MS(SGP30_CMD_GET_IAQ_BASELINE_DURATION_US));

    ret = sensirion_i2c_read_words(svm30_handle, words,
                                   SGP30_CMD_GET_IAQ_BASELINE_WORDS);

    if (ret != ESP_OK)
        return ret;

    *baseline = ((uint32_t)words[1] << 16) | ((uint32_t)words[0]);

    if (*baseline)
        return ESP_OK;
    return ESP_FAIL;
}


/************************ */
esp_err_t SVM30_transmit_receive(i2c_master_dev_handle_t i2c_handle, uint8_t *send_buff, size_t send_buff_len,uint8_t *recive_buff, size_t recive_data_len,uint8_t wait) {
    // Logging the I2C address
    #ifdef debug_svm30
    if(i2c_handle == rest_handle) {
        ESP_LOGI(TAG_SVM30, "I2C address 0x%02X :", REST_I2C_ADDRESS);
    } else if(i2c_handle == svm30_handle) {
        ESP_LOGI(TAG_SVM30, "I2C address 0x%02X :", SVM30_I2C_ADDRESS);
    } else if(i2c_handle == scht1_handle) {
        ESP_LOGI(TAG_SVM30, "I2C address 0x%02X :", SCHT1_I2C_ADDRESS);
    }
    // Log the transmitted data
    ESP_LOG_BUFFER_HEX(TAG_SVM30, send_buff, send_buff_len);
    #endif
    // Perform I2C transmit/receive operation
    esp_err_t err = i2c_master_transmit(i2c_handle, send_buff, send_buff_len,  2000);
    if (err != ESP_OK) {
        ESP_LOGE("TAG_SVM30", "I2C transmit failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(wait));

    err = i2c_master_receive(i2c_handle, recive_buff, recive_data_len, 2000);
    if (err != ESP_OK) {
        ESP_LOGE("TAG_SVM30", "I2C receive failed: %s", esp_err_to_name(err));
         return err;
    } else {
         #ifdef debug_svm30
        ESP_LOGI(TAG_SVM30, "I2C receive success :");
        ESP_LOG_BUFFER_HEX(TAG_SVM30, recive_buff, recive_data_len);
        #endif

    }
    return err;
}


esp_err_t probe1() {
    esp_err_t err=ESP_FAIL;
      // SGP30 has 3 words, SHTC1 has 1 word
        uint8_t send_buff[2] = {(SGP30_Read_ID >> 8) & 0xFF, SGP30_Read_ID & 0xFF};
        uint8_t recive_buff[6]={0};
        err=SVM30_transmit_receive(svm30_handle, &send_buff,sizeof(send_buff),&recive_buff,6,15);
        if (err != ESP_OK) {
        #ifdef debug_svm30_Values
        ESP_LOGE(TAG_SVM30, "SVM30 Error probe SVM30_transmit_receive");
        #endif
        }else{
            char  id[15];
            sprintf(id, "%02x%02x %02x%02x %02x%02x", recive_buff[0], recive_buff[1], recive_buff[2],recive_buff[3], recive_buff[4], recive_buff[5]);
            ESP_LOGI("TAG_SVM30", "SGP30 id : %s",(char *)id);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        // SHTC1 has 1 word
        uint8_t send_buff1[2] = {(SHTC1_Read_ID >> 8) & 0xFF, SHTC1_Read_ID & 0xFF};
        uint8_t recive_buff1[2]={0};
        err=SVM30_transmit_receive(scht1_handle, &send_buff1,sizeof(send_buff1),&recive_buff1,2,15);
        if (err != ESP_OK) {
        #ifdef debug_svm30_Values
        ESP_LOGE("TAG_SVM30", "SVM30 Error probe SVM30_transmit_receive");
        #endif
        }else{
            char id[15];
            sprintf(id, "%02x%02x", recive_buff1[0], recive_buff1[1]);
            ESP_LOGI("TAG_SVM30", "SHTC1 id :  %s",(char *)id);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    return err;
}

const char * GetDriverVersion() {
    return (SVM30_VERSION);
}

void read_featureSet(){
        esp_err_t err=ESP_FAIL;
       // SGP30 has 3 words, SHTC1 has 1 word
        uint8_t send_buff[2] = {(SGP30_Get_Feature_Set >> 8) & 0xFF, SGP30_Get_Feature_Set & 0xFF};
        uint8_t recive_buff[2]={0};
        err=SVM30_transmit_receive(svm30_handle, &send_buff,sizeof(send_buff),&recive_buff,2,10);
        if (err != ESP_OK) {
            ESP_LOGE("TAG_SVM30", "I2C receive failed: %s", esp_err_to_name(err));
        }else{
            ESP_LOGI("TAG_SVM30", "SGP30 product type : %02x ",(recive_buff[0] & 0xf0));
            ESP_LOGI("TAG_SVM30", "Product version : %02x ",(recive_buff[1]));
        }
}

void app_main(void)
{
    uint16_t i = 0;
    esp_err_t err;
    uint16_t tvoc_ppb, co2_eq_ppm;
    uint32_t iaq_baseline;
    int32_t temperature, humidity;

    /* Initialize I2C */
    i2c_init(); 
    vTaskDelay(pdTICKS_TO_MS(10));  // SVM30 / SGP30        sensirion_sleep_usec(1000000); == 1s
    printf("i2c_init\n");
    ESP_LOGI("TAG_SVM30","Driver version :%s ",GetDriverVersion());
      // try to detect SVM30 sensors
    while (probe1() !=ESP_OK){
         ESP_LOGI("TAG_SVM30","could not detect SVM30 sensors");
         vTaskDelay(pdMS_TO_TICKS(10));
    } 
    ESP_LOGI("TAG_SVM30","SVM30 detected");
    read_featureSet();
    printf("SVM30 module probing successful\n");
    
    /* Consider the two cases (A) and (B):
     * (A) If no baseline is available or the most recent baseline is more than
     *     one week old, it must discarded. A new baseline is found with
     *     sgp_iaq_init() */
    err = sgp30_iaq_init();
    /* (B) If a recent baseline is available, set it after sgp_iaq_init() for
     * faster start-up */
    /* IMPLEMENT: retrieve iaq_baseline from presistent storage;
     * err = sgp_set_iaq_baseline(iaq_baseline);
     */
    /* Run periodic IAQ measurements at defined intervals */
    while (1) {
        err = svm_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm,
                                            &temperature, &humidity);
        if (err == ESP_OK) {
            printf("tVOC  Concentration: %dppb\n", tvoc_ppb);
            printf("CO2eq Concentration: %dppm\n", co2_eq_ppm);
            printf("Temperature: %0.3fC\n", temperature / 1000.0f);
            printf("Humidity: %0.3f%%RH\n", humidity / 1000.0f);
        } else {
                printf("error reading sensor\n");
        }

        /* Persist the current baseline every hour */
        if (++i % 3600 == 3599) {
            err = sgp30_get_iaq_baseline(&iaq_baseline);
            if (err == ESP_OK) {
                /* IMPLEMENT: store baseline to presistent storage */
           }
           i=0;
        }
                                                              
        /* The IAQ measurement must be triggered exactly once per second (SGP30)
         * to get accurate values.
         */
        vTaskDelay(pdTICKS_TO_MS(10));  // SVM30 / SGP30        sensirion_sleep_usec(1000000); == 1s

    }
}
