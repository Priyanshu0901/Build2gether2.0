#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bme68x.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "bme680.h"

float readTemperature();
float readPressure();
float readHumidity();
uint32_t readGas();
float readAltitude(float seaLevel);

bool setTemperatureOversampling(uint8_t os);
bool setPressureOversampling(uint8_t os);
bool setHumidityOversampling(uint8_t os);
bool setIIRFilterSize(uint8_t fs);
bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);
bool setODR(uint8_t odr);

// Perform a reading in blocking mode.
bool performReading();

uint32_t beginReading();

bool endReading();

int remainingReadingMillis();

static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface);
static void delay_usec(uint32_t us, void *intf_ptr);

/** Value returned by remainingReadingMillis indicating no asynchronous
 * reading has been initiated by beginReading. **/
static const int reading_not_started = -1;
/** Value returned by remainingReadingMillis indicating asynchronous reading
 * is complete and calling endReading will not block. **/
static const int reading_complete = 0;

typedef struct
{
    uint8_t port;
    uint8_t dev_address;
} i2c_config_handle_t;

struct bme68x_dev bme_dev_handle;
struct bme68x_conf bme_dev_config;
struct bme68x_heatr_conf bme_heater_config;
sensor_result_t bme_result;

int32_t _sensorID;
uint32_t _meas_start = 0;
uint16_t _meas_period = 0;

void initialize_i2c_port(void)
{
    ESP_LOGI("I2C_Interface", "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PORT_0_SDA,
        .scl_io_num = I2C_PORT_0_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_0_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_0_HOST, I2C_MODE_MASTER, 0, 0, 0));
}

/*!
 *  @brief  Performs a reading and returns the ambient temperature.
 *  @return Temperature in degrees Centigrade
 */
float readTemperature(void)
{
    performReading();
    return bme_result.temperature;
}

/*!
 *  @brief Performs a reading and returns the barometric pressure.
 *  @return Barometic pressure in Pascals
 */
float readPressure(void)
{
    performReading();
    return bme_result.pressure;
}

/*!
 *  @brief  Performs a reading and returns the relative humidity.
 *  @return Relative humidity as floating point
 */
float readHumidity(void)
{
    performReading();
    return bme_result.humidity;
}

/*!
 *  @brief Calculates the resistance of the MOX gas sensor.
 *  @return Resistance in Ohms
 */
uint32_t readGas(void)
{
    performReading();
    return bme_result.gas;
}

/*!
 *  @brief  Calculates the altitude (in meters).
 *          Reads the current atmostpheric pressure (in hPa) from the sensor and
 * calculates via the provided sea-level pressure (in hPa).
 *  @param  seaLevel
 *          Sea-level pressure in hPa
 *  @return Altitude in meters
 */
float readAltitude(float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.

    float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}
/*!
 *  @brief  Performs a full reading of all 4 sensors in the BME680.
 *          Assigns the internal BME680#temperature,
 * BME680#pressure, BME680#humidity and
 * BME680#gas_resistance member variables
 *  @return True on success, False on failure
 */
bool performReading(void) { return endReading(); }

/*! @brief Begin an asynchronous reading.
 *  @return When the reading would be ready as absolute time in millis().
 */
uint32_t beginReading(void)
{
    if (_meas_start != 0)
    {
        /* A measurement is already in progress */
        return _meas_start + _meas_period;
    }

    int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_dev_handle);

    if (rslt != BME68X_OK)
        return false;

    /* Calculate delay period in microseconds */
    uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(
                                  BME68X_FORCED_MODE, &bme_dev_config, &bme_dev_handle) +
                              ((uint32_t)bme_heater_config.heatr_dur * 1000);

    _meas_start = esp_timer_get_time();
    _meas_period = delayus_period / 1000;

    return _meas_start + _meas_period;
}

/*! @brief  End an asynchronous reading.
 *          If the asynchronous reading is still in progress, block until it
 * ends. If no asynchronous reading has started, this is equivalent to
 * performReading().
 *  @return Whether success.
 */
bool endReading(void)
{
    uint32_t meas_end = beginReading();

    if (meas_end == 0)
    {
        return false;
    }

    int remaining_millis = remainingReadingMillis();

    if (remaining_millis > 0)
    {

        vTaskDelay(pdMS_TO_TICKS((remaining_millis) * 2)); /* Delay till the measurement is ready */
    }
    _meas_start = 0; /* Allow new measurement to begin */
    _meas_period = 0;

    struct bme68x_data data;
    uint8_t n_fields;

    int8_t rslt =
        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme_dev_handle);

    if (rslt != BME68X_OK)
        return false;

    if (n_fields)
    {
        bme_result.temperature = data.temperature;
        bme_result.humidity = data.humidity;
        bme_result.pressure = data.pressure;

        if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK))
        {
            bme_result.gas = data.gas_resistance;
        }
        else
        {
            bme_result.gas = 0;
        }
    }

    return true;
}

/*! @brief  Get remaining time for an asynchronous reading.
 *          If the asynchronous reading is still in progress, how many millis
 * until its completion. If the asynchronous reading is completed, 0. If no
 * asynchronous reading has started, -1 or
 * reading_not_started. Does not block.
 *  @return Remaining millis until endReading will not block if invoked.
 */
int remainingReadingMillis(void)
{
    if (_meas_start != 0)
    {
        /* A measurement is already in progress */
        int remaining_time = (int)_meas_period - (esp_timer_get_time() - _meas_start);
        return remaining_time < 0 ? reading_complete : remaining_time;
    }
    return reading_not_started;
}

/*!
 *  @brief  Enable and configure gas reading + heater
 *  @param  heaterTemp
 *          Desired temperature in degrees Centigrade
 *  @param  heaterTime
 *          Time to keep heater on in milliseconds
 *  @return True on success, False on failure
 */
bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime)
{

    if ((heaterTemp == 0) || (heaterTime == 0))
    {
        bme_heater_config.enable = BME68X_DISABLE;
    }
    else
    {
        bme_heater_config.enable = BME68X_ENABLE;
        bme_heater_config.heatr_temp = heaterTemp;
        bme_heater_config.heatr_dur = heaterTime;
    }

    int8_t rslt =
        bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme_heater_config, &bme_dev_handle);

    return rslt == 0;
}

/*!
 *  @brief  Setter for Output Data Rate
 *  @param  odr
 *          Output data rate setting, can be BME68X_ODR_NONE,
 * BME68X_ODR_0_59_MS, BME68X_ODR_10_MS, BME68X_ODR_20_MS, BME68X_ODR_62_5_MS,
 * BME68X_ODR_125_MS, BME68X_ODR_250_MS, BME68X_ODR_500_MS, BME68X_ODR_1000_MS
 *  @return True on success, False on failure
 */

bool setODR(uint8_t odr)
{
    if (odr > BME68X_ODR_NONE)
        return false;

    bme_dev_config.odr = odr;

    int8_t rslt = bme68x_set_conf(&bme_dev_config, &bme_dev_handle);

    return rslt == 0;
}

/*!
 *  @brief  Setter for Temperature oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Temperature
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */

bool setTemperatureOversampling(uint8_t oversample)
{
    if (oversample > BME68X_OS_16X)
        return false;

    bme_dev_config.os_temp = oversample;

    int8_t rslt = bme68x_set_conf(&bme_dev_config, &bme_dev_handle);

    return rslt == 0;
}

/*!
 *  @brief  Setter for Humidity oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Humidity
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool setHumidityOversampling(uint8_t oversample)
{
    if (oversample > BME68X_OS_16X)
        return false;

    bme_dev_config.os_hum = oversample;

    int8_t rslt = bme68x_set_conf(&bme_dev_config, &bme_dev_handle);

    return rslt == 0;
}

/*!
 *  @brief  Setter for Pressure oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Pressure
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool setPressureOversampling(uint8_t oversample)
{
    if (oversample > BME68X_OS_16X)
        return false;

    bme_dev_config.os_pres = oversample;

    int8_t rslt = bme68x_set_conf(&bme_dev_config, &bme_dev_handle);

    return rslt == 0;
}

/*!
 *  @brief  Setter for IIR filter.
 *  @param  filtersize
 *          Size of the filter (in samples).
 *          Can be BME68X_FILTER_SIZE_0 (no filtering), BME68X_FILTER_SIZE_1,
 * BME68X_FILTER_SIZE_3, BME68X_FILTER_SIZE_7, BME68X_FILTER_SIZE_15,
 * BME68X_FILTER_SIZE_31, BME68X_FILTER_SIZE_63, BME68X_FILTER_SIZE_127
 *  @return True on success, False on failure
 */
bool setIIRFilterSize(uint8_t filtersize)
{
    if (filtersize > BME68X_FILTER_SIZE_127)
        return false;
    bme_dev_config.filter = filtersize;

    int8_t rslt = bme68x_set_conf(&bme_dev_config, &bme_dev_handle);

    return rslt == 0;
}

void print_temp(void *pvParameters)
{
    for (;;)
    {
        performReading();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

sensor_result_t getBME680data(){
    return bme_result;
}
bool init_bme680(bool i2c_to_init)
{
    if (i2c_to_init)
    {
        initialize_i2c_port();
    }
    i2c_config_handle_t dev_i2c;
    dev_i2c.dev_address = DEVICE_ADDRESS;
    dev_i2c.port = I2C_PORT_0_HOST;

    memset(&bme_dev_handle, 0, sizeof(bme_dev_handle));

    bme_dev_handle.read = &i2c_read;
    bme_dev_handle.write = &i2c_write;
    bme_dev_handle.intf_ptr = (void *)&dev_i2c;
    bme_dev_handle.chip_id = DEVICE_ADDRESS;
    bme_dev_handle.intf = BME68X_I2C_INTF;
    bme_dev_handle.amb_temp = 25; /* The ambient temperature in deg C is used for
                             defining the heater temperature */
    bme_dev_handle.delay_us = &delay_usec;

    int8_t rslt;
    rslt = bme68x_init(&bme_dev_handle);

    if (rslt != BME68X_OK)
        return false;

    setIIRFilterSize(BME68X_FILTER_SIZE_3);
    setODR(BME68X_ODR_NONE);
    setHumidityOversampling(BME68X_OS_2X);
    setPressureOversampling(BME68X_OS_4X);
    setTemperatureOversampling(BME68X_OS_8X);
    setGasHeater(320, 150); // 320*C for 150 ms

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme_dev_handle);
    if (rslt != BME68X_OK)
    {
        return false;
    }
    xTaskCreatePinnedToCore(
        print_temp,                   /* Function that implements the task. */
        "printDandT",                 /* Text name for the task. */
        configMINIMAL_STACK_SIZE * 2, /* Stack size in words, not bytes. */
        NULL,                         /* Parameter passed into the task. */
        tskIDLE_PRIORITY,             /* Priority at which the task is created. */
        NULL,                         /* Used to pass out the created task's handle. */
        0);                           /* Core ID */
    return true;
}

/*!
 *  @brief  Writes 8 bit values over I2C
 */
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *interface)
{

    // Check for valid inputs
    if (len == 0 || reg_data == NULL)
    {
        return -1; // Error: Invalid input parameters
    }
    (void)interface; // Unused parameter
    // i2c_config_handle_t *_i2c_dev = (i2c_config_handle_t *)interface;
    // Create I2C command handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start I2C transaction
    i2c_master_start(cmd);

    // Send device address and register address
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Write data
    i2c_master_write(cmd, reg_data, len, true);

    // Stop I2C transaction
    i2c_master_stop(cmd);

    // Execute command
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(1000));

    // Delete command handle
    i2c_cmd_link_delete(cmd);

    // Return bme_result
    return (int8_t)ret;
}

/*!
 *  @brief  Reads 8 bit values over I2C
 */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                void *interface)
{

    // Check for valid inputs
    if (len == 0 || reg_data == NULL)
    {
        return -1; // Error: Invalid input parameters
    }
    (void)interface; // Unused parameter
    // i2c_config_handle_t *_i2c_dev = (i2c_config_handle_t *)interface;

    // Create I2C command handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start I2C transaction
    i2c_master_start(cmd);

    // Send device address and register address
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Restart I2C transaction
    i2c_master_start(cmd);

    // Send device address for reading
    i2c_master_write_byte(cmd, (DEVICE_ADDRESS << 1) | I2C_MASTER_READ, true);

    // Read data
    if (len > 1)
    {
        i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
    }

    // Read last byte with NACK
    i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);

    // Stop I2C transaction
    i2c_master_stop(cmd);

    // Execute command
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(1000));

    // Delete command handle
    i2c_cmd_link_delete(cmd);

    // Return bme_result
    return (int8_t)ret;
}

static void delay_usec(uint32_t us, void *intf_ptr)
{
    (void)intf_ptr; // Unused parameter
    // Convert microseconds to milliseconds
    uint32_t t_ms = us / 1000;
    // Use vTaskDelay for ESP32-specific sleep
    vTaskDelay(pdMS_TO_TICKS(t_ms));
}
