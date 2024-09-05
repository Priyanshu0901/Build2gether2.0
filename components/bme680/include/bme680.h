#ifndef BME680_H
#define BME680_H

#include <stdbool.h>

#define I2C_PORT_0_HOST 0

#define I2C_PORT_0_SDA 5
#define I2C_PORT_0_SCL 6

#define DEVICE_ADDRESS 0x76

#define CHIP_ID 0x61

#define BME680_OS_16X BME68X_OS_16X   ///< Alias for BME680 existing examples
#define BME680_OS_8X BME68X_OS_8X     ///< Alias for BME680 existing examples
#define BME680_OS_4X BME68X_OS_4X     ///< Alias for BME680 existing examples
#define BME680_OS_2X BME68X_OS_2X     ///< Alias for BME680 existing examples
#define BME680_OS_1X BME68X_OS_1X     ///< Alias for BME680 existing examples
#define BME680_OS_NONE BME68X_OS_NONE ///< Alias for BME680 existing examples

#define BME680_FILTER_SIZE_127 \
    BME68X_FILTER_SIZE_127 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_63 \
    BME68X_FILTER_SIZE_63 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_31 \
    BME68X_FILTER_SIZE_31 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_15 \
    BME68X_FILTER_SIZE_15 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_7 \
    BME68X_FILTER_SIZE_7 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_3 \
    BME68X_FILTER_SIZE_3 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_1 \
    BME68X_FILTER_SIZE_1 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_0 \
    BME68X_FILTER_OFF ///< Alias for BME680 existing examples

typedef struct Result
{
    float temperature;

    float pressure;

    float humidity;

    uint32_t gas;

} sensor_result_t;


extern sensor_result_t getBME680data();
bool init_bme680(bool);

#endif