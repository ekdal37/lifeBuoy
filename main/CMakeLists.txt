set(COMPONENT_REQUIRES "SparkFun_BME280_Arduino_Library")
set(COMPONENT_REQUIRES "SparkFun_Indoor_Air_Quality_Sensor-ENS160_Arduino_Library")
set(COMPONENT_REQUIRES "ttn-esp32")
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
)
