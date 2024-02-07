set(COMPONENT_REQUIRES "BME280")
set(COMPONENT_REQUIRES "ENS160")
set(COMPONENT_REQUIRES "ttn-esp32")
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
)
