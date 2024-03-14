#include <stdio.h>
#include "Wire.h"
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "SparkFunBME280.h"
#include "SparkFun_ENS160.h"
#include "TheThingsNetwork.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>

BME280 myBME;
SparkFun_ENS160 myENS;
int ensStatus;

const char *appEui = "0000000000000000";
// DevEUI
const char *devEui = "70B3D57ED800296C";
// AppKey
const char *appKey = "A934D9F4F289E188EDD8195A2C396D48";

// Queue handle
QueueHandle_t sensorDataQueue;

#define queueLength 1

// Task handles
TaskHandle_t sensorDataTaskHandle = NULL;
TaskHandle_t processDataTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

struct SensorData
{
  float humidity;
  float pressure;
  float altitude;
  float temp;
  uint8_t aqi;
  uint16_t tvoc;
  uint16_t co2;
  uint8_t gasStatus;
};

struct themsgData
{
  uint8_t buttonState;
  float humidity;
  float pressure;
  float altitude;
  float temp;
  uint8_t aqi;
  uint16_t tvoc;
  uint16_t co2;
  uint8_t gasStatus;
};

volatile uint8_t thebuttonState = 0;

// Pins and other resources
#define TTN_SPI_HOST SPI2_HOST
#define TTN_SPI_DMA_CHAN SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK 5
#define TTN_PIN_SPI_MOSI 27
#define TTN_PIN_SPI_MISO 19
#define TTN_PIN_NSS 18
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST 14
#define TTN_PIN_DIO0 26
#define TTN_PIN_DIO1 35
static TheThingsNetwork ttn;
const unsigned TX_INTERVAL = 30;

#define BUTTON_PIN 36

/**
 * @brief Function to send messages periodically.
 *
 * This function reads the state of a button, prepares the data to be sent, and transmits the message using TTN.
 * It also prints the button state and the result of the transmission.
 * The function runs in an infinite loop with a delay between each transmission.
 *
 * @param pvParameter A pointer to the parameter passed to the task (not used in this function).
 */
void sendMessages(void *pvParameter)
{
  while (1)
  {
    int buttonState = digitalRead(36);
    printf("Button state: %d\n", buttonState);

    // Prepare the data to be sent
    uint8_t msgData[1];
    msgData[0] = buttonState;

    printf("Sending message...\n");
    TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData));
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

    vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(50000));
  }
}

/**
 * @brief Handles the received message.
 *
 * This function is called when a message is received from the TTN network.
 * It prints the received message along with its length and port number.
 *
 * @param message Pointer to the received message.
 * @param length Length of the received message.
 * @param port Port number on which the message was received.
 */
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
  printf("Message of %d bytes received on port %d:", length, port);
  for (int i = 0; i < length; i++)
    printf(" %02x", message[i]);
  printf("\n");
}

/**
 * Initializes the sensor by setting up the serial communication, checking the sensor's response,
 * and configuring the operating mode of the indoor air quality sensor.
 */
void initSensor()
{
  Serial.begin(115200);
  Serial.println("Reading basic values from BME280");

  Wire.begin();

  if (myBME.beginI2C() == false)
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while (1)
      ; // Freeze
  }

  if (!myENS.begin())
  {
    Serial.println("Could not communicate with the ENS160, check wiring.");
    while (1)
      ;
  }

  // Reset the indoor air quality sensor's settings.
  if (myENS.setOperatingMode(SFE_ENS160_RESET))
    Serial.println("Ready.");

  delay(100);

  // Set to standard operation
  // Others include SFE_ENS160_DEEP_SLEEP and SFE_ENS160_IDLE
  myENS.setOperatingMode(SFE_ENS160_STANDARD);

  // There are four values here:
  // 0 - Operating ok: Standard Operation
  // 1 - Warm-up: occurs for 3 minutes after power-on.
  // 2 - Initial Start-up: Occurs for the first hour of operation.
  //												and only once in sensor's lifetime.
  // 3 - No Valid Output
  ensStatus = myENS.getFlags();

  Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
  Serial.println(ensStatus);
}

/**
 * @brief Puts the sensors into sleep mode.
 *
 * This function sets the BME sensor to sleep mode and the ENS sensor to deep sleep mode.
 * It is used to conserve power when the sensors are not actively being used.
 */
void sleepSensors()
{
  myBME.setMode(MODE_SLEEP);
  myENS.setOperatingMode(SFE_ENS160_DEEP_SLEEP);
}

/**
 * @brief Wakes up the sensors by setting their operating modes.
 *
 * This function sets the operating mode of the BME sensor to MODE_NORMAL
 * and the operating mode of the ENS sensor to SFE_ENS160_STANDARD.
 */
void wakeSensors()
{
  myBME.setMode(MODE_NORMAL);
  myENS.setOperatingMode(SFE_ENS160_STANDARD);
}

/**
 * @brief Task function that reads sensor data and sends it to a queue.
 *
 * This function continuously reads sensor data from the BME sensor and the ENS sensor,
 * and sends the data to a queue for further processing. It reads humidity, pressure,
 * altitude, and temperature from the BME sensor, and air quality index (AQI), total
 * volatile organic compounds (TVOC), carbon dioxide (CO2) concentration, and gas sensor
 * status from the ENS sensor. If the ENS sensor data is not available, it initializes
 * the data to default values. The function also prints the gas sensor status, AQI, TVOC,
 * and CO2 concentration to the serial monitor.
 *
 * @param parameter A pointer to the task parameter (not used in this function).
 */
void sensorDataTaskFunction(void *parameter)
{
  while (1)
  {
    // Read sensor data
    SensorData data;

    data.humidity = myBME.readFloatHumidity();
    data.pressure = myBME.readFloatPressure();
    data.altitude = myBME.readFloatAltitudeMeters();
    data.temp = myBME.readTempC();

    vTaskDelay(pdMS_TO_TICKS(20000));

    if (myENS.checkDataStatus())
    {
      data.aqi = myENS.getAQI();
      data.tvoc = myENS.getTVOC();
      data.co2 = myENS.getECO2();
      data.gasStatus = myENS.getFlags();

      Serial.print("Gas Sensor Status Flag: ");
      Serial.println(data.gasStatus);

      Serial.print("Air Quality Index: ");
      Serial.println(data.aqi);
      Serial.print("TVOC: ");
      Serial.println(data.tvoc);
      Serial.print("CO2 concentration: ");
      Serial.println(data.co2);
    }
    else
    {
      // Initialize to default values if sensor data is not available
      data.aqi = 0;
      data.tvoc = 0;
      data.co2 = 0;
      data.gasStatus = 0;
    }

    // Send sensor data to the queue
    xQueueSend(sensorDataQueue, &data, portMAX_DELAY);

    // sleepSensors();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Task function that checks the state of a button periodically.
 *
 * This function is responsible for checking the state of a button connected to BUTTON_PIN.
 * It sets the value of thebuttonState based on the button state.
 * The button state is checked every 3 seconds.
 *
 * @param parameter A pointer to any additional parameters that may be passed to the task function (not used in this case).
 */
void buttonTaskFunction(void *parameter)
{

  while (1)
  {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    printf("Checking button state...\n");
    if (digitalRead(BUTTON_PIN) == 1)
    {
      printf("Button pressed\n");

      vTaskDelay(pdMS_TO_TICKS(50));

      if (digitalRead(BUTTON_PIN) == 1)
      {
        thebuttonState = static_cast<uint8_t>(1);
      }
    }
    else
    {
      thebuttonState = static_cast<uint8_t>(0);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

/**
 * @brief Function to process sensor data and transmit it over TTN.
 *
 * This function waits for data to arrive in the sensorDataQueue, processes the received data,
 * prints the sensor values, prepares the data to be sent, transmits the data over TTN,
 * and then enters deep sleep for a specified interval.
 *
 * @param parameter A pointer to any additional parameters that may be passed to the function.
 */
void processDataTaskFunction(void *parameter)
{
  while (1)
  {
    // Wait for data to arrive in the queue
    SensorData receivedData;
    xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY);

    // Process and print sensor data
    Serial.print("Humidity: ");
    Serial.println(receivedData.humidity, 2);
    Serial.print("Pressure: ");
    Serial.println(receivedData.pressure, 2);
    Serial.print("Altitude: ");
    Serial.println(receivedData.altitude, 2);
    Serial.print("Temperature: ");
    Serial.println(receivedData.temp, 2);
    Serial.print("Air Quality Index: ");
    Serial.println(receivedData.aqi);
    Serial.print("TVOC: ");
    Serial.println(receivedData.tvoc);
    Serial.print("CO2 concentration: ");
    Serial.println(receivedData.co2);
    Serial.print("Gas Sensor Status Flag: ");
    Serial.println(receivedData.gasStatus);
    Serial.print("The button state is: ");
    Serial.println(thebuttonState);
    Serial.println();
    // Add additional processing or actions here

    // Prepare the data to be sent
    themsgData message;
    message.buttonState = thebuttonState;
    message.humidity = receivedData.humidity;
    message.pressure = receivedData.pressure;
    message.altitude = receivedData.altitude;
    message.temp = receivedData.temp;
    message.aqi = receivedData.aqi;
    message.tvoc = receivedData.tvoc;
    message.co2 = receivedData.co2;
    message.gasStatus = receivedData.gasStatus;

    sleepSensors();

    printf("Sending message...\n");
    TTNResponseCode res = ttn.transmitMessage((uint8_t *)&message, sizeof(themsgData));
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

    // enter deep sleep
    esp_deep_sleep(TX_INTERVAL * 1000000LL);
  }
}

/**
 * @brief Entry point of the application.
 *
 * This function is the entry point of the application. It initializes the necessary components,
 * configures the pins, joins the network, and creates tasks for sensor data, button, and data processing.
 *
 * @note This function should be called only once at the start of the application.
 */
extern "C" void app_main()
{
  initSensor();
  sleepSensors();

  esp_err_t err;
  // Initialize the GPIO ISR handler service
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  ESP_ERROR_CHECK(err);

  // Initialize the NVS (non-volatile storage) for saving and restoring the keys
  err = nvs_flash_init();
  ESP_ERROR_CHECK(err);

  // Initialize SPI bus
  spi_bus_config_t spi_bus_config;
  memset(&spi_bus_config, 0, sizeof(spi_bus_config));
  spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
  spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
  spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
  err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
  ESP_ERROR_CHECK(err);

  // Configure the SX127x pins

  ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

  // The below line can be commented after the first run as the data is saved in NVS
  // ttn.provision(devEui, appEui, appKey);

  // Register callback for received messages
  ttn.onMessage(messageReceived);

  pinMode(36, INPUT_PULLUP);
  printf("Joining...\n");
  if (ttn.join())
  {
    printf("Joined.\n");

    wakeSensors();

    // xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
    sensorDataQueue = xQueueCreate(queueLength, sizeof(SensorData));
    xTaskCreate(sensorDataTaskFunction, "sensor_data_task", 1024 * 4, NULL, 2, &sensorDataTaskHandle);
    xTaskCreate(buttonTaskFunction, "button_task", 1024 * 4, NULL, 2, &buttonTaskHandle);
    xTaskCreate(processDataTaskFunction, "process_data_task", 1024 * 4, NULL, 2, &processDataTaskHandle);
  }
  else
  {
    printf("Join failed. Goodbye\n");
  }
}