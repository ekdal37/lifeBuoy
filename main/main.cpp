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

BME280 mySensor;
SparkFun_ENS160 myENS;
int ensStatus;

const char *appEui = "0000000000000000";
// DevEUI
const char *devEui = "70B3D57ED800296C";
// AppKey
const char *appKey = "A934D9F4F289E188EDD8195A2C396D48";


// Queue handle
QueueHandle_t sensorDataQueue;

# define queueLength 1

// Task handles
TaskHandle_t sensorDataTaskHandle = NULL;
TaskHandle_t processDataTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

// mmm very good safe data sturcture.
struct SensorData {
    float humidity;
    float pressure;
    float altitude;
    float temp;
    uint8_t aqi;
    uint16_t tvoc;
    uint16_t co2;
    uint8_t gasStatus;
};

struct themsgData {
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

void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
{
	printf("Message of %d bytes received on port %d:", length, port);
	for (int i = 0; i < length; i++)
		printf(" %02x", message[i]);
	printf("\n");
}

void initSensor()
{
	Serial.begin(115200);
	Serial.println("Reading basic values from BME280");
	Serial.println("hello world\n");

	Wire.begin();

	if (mySensor.beginI2C() == false)
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

	// Device needs to be set to idle to apply any settings.
	// myENS.setOperatingMode(SFE_ENS160_IDLE);

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

void sensorDataTaskFunction(void *parameter) {
    while (1) {
        // Read sensor data
        SensorData data;

        data.humidity = mySensor.readFloatHumidity();
        data.pressure = mySensor.readFloatPressure();
        data.altitude = mySensor.readFloatAltitudeMeters();
        data.temp = mySensor.readTempC();

        if (myENS.checkDataStatus()) {
            data.aqi = myENS.getAQI();
            data.tvoc = myENS.getTVOC();
            data.co2 = myENS.getECO2();
            data.gasStatus = myENS.getFlags();
        } else {
            // Initialize to default values if sensor data is not available
            data.aqi = 0;
            data.tvoc = 0;
            data.co2 = 0;
            data.gasStatus = 0;
        }

        // Send sensor data to the queue
        xQueueSend(sensorDataQueue, &data, portMAX_DELAY);

        // Delay before reading the next set of sensor data
        vTaskDelay(pdMS_TO_TICKS(50000));
    }
}

void buttonTaskFunction(void *parameter) {
		while (1)
		{
			// redundacey for the button state			
			thebuttonState = static_cast<uint8_t>(digitalRead(36));

			vTaskDelay(pdMS_TO_TICKS(5000));
		}
}

// Task that processes sensor data
void processDataTaskFunction(void *parameter) {
    while (1) {
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

				printf("Sending message...\n");
				TTNResponseCode res = ttn.transmitMessage((uint8_t *)&message, sizeof(themsgData));
				printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

        // Optional delay before processing the next set of data
        // esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000); // Convert seconds to microseconds

        mySensor.setMode(MODE_SLEEP); //Sleep for now
        myENS.setOperatingMode(SFE_ENS160_DEEP_SLEEP);

        esp_deep_sleep(TX_INTERVAL * 1000000LL);

        mySensor.setMode(MODE_FORCED); //Wake up sensor and take reading
        myENS.setOperatingMode(SFE_ENS160_STANDARD);

        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL * 20000));

    }
}

extern "C" void app_main()
{
	initSensor();
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
  if (ttn.join()) {
    printf("Joined.\n");
    // xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
		sensorDataQueue = xQueueCreate(queueLength, sizeof(SensorData));
		xTaskCreate(sensorDataTaskFunction, "sensor_data_task", 1024 * 4, NULL, 5, &sensorDataTaskHandle);
		xTaskCreate(buttonTaskFunction, "button_task", 1024 * 4, NULL, 5, &buttonTaskHandle);
		xTaskCreate(processDataTaskFunction, "process_data_task", 1024 * 4, NULL, 5, &processDataTaskHandle);
  } else {
    printf("Join failed. Goodbye\n");
  }	
}