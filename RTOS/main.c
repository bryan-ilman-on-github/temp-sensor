#include <asf.h>
#include <util/delay.h>
#include <stdio.h>

// FreeRTOS includes with specified path.
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/queue.h"

// Buffer for LCD output.
static char strbuf[128];

// Define DHT22 data pin.
#define DHT22_PIN PIN0_bm  // Using PORTC PIN0 for data.

// Structure to hold sensor data.
typedef struct {
	uint16_t temperature; // Raw temperature data.
	uint16_t humidity;    // Raw humidity data.
	uint8_t error;        // 0: No Error, 1: Timeout/Error, 2: Checksum Error.
} sensor_data_t;

// Queue handle for sensor data.
QueueHandle_t xSensorQueue;

// Function Prototypes.
void dht22_init(void);
void dht22_request(void);
uint8_t dht22_read(void);
sensor_data_t dht22_read_data(void);
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);

// Initialize the DHT22 data pin.
void dht22_init(void) {
	PORTC.DIRSET = DHT22_PIN;  // Set PIN0 as output.
	PORTC.OUTSET = DHT22_PIN;  // Set pin high.
}

// Send start signal to DHT22.
void dht22_request(void) {
	PORTC.DIRSET = DHT22_PIN;   // Set pin as output.
	PORTC.OUTCLR = DHT22_PIN;   // Pull pin low.
	_delay_ms(20);               // Wait for at least 18ms.
	PORTC.OUTSET = DHT22_PIN;   // Pull pin high.
	_delay_us(40);               // Wait 20-40us.
	PORTC.DIRCLR = DHT22_PIN;   // Set pin as input to read data.
}

// Read 8 bits from DHT22 with timeout.
uint8_t dht22_read(void) {
	uint8_t result = 0;
	for (int i = 0; i < 8; i++) {
		uint16_t timeout = 0;

		// Wait for pin to go high.
		while (!(PORTC.IN & DHT22_PIN)) {
			_delay_us(1);
			if (++timeout > 1000) {
				return 0xFF;  // Indicate a timeout error.
			}
		}

		// Measure the width of the high signal.
		_delay_us(30);
		if (PORTC.IN & DHT22_PIN) {
			result |= (1 << (7 - i));  // If high after 30us, it's a 1.
		}

		// Wait for pin to go low.
		timeout = 0;
		while (PORTC.IN & DHT22_PIN) {
			_delay_us(1);
			if (++timeout > 1000) {
				return 0xFF;  // Indicate a timeout error.
			}
		}
	}
	return result;
}

// Read temperature and humidity from DHT22.
sensor_data_t dht22_read_data(void) {
	sensor_data_t data = {0, 0, 0};
	uint8_t hum_high, hum_low, temp_high, temp_low, checksum;
	uint16_t timeout;

	dht22_request();  // Send request to DHT22.

	// Wait for sensor's response with timeouts.
	timeout = 0;
	while (PORTC.IN & DHT22_PIN) {  // Wait for pin to go low.
		_delay_us(1);
		if (++timeout > 1000) {
			data.error = 1;  // Timeout Error.
			return data;
		}
	}

	timeout = 0;
	while (!(PORTC.IN & DHT22_PIN)) {  // Wait for pin to go high.
		_delay_us(1);
		if (++timeout > 1000) {
			data.error = 1;  // Timeout Error.
			return data;
		}
	}

	timeout = 0;
	while (PORTC.IN & DHT22_PIN) {  // Wait for pin to go low.
		_delay_us(1);
		if (++timeout > 1000) {
			data.error = 1;  // Timeout Error.
			return data;
		}
	}

	// Read all 5 bytes (humidity high, humidity low, temp high, temp low, checksum).
	hum_high = dht22_read();
	if (hum_high == 0xFF) {
		data.error = 1;
		return data;
	}

	hum_low = dht22_read();
	if (hum_low == 0xFF) {
		data.error = 1;
		return data;
	}

	temp_high = dht22_read();
	if (temp_high == 0xFF) {
		data.error = 1;
		return data;
	}

	temp_low = dht22_read();
	if (temp_low == 0xFF) {
		data.error = 1;
		return data;
	}

	checksum = dht22_read();
	if (checksum == 0xFF) {
		data.error = 1;
		return data;
	}

	// Combine high and low bytes.
	data.humidity = (hum_high << 8) | hum_low;
	data.temperature = (temp_high << 8) | temp_low;

	// Calculate checksum correctly (only lowest 8 bits matter).
	uint8_t calc_checksum = (hum_high + hum_low + temp_high + temp_low) & 0xFF;

	if (calc_checksum != checksum) {
		data.error = 2;  // Checksum Error.
	}

	return data;
}

// Task to read sensor data periodically.
void SensorTask(void *pvParameters) {
	sensor_data_t sensorData;

	dht22_init();  // Initialize DHT22 sensor.

	while (1) {
		sensorData = dht22_read_data();

		// Send sensor data to the queue.
		if (xQueueSend(xSensorQueue, &sensorData, (TickType_t)10) != pdPASS) {
			// Optionally handle queue send failure (e.g., log error).
		}

		// Wait for 1 second before next read to match original behavior.
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// Task to display data on LCD.
void DisplayTask(void *pvParameters) {
	sensor_data_t receivedData;

	while (1) {
		// Wait indefinitely for data from the queue.
		if (xQueueReceive(xSensorQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
			if (receivedData.error == 0) {
				// Convert raw values to human-readable format (integer display).
				int16_t temp_celsius = receivedData.temperature;
				int16_t hum_percent = receivedData.humidity;

				// Handle negative temperatures.
				if (temp_celsius & 0x8000) {
					temp_celsius = -(temp_celsius & 0x7FFF);
				}

				temp_celsius /= 10;
				hum_percent /= 10;

				// Prepare temperature string with padding to overwrite previous content.
				snprintf(strbuf, sizeof(strbuf), "Temp: %d C  ", temp_celsius);
				gfx_mono_draw_string(strbuf, 0, 10, &sysfont);  // Temperature.

				// Prepare humidity string with padding to overwrite previous content.
				snprintf(strbuf, sizeof(strbuf), "Humid: %d %% ", hum_percent);
				gfx_mono_draw_string(strbuf, 0, 20, &sysfont);  // Humidity.
			}
			else if (receivedData.error == 1) {
				// Timeout or Read Error.
				gfx_mono_draw_string("Timeout/Error   ", 0, 10, &sysfont);
				gfx_mono_draw_string("Humid: -- %    ", 0, 20, &sysfont);
			}
			else if (receivedData.error == 2) {
				// Checksum Error.
				gfx_mono_draw_string("Checksum Error ", 0, 10, &sysfont);
				gfx_mono_draw_string("Humid: -- %    ", 0, 20, &sysfont);
			}
		}
	}
}

int main(void) {
	uint16_t temperature = 0, humidity = 0;

	// Initialize system, board, and LCD.
	sysclk_init();
	board_init();
	gfx_mono_init();
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	// Clear the LCD before starting.
	gfx_mono_draw_filled_rect(0, 0, GFX_MONO_LCD_WIDTH, GFX_MONO_LCD_HEIGHT, GFX_PIXEL_CLR);

	// Display static header once.
	gfx_mono_draw_string("Bryan dan Alvaro", 0, 0, &sysfont);

	// Display initial placeholders.
	gfx_mono_draw_string("Temp: -- C  ", 0, 10, &sysfont);
	gfx_mono_draw_string("Humid: -- % ", 0, 20, &sysfont);

	// Create a queue capable of holding 10 sensor_data_t structures.
	xSensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
	if (xSensorQueue == NULL) {
		// Handle queue creation failure (e.g., halt the system).
		while (1);
	}

	// Create Display Task.
	if (xTaskCreate(
	DisplayTask,          // Task function.
	"Display Task",       // Task name.
	512,                  // Stack size (in words).
	NULL,                 // Task input parameter.
	2,                    // Priority.
	NULL ) != pdPASS) {   // Task handle.
		// Handle task creation failure (e.g., halt the system).
		while (1);
	}

	// Create Sensor Task.
	if (xTaskCreate(
	SensorTask,           // Task function.
	"Sensor Task",        // Task name.
	256,                  // Stack size (in words).
	NULL,                 // Task input parameter.
	1,                    // Priority.
	NULL ) != pdPASS) {   // Task handle.
		// Handle task creation failure (e.g., halt the system).
		while (1);
	}

	// Start the scheduler so the tasks start executing.
	vTaskStartScheduler();

	// If all is well, the scheduler will now be running, and the following line will never be reached.
	// If the following line does execute, then there was insufficient FreeRTOS heap memory available.
	while (1);
}
