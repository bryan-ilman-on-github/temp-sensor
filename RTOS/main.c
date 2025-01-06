#include <asf.h>
#include <util/delay.h>
#include <stdio.h>

// FreeRTOS includes with specified path.
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"


// Buffer for LCD output.
//static char strbuf[128];
static char strbuf[201];
static char reads[100];

// Define DHT22 data pin.
#define DHT22_PIN PIN0_bm  // Using PORTA PIN0 for data.

#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false

#define MY_ADC    ADCB
#define MY_ADC_CH ADC_CH0

// Structure to hold sensor data.
typedef struct {
	uint16_t temperature; // Raw temperature data.
	uint16_t humidity;    // Raw humidity data.
	uint8_t error;        // 0: No Error, 1: Timeout/Error, 2: Checksum Error.
} sensor_data_t;

// Queue handle for sensor data.
QueueHandle_t xSensorQueue;
QueueHandle_t xGasSensorQueue;

SemaphoreHandle_t xSemaphore;


// Function Prototypes.
void adc_init(void);
uint16_t adc_read(void);
void dht22_init(void);
void setUpSerial();
void sendChar(char c);
void sendString(char *text);
char receiveChar();
void dht22_request(void);
uint8_t dht22_read(void);
sensor_data_t dht22_read_data(void);
void SensorTask(void *pvParameters);
void GasSensorTask(void *pvParameters);
void PostDataTask(void *pvParameters);
void PWM_Init(void);
void ServoTask(void *pvParameters);

void PWM_Init(void)
{
	/* Set output */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = PIN1_bm; //(PIN2_bm) | (PIN0_bm);
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);
	
	/* Set Period */
	TCC0.PER = 2000;

	/* Set Compare Register value*/
	TCC0.CCA = 100;
}

void setUpSerial()
{
	// Baud rate selection
	// BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
	// FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
	
	USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
	USARTC0_BAUDCTRLA = 0x0C; // 12
	
	//USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	//USARTC0_BAUDCTRLA = 0xCF; // 207
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void sendChar(char c)
{
	
	while( !(USARTC0_STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
	
	USARTC0_DATA = c;
	
}

void sendString(char *text)
{
	while(*text)
	{
		//sendChar(*text++);
		usart_putchar(USART_SERIAL_EXAMPLE, *text++);
		//text++;
		delay_ms(20);
	}
}

char receiveChar()
{
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Wait until receive finish
	return USARTC0_DATA;
}

void adc_init(void) {
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adcch_set_input(&adcch_conf, J2_PIN0, ADCCH_NEG_NONE, 1);
	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

uint16_t adc_read(void) {
	uint16_t result;
	adc_enable(&MY_ADC);
	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
	result = adc_get_result(&MY_ADC, MY_ADC_CH);
	return result;
}

// Initialize the DHT22 data pin.
void dht22_init(void) {
	PORTA.DIRSET = DHT22_PIN;  // Set PIN0 as output.
	PORTA.OUTSET = DHT22_PIN;  // Set pin high.
}

// Send start signal to DHT22.
void dht22_request(void) {
	PORTA.DIRSET = DHT22_PIN;   // Set pin as output.
	PORTA.OUTCLR = DHT22_PIN;   // Pull pin low.
	_delay_ms(20);               // Wait for at least 18ms.
	PORTA.OUTSET = DHT22_PIN;   // Pull pin high.
	_delay_us(40);               // Wait 20-40us.
	PORTA.DIRCLR = DHT22_PIN;   // Set pin as input to read data.
}

// Read 8 bits from DHT22 with timeout.
uint8_t dht22_read(void) {
	uint8_t result = 0;
	for (int i = 0; i < 8; i++) {
		uint16_t timeout = 0;

		// Wait for pin to go high.
		while (!(PORTA.IN & DHT22_PIN)) {
			_delay_us(1);
			if (++timeout > 1000) {
				return 0xFF;  // Indicate a timeout error.
			}
		}

		// Measure the width of the high signal.
		_delay_us(30);
		if (PORTA.IN & DHT22_PIN) {
			result |= (1 << (7 - i));  // If high after 30us, it's a 1.
		}

		// Wait for pin to go low.
		timeout = 0;
		while (PORTA.IN & DHT22_PIN) {
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
	while (PORTA.IN & DHT22_PIN) {  // Wait for pin to go low.
		_delay_us(1);
		if (++timeout > 1000) {
			data.error = 1;  // Timeout Error.
			return data;
		}
	}

	timeout = 0;
	while (!(PORTA.IN & DHT22_PIN)) {  // Wait for pin to go high.
		_delay_us(1);
		if (++timeout > 1000) {
			data.error = 1;  // Timeout Error.
			return data;
		}
	}

	timeout = 0;
	while (PORTA.IN & DHT22_PIN) {  // Wait for pin to go low.
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

void GasSensorTask(void *pvParameters) {
	uint16_t sensorData;
	
	adc_init();
	
	while (1) {
		sensorData = adc_read();

		// Send sensor data to the queue.
		if (xQueueSend(xGasSensorQueue, &sensorData, (TickType_t)10) != pdPASS) {
			// Optionally handle queue send failure (e.g., log error).
		}

		// Wait for 1 second before next read to match original behavior.
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void GetDataTask(void *pvParameters){
	char receivedData;
	
	receivedData = receiveChar();
}

void ServoTask(void *pvParameters){
	sensor_data_t receivedTHData;
	uint16_t receivedGData;
	int16_t temp_celsius;
	int16_t hum_percent;
	uint16_t gas_value;
	
	PWM_Init();
	while (2) {
	if (xQueueReceive(xSensorQueue, &receivedTHData, portMAX_DELAY) == pdTRUE) {
		if (receivedTHData.error == 0) {
			// Convert raw values to human-readable format (integer display).
			int16_t temp_celsius = receivedTHData.temperature;
			int16_t hum_percent = receivedTHData.humidity;

			// Handle negative temperatures.
			if (temp_celsius & 0x8000) {
				temp_celsius = -(temp_celsius & 0x7FFF);
			}

			temp_celsius /= 10;
			hum_percent /= 10;
		}
	}
	if (xQueueReceive(xGasSensorQueue, &receivedGData, portMAX_DELAY) == pdTRUE) {
		// Convert raw values to human-readable format (integer display).
		gas_value = receivedGData;		
	}
	
	if (gas_value > 4095 || temp_celsius > 30 || hum_percent > 50){
		TCC0.CCA = 300;	
	} else {
		TCC0.CCA = 1300;
	}
	}
}

void PostDataTask(void *pvParameters) {
	sensor_data_t receivedTHData;
	uint16_t receivedGData;

	while (2) {
		// Wait indefinitely for data from the queue.
		if (xQueueReceive(xSensorQueue, &receivedTHData, portMAX_DELAY) == pdTRUE) {
			if (receivedTHData.error == 0) {
				// Convert raw values to human-readable format (integer display).
				int16_t temp_celsius = receivedTHData.temperature;
				int16_t hum_percent = receivedTHData.humidity;

				// Handle negative temperatures.
				if (temp_celsius & 0x8000) {
					temp_celsius = -(temp_celsius & 0x7FFF);
				}

				temp_celsius /= 10;
				hum_percent /= 10;

				snprintf(strbuf, sizeof(strbuf), "T:%d\n", temp_celsius);
				gfx_mono_draw_string(strbuf, 0, 0, &sysfont);  // Temperature.
				sendString(strbuf);  // Send the string

				snprintf(strbuf, sizeof(strbuf), "H:%d\n", hum_percent);
				gfx_mono_draw_string(strbuf, 0, 10, &sysfont);  // Humidity.
				sendString(strbuf);  // Send the string
			}
		}
		if (xQueueReceive(xGasSensorQueue, &receivedGData, portMAX_DELAY) == pdTRUE) {
			// Convert raw values to human-readable format (integer display).
			uint16_t gas_value = receivedGData;

			snprintf(strbuf, sizeof(strbuf), "G:%d\n", gas_value);
			gfx_mono_draw_string(strbuf, 0, 20, &sysfont);
			sendString(strbuf);  // Send the string
			
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
	
	PORTC_OUTSET = PIN3_bm; // PC3 as TX
	PORTC_DIRSET = PIN3_bm; //TX pin as output
	
	PORTC_OUTCLR = PIN2_bm; //PC2 as RX
	PORTC_DIRCLR = PIN2_bm; //RX pin as input
	
	setUpSerial();
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	
	ioport_set_pin_dir(J2_PIN0, IOPORT_DIR_OUTPUT);
	
	// Clear the LCD before starting.
	gfx_mono_draw_filled_rect(0, 0, GFX_MONO_LCD_WIDTH, GFX_MONO_LCD_HEIGHT, GFX_PIXEL_CLR);

	// Display static header once.
	//gfx_mono_draw_string("Bryan dan Alvaro", 0, 0, &sysfont);

	// Create a queue capable of holding 10 sensor_data_t structures.
	xSensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
	if (xSensorQueue == NULL) {
		// Handle queue creation failure (e.g., halt the system).
		while (1);
	}
	
	xGasSensorQueue = xQueueCreate(10, sizeof(uint16_t));
	if (xGasSensorQueue == NULL) {
		// Handle queue creation failure (e.g., halt the system).
		while (1);
	}
	
	xTaskCreate(PostDataTask,"Post Task",1000,NULL,tskIDLE_PRIORITY + 3,NULL);                   
	
	xTaskCreate(ServoTask,"Servo Task",1000,NULL,tskIDLE_PRIORITY,NULL);
	
	xTaskCreate(GasSensorTask,"Gas Sensor Task",1000,NULL,tskIDLE_PRIORITY + 1,NULL );
	
	xTaskCreate(SensorTask,"Sensor Task",1000,NULL,tskIDLE_PRIORITY + 2,NULL);
	
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
	// Start the scheduler so the tasks start executing.
	vTaskStartScheduler();

}
