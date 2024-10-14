#include <asf.h>
#include <util/delay.h>
#include <stdio.h>

static char strbuf[128];  // Buffer for LCD output

#define DHT22_PIN PIN0_bm  // Using PORTC PIN0 for data

// Initialize the DHT22 data pin
void dht22_init(void) {
	PORTC.DIRSET = DHT22_PIN;  // Set PIN0 as output
}

// Send start signal to DHT22
void dht22_request(void) {
	PORTC.OUTCLR = DHT22_PIN;  // Pull pin low
	_delay_ms(20);             // Wait for at least 18ms
	PORTC.OUTSET = DHT22_PIN;  // Pull pin high
	_delay_us(40);             // Wait 20-40us
	PORTC.DIRCLR = DHT22_PIN;  // Set pin as input to read data
}

// Read 8 bits from DHT22
uint8_t dht22_read(void) {
	uint8_t result = 0;
	for (int i = 0; i < 8; i++) {
		while (!(PORTC.IN & DHT22_PIN));  // Wait for pin to go high
		_delay_us(30);                    // Wait for 30us
		if (PORTC.IN & DHT22_PIN) {
			result |= (1 << (7 - i));  // If high after 30us, it's a 1
		}
		while (PORTC.IN & DHT22_PIN);  // Wait for pin to go low
	}
	return result;
}

// Read temperature and humidity from DHT22
void dht22_read_data(uint16_t *temperature, uint16_t *humidity) {
	uint8_t hum_high, hum_low, temp_high, temp_low, checksum;

	dht22_request();  // Send request to DHT22

	// Wait for sensor's response
	while (PORTC.IN & DHT22_PIN);      // Wait for low signal
	while (!(PORTC.IN & DHT22_PIN));   // Wait for high signal
	while (PORTC.IN & DHT22_PIN);      // Wait for low signal

	// Read all 5 bytes (humidity high, humidity low, temp high, temp low, checksum)
	hum_high = dht22_read();
	hum_low = dht22_read();
	temp_high = dht22_read();
	temp_low = dht22_read();
	checksum = dht22_read();

	// Combine high and low bytes
	*humidity = (hum_high << 8) | hum_low;
	*temperature = (temp_high << 8) | temp_low;

	// Calculate checksum correctly (only lowest 8 bits matter)
	uint8_t calc_checksum = (hum_high + hum_low + temp_high + temp_low) & 0xFF;

	// Display checksum error if validation fails
	if (calc_checksum != checksum) {
		gfx_mono_draw_string("Checksum Error", 0, 20, &sysfont);
		return;
	}

	// Convert raw values to human-readable format (integer display)
	uint16_t temp_celsius = *temperature / 10;  // Avoid floating point
	uint16_t hum_percent = *humidity / 10;      // Avoid floating point

	// Display formatted temperature and humidity as integers on the LCD
	snprintf(strbuf, sizeof(strbuf), "Temp: %d C", temp_celsius);  // Use integer display
	gfx_mono_draw_string("Bryan dan Alvaro", 0, 0, &sysfont);      // Heading
	gfx_mono_draw_string(strbuf, 0, 10, &sysfont);                 // Temperature

	snprintf(strbuf, sizeof(strbuf), "Humid: %d %%", hum_percent); // Use integer display
	gfx_mono_draw_string(strbuf, 0, 20, &sysfont);                 // Humidity

	_delay_ms(1000);  // Refresh every second
}

int main(void) {
	uint16_t temperature = 0, humidity = 0;

	// Initialize system, board, and LCD
	sysclk_init();
	board_init();
	gfx_mono_init();
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	dht22_init();  // Initialize DHT22 sensor
	while (1) {
		// Read and display temperature and humidity every second
		dht22_read_data(&temperature, &humidity);
	}
}
