#include <Arduino.h>
#include <Bluepad32.h>
#include "pinout.h"
#include "state.h"
#include "utils.h"
#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Interval:
#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 100

unsigned long previousMillis = 0;  // will store last time a message was send

int MY_ESP32_CAN_ID = 2;

void setupCAN() {

	// Initialize configuration structures using macro initializers
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	//	twai_filter_config_t f_config = {
//			.acceptance_code = 0,  // Accept everything initially
//			.acceptance_mask = static_cast<uint32_t>((MY_ESP32_CAN_ID << 21)),  // Ignore only this ID
//			.single_filter = true  // Use a single filter for both standard and extended IDs
//	};

	// Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		Serial.println("Driver installed");
	} else {
		Serial.println("Failed to install driver");
		return;
	}

	// Start TWAI driver
	if (twai_start() == ESP_OK) {
		Serial.println("Driver started");
	} else {
		Serial.println("Failed to start driver");
		return;
	}

	// Reconfigure alerts to detect TX alerts and Bus-Off errors
	uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
	if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
		Serial.println("CAN Alerts reconfigured");
	} else {
		Serial.println("Failed to reconfigure alerts");
		return;
	}

	features.CAN = true;
}


static void print_message(twai_message_t &message, bool received) {
	if(received) Serial.print("<< RECEIVED");
	else Serial.print(">> SENT");
	// Process received message
	if (message.extd) {
		Serial.print(" message in Extended Format: ");
	} else {
		Serial.print(" message in Standard Format: ");
	}
	Serial.printf("ID: %lx Byte:", message.identifier);
	if (!(message.rtr)) {
		for (int i = 0; i < message.data_length_code; i++) {
			Serial.printf(" %d = %02x,", i, message.data[i]);
		}
	}
	Serial.println("");
}

static void send_message() {
	// Send message
	static u8_t c = 0;
	c++;
	// Configure message to transmit
	twai_message_t message =  {0};
	message.identifier = MY_ESP32_CAN_ID;
	message.rtr = 0;
	message.data_length_code = 4;
	for (int i = 0; i < 8; i++) {
		message.data[i] = 0;
	}
	message.data[3] = c;
	// Queue message for transmission
	if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
		//printf("Message queued for transmission 0x%x 0x%x 0x%x 0x%x\n", message.data[0], message.data[1], message.data[2], message.data[3]);
		print_message(message, false);
	} else {
		printf("Failed to queue message for transmission\n");
	}
}

void loopCAN() {
	if(!features.CAN) return;
	// Check if alert happened
	uint32_t alerts_triggered;
	twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
//	twai_status_info_t twaistatus;
//	twai_get_status_info(&twaistatus);

//	// Handle alerts
//	if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
//		Serial.println("Alert: TWAI controller has become error passive.");
//	}
//	if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
//		Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
//		Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
//	}
//	if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
//		Serial.println("Alert: The Transmission failed.");
//		Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
//		Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
//		Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
//	}
//	if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
//		Serial.println("Alert: The Transmission was successful.");
//		Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
//	}
//	if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
//		Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
//		Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
//		Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
//		Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
//	}

	// Check if message is received
	if (alerts_triggered & TWAI_ALERT_RX_DATA) {
		// One or more messages received. Handle all.
		twai_message_t message;
		while (twai_receive(&message, 0) == ESP_OK) {
			print_message(message, true);
		}
	}
	static u64_t lastUpdate = 0;
	if(timestamp_ms - lastUpdate > 500) {
		lastUpdate = timestamp_ms;
		//Console.printf("Sending CAN packet\n");
		send_message();
	}
}
