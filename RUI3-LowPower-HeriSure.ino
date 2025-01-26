/**
 * @file RUI3-LowPower-HeriSure.ino
 * @author Raul Ceron (recerpin@posgrado.upv.es)
 * @brief RUI3 based code for low power RAK1901 data transmission
 * @version 0.1
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "app.h"
#include "sensor_rak1901.hpp"

/** Packet is confirmed/unconfirmed (Set with AT commands) */
bool g_confirmed_mode = false;
/** If confirmed packet, number or retries (Set with AT commands) */
uint8_t g_confirmed_retry = 1;
/** Data rate  (Set with AT commands) */
uint8_t g_data_rate = 3;

/** Flag if transmit is active, used by some sensors */
volatile bool tx_active = false;

/** fPort to send packages */
uint8_t fPort = 66;

/** Payload buffer */
uint8_t uplink_payload[64];

uint16_t my_fcount = 1;

int16_t temperature_encoded = 0;
int16_t humidity_encoded = 0;

/**
 * @brief Callback after join request cycle
 *
 * @param status Join result
 */
void joinCallback(int32_t status)
{
	if (status != 0)
	{
		MYLOG("JOIN-CB", "LoRaWan OTAA - join fail! \r\n");
		// To be checked if this makes sense
		// api.lorawan.join();
	}
	else
	{
		MYLOG("JOIN-CB", "LoRaWan OTAA - joined! \r\n");
		digitalWrite(LED_BLUE, LOW);
	}
}

/**
 * @brief LoRaWAN callback after packet was received
 *
 * @param data pointer to structure with the received data
 */
void receiveCallback(SERVICE_LORA_RECEIVE_T *data)
{
	MYLOG("RX-CB", "RX, port %d, DR %d, RSSI %d, SNR %d", data->Port, data->RxDatarate, data->Rssi, data->Snr);
	for (int i = 0; i < data->BufferSize; i++)
	{
		Serial.printf("%02X", data->Buffer[i]);
	}
	Serial.print("\r\n");
	tx_active = false;
}

/**
 * @brief LoRaWAN callback after TX is finished
 *
 * @param status TX status
 */
void sendCallback(int32_t status)
{
	MYLOG("TX-CB", "TX status %d", status);
	digitalWrite(LED_BLUE, LOW);
	tx_active = false;
}

/**
 * @brief Arduino setup, called once after reboot/power-up
 *
 */
void setup()
{
	/* Make sure module is in LoRaWAN mode */
	if (api.lorawan.nwm.get() != 1) /* check LoRaWAN mode */
	{
		if (api.lorawan.nwm.set()) /* set LoRaWAN mode */
		{
			api.system.reboot();
		}
	}

	g_confirmed_mode = api.lorawan.cfm.get();
	g_confirmed_retry = api.lorawan.rety.get();
	g_data_rate = api.lorawan.dr.get();

	// Setup the callbacks for joined and send finished
	api.lorawan.registerRecvCallback(receiveCallback);
	api.lorawan.registerSendCallback(sendCallback);
	api.lorawan.registerJoinCallback(joinCallback);

	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, HIGH);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, HIGH);

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, LOW);

	// Start Serial
	Serial.begin(115200);

	// Delay for 5 seconds to give the chance for AT+BOOT
	delay(5000);

	api.system.firmwareVersion.set("RUI3-LowPower-HeriSure-V1.0.0");

	Serial.println("RAKwireless RUI3 Node");
	Serial.println("------------------------------------------------------");
	Serial.println("Setup the device with WisToolBox or AT commands before using it");
	Serial.printf("Version %s\n", api.system.firmwareVersion.get().c_str());
	Serial.println("------------------------------------------------------");

	// Initialize module
	Wire.begin();

	// Register the custom AT command to get device status
	if (!init_status_at())
	{
		MYLOG("SETUP", "Add custom AT command STATUS fail");
	}

	// Register the custom AT command to set the send interval
	if (!init_interval_at())
	{
		MYLOG("SETUP", "Add custom AT command Send Interval fail");
	}

	// Get saved sending interval from flash
	get_at_setting();

	digitalWrite(LED_GREEN, LOW);

	// Create a timer.
	api.system.timer.create(RAK_TIMER_0, sensor_handler, RAK_TIMER_PERIODIC);
	if (custom_parameters.send_interval != 0)
	{
		// Start a timer.
		api.system.timer.start(RAK_TIMER_0, custom_parameters.send_interval, NULL);
	}

	if (api.lorawan.nwm.get() == 1)
	{
		if (g_confirmed_mode)
		{
			MYLOG("SETUP", "Confirmed enabled");
		}
		else
		{
			MYLOG("SETUP", "Confirmed disabled");
		}

		MYLOG("SETUP", "Retry = %d", g_confirmed_retry);

		MYLOG("SETUP", "DR = %d", g_data_rate);
	}

	/** Wait for Join success */
	while (api.lorawan.njs.get() == 0)
	{
		Serial.println("Wait for LoRaWAN join...");
		api.lorawan.join();
		delay(10000);
	}

	// Enable low power mode
	api.system.lpm.set(1);
}

/**
 * @brief sensor_handler is a timer function called every
 * custom_parameters.send_interval milliseconds. Default is 120000. Can be
 * changed with ATC+SENDINT command
 *
 */
void sensor_handler(void *)
{
	MYLOG("UPLINK", "Start");
	digitalWrite(LED_BLUE, HIGH);

	if (api.lorawan.nwm.get() == 1)
	{
		// Check if the node has joined the network
		if (!api.lorawan.njs.get())
		{
			MYLOG("UPLINK", "Not joined, skip sending");
			return;
		}
	}

	temperature_encoded = temperature_Read() * 10.0f;
	humidity_encoded = humidity_Read() * 2.0f;

	// Create payload
	// Cayenne LPP temperature
	uplink_payload[0] = 0x01;
	uplink_payload[1] = 0x67; // Cayenne LPP temperature
	uplink_payload[2] = (uint8_t)(temperature_encoded >> 8);
	uplink_payload[3] = (uint8_t)(temperature_encoded & 0xFF);

	// Cayenne LPP humidity
	uplink_payload[4] = 0x01;
	uplink_payload[5] = 0x68; // Cayenne LPP humidity
	uplink_payload[6] = (uint8_t)(humidity_encoded);

	// Send the packet
	send_packet();
}

/**
 * @brief Send the data packet that was prepared in
 * Cayenne LPP format by the different sensor and location
 * aqcuision functions
 *
 */
void send_packet(void)
{
	MYLOG("UPLINK", "Sending packet # %d", my_fcount);
	my_fcount++;
	// Send the packet
	if (api.lorawan.send(7, uplink_payload, fPort, g_confirmed_mode, g_confirmed_retry))
	{
		MYLOG("UPLINK", "Packet enqueued, size 4");
		tx_active = true;
	}
	else
	{
		MYLOG("UPLINK", "Send failed");
		tx_active = false;
	}
}

/**
 * @brief This example is complete timer driven.
 * The loop() does nothing than sleep.
 *
 */
void loop()
{
	api.system.sleep.all();
}
