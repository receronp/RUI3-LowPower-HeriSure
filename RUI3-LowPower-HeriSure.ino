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

enum class Command : uint8_t
{
  SetAlarm = 0,
  SetInterval = 1
};

// Structure to hold LoRaWAN settings
struct LoraWanConfig
{
  bool confirmedMode = false;
  uint8_t confirmedRetry = 1;
  uint8_t dataRate = 3;
};

// Structure to hold application state
struct AppState
{
  volatile bool txActive = false;
  uint8_t fPort = 66;
  uint8_t uplinkPayload[64];
  uint16_t frameCounter = 1;
  int16_t temperature = 0;
  int16_t humidity = 0;
};

// Global instances of the structs
LoraWanConfig loraConfig;
AppState appState;

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

  switch (static_cast<Command>(data->Buffer[0]))
  {
  case Command::SetAlarm:
    digitalWrite(LED_GREEN, data->Buffer[1] != 0);
    MYLOG("RX-CB", "Alert turned %s", data->Buffer[1] ? "on" : "off");
    break;
  case Command::SetInterval:
    if (data->BufferSize > 1)
    {
      uint32_t newInterval = 0;
      for (size_t i = 1; i < data->BufferSize; ++i)
      {
        // Big-Endian encoding from bytes received
        newInterval = (newInterval << 8) | (uint32_t)data->Buffer[i];
      }
      MYLOG("RX-CB", "Requested interval %lu s", newInterval);
      char intervalBuffer[20] = {'\n'};
      sprintf(intervalBuffer, "%lu", newInterval);
      MYLOG("RX-CB", "String representation: %s\n", intervalBuffer);

      update_send_interval(intervalBuffer);
    }
    break;

  default:
    MYLOG("RX-CB", "Unknown Command received: %d", data->Buffer[0]);
    break;
  }
  appState.txActive = false;
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
  appState.txActive = false;
}

/**
 * @brief Attempts to join the LoRaWAN network.  Loops until successful.
 *
 */
void lorawanJoin(int maxRetries = 5)
{
  int retryCount = 0;
  while (!api.lorawan.njs.get() && retryCount < maxRetries)
  {
    Serial.print("Waiting for LoRaWAN join... Attempt ");
    Serial.println(retryCount + 1);
    api.lorawan.join();
    delay(10000 * ++retryCount); // Exponential backoff
  }

  if (!api.lorawan.njs.get()) // Check if join failed after max retries
  {
    MYLOG("LORAWAN-JOIN", "LoRaWAN join failed after multiple attempts.");
    api.system.sleep.all(600000); // Deep sleep for 10 minutes (in milliseconds)
  }
  else
  {
    MYLOG("LORAWAN-JOIN", "LoRaWAN joined successfully!");
  }
}

/**
 * @brief Arduino setup function. Configures the device and initializes peripherals.
 * Called once after reboot or power-up.
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

  // Initialize LoRaWAN config
  loraConfig.confirmedMode = api.lorawan.cfm.get();
  loraConfig.confirmedRetry = api.lorawan.rety.get();
  loraConfig.dataRate = api.lorawan.dr.get();

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
    if (loraConfig.confirmedMode)
    {
      MYLOG("SETUP", "Confirmed enabled");
    }
    else
    {
      MYLOG("SETUP", "Confirmed disabled");
    }

    MYLOG("SETUP", "Retry = %d", loraConfig.confirmedRetry);

    MYLOG("SETUP", "DR = %d", loraConfig.dataRate);
  }

  lorawanJoin();

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
      lorawanJoin();
      return;
    }
  }

  appState.temperature = temperature_Read() * 10.0f;
  appState.humidity = humidity_Read() * 2.0f;

  // Cayenne LPP payload creation
  appState.uplinkPayload[0] = 0x01;
  appState.uplinkPayload[1] = 0x67; // Cayenne LPP temperature
  appState.uplinkPayload[2] = static_cast<uint8_t>(appState.temperature >> 8);
  appState.uplinkPayload[3] = static_cast<uint8_t>(appState.temperature & 0xFF);
  appState.uplinkPayload[4] = 0x01;
  appState.uplinkPayload[5] = 0x68; // Cayenne LPP humidity
  appState.uplinkPayload[6] = static_cast<uint8_t>(appState.humidity);

  send_packet();
}

/**
 * @brief Send the data packet that was prepared in
 * Cayenne LPP format by the different sensor and location
 * acquisition functions
 *
 */
void send_packet(void)
{
  MYLOG("UPLINK", "Sending packet # %u", appState.frameCounter);
  appState.frameCounter++;

  if (api.lorawan.send(7, appState.uplinkPayload, appState.fPort, loraConfig.confirmedMode, loraConfig.confirmedRetry))
  {
    MYLOG("UPLINK", "Packet enqueued, size 4");
    appState.txActive = true;
  }
  else
  {
    MYLOG("UPLINK", "Send failed");
    appState.txActive = false;
  }
}

/**
 * @brief This program is completely timer driven.
 * The loop() does nothing other than sleep.
 *
 */
void loop()
{
  api.system.sleep.all();
}
