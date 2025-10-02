// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Matter Manager
#include <Matter.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <inttypes.h>
#include <lib/core/CHIPError.h>
#include <lib/support/TimeUtils.h>
#include <type_traits>
#include <utility>
#if !CONFIG_ENABLE_CHIPOBLE
// if the device can be commissioned using BLE, WiFi is not used - save flash space
#include <WiFi.h>
#endif
#include <Preferences.h>

namespace {
template <typename MatterT, typename = void>
struct HasLoopMethod : std::false_type {};

template <typename MatterT>
struct HasLoopMethod<MatterT, std::void_t<decltype(std::declval<MatterT &>().loop())>> : std::true_type {};

template <typename MatterT>
void PumpMatterStack(MatterT &matter) {
  if constexpr (HasLoopMethod<MatterT>::value) {
    // Recent Arduino Matter releases provide a loop() helper that pumps the
    // stack to keep CASE sessions progressing and free PacketBuffers.
    matter.loop();
  } else {
    // Older releases run the Matter event loop on their own tasks. Yielding to
    // the scheduler ensures those tasks get CPU time so buffers can be
    // reclaimed between iterations.
    delay(1);
  }
}

void EnsureCommissioningWindowOpen() {
  chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
  if (commissionMgr.IsCommissioningWindowOpen()) {
    return;
  }

  constexpr auto kTimeout = chip::System::Clock::Seconds16(300);
#if CONFIG_ENABLE_CHIPOBLE
  constexpr auto kAdvertisementMode = chip::CommissioningWindowAdvertisement::kAllSupported;
#else
  constexpr auto kAdvertisementMode = chip::CommissioningWindowAdvertisement::kDnssdOnly;
#endif

  const CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeout, kAdvertisementMode);
  if (err != CHIP_NO_ERROR) {
    Serial.printf(
      "Failed to open commissioning window (err 0x%08" PRIX32 ")\r\n",
      static_cast<uint32_t>(err.AsInteger())
    );
  } else {
#if CONFIG_ENABLE_CHIPOBLE
    Serial.println("Commissioning window opened over BLE (and IP).");
#else
    Serial.println("Commissioning window opened over IP.");
#endif
  }
}
}  // namespace

// List of Matter Endpoints for this Node
// On/Off Plugin Endpoint
MatterOnOffPlugin OnOffPlugin;

// CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
#if !CONFIG_ENABLE_CHIPOBLE
// WiFi is manually set and started
const char *ssid = "your-ssid";          // Change this to your WiFi SSID
const char *password = "your-password";  // Change this to your WiFi password
#endif

// it will keep last OnOff state stored, using Preferences
Preferences matterPref;
const char *onOffPrefKey = "OnOff";

// set your board Power Relay pin here - this example uses the built-in LED for easy visualization
#ifdef LED_BUILTIN
const uint8_t onoffPin = LED_BUILTIN;
#else
const uint8_t onoffPin = 2;  // Set your pin here - usually a power relay
#warning "Do not forget to set the Power Relay pin"
#endif

// board USER BUTTON pin necessary for Decommissioning
const uint8_t buttonPin = BOOT_PIN;  // Set your pin here. Using BOOT Button.

// Button control
uint32_t button_time_stamp = 0;                // debouncing control
bool button_state = false;                     // false = released | true = pressed
bool button_long_press_handled = false;        // tracks whether the long-press action has already run
const uint32_t decommissioningTimeout = 5000;  // keep the button pressed for 5s, or longer, to decommission
const uint32_t reopenCommissioningTimeout = 1000;  // 1s press re-opens the commissioning window for multi-admin

namespace {
constexpr uint32_t kRelayPulseDurationMs = 1000;
bool relayPulseActive = false;
uint32_t relayPulseStartMs = 0;

void StartRelayPulse() {
  relayPulseStartMs = millis();
  relayPulseActive = true;
  digitalWrite(onoffPin, LOW);
}

void ServiceRelayPulse() {
  if (relayPulseActive && (millis() - relayPulseStartMs >= kRelayPulseDurationMs)) {
    relayPulseActive = false;
    digitalWrite(onoffPin, HIGH);
  }
}
}  // namespace

// Matter Protocol Endpoint Callback
bool setPluginOnOff(bool state) {
  Serial.printf("User Callback :: New Plugin State = %s\r\n", state ? "ON" : "OFF");
  StartRelayPulse();
  // store last OnOff state for when the Plugin is restarted / power goes off
  matterPref.putBool(onOffPrefKey, state);
  // This callback must return the success state to Matter core
  return true;
}

void setup() {
  // Initialize the USER BUTTON
  pinMode(buttonPin, INPUT_PULLUP);
  // Initialize the Power Relay (plugin) GPIO
  pinMode(onoffPin, OUTPUT);
  digitalWrite(onoffPin, HIGH);

  Serial.begin(115200);

// CONFIG_ENABLE_CHIPOBLE is enabled when BLE is used to commission the Matter Network
#if !CONFIG_ENABLE_CHIPOBLE
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\r\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
#endif

  // Initialize Matter EndPoint
  matterPref.begin("MatterPrefs", false);
  bool lastOnOffState = matterPref.getBool(onOffPrefKey, false);
  OnOffPlugin.begin(lastOnOffState);
  OnOffPlugin.onChange(setPluginOnOff);

  // Matter beginning - Last step, after all EndPoints are initialized
  Matter.begin();
  // This may be a restart of a already commissioned Matter accessory
  if (Matter.isDeviceCommissioned()) {
    Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
    Serial.printf("Initial state: %s\r\n", OnOffPlugin.getOnOff() ? "ON" : "OFF");
    OnOffPlugin.updateAccessory();  // configure the Plugin based on initial state
  } else {
#if CONFIG_ENABLE_CHIPOBLE
    Serial.println("Opening commissioning window over BLE...");
#else
    Serial.println(
      "BLE commissioning is disabled in this build; falling back to IP-only discovery."
    );
#endif
    EnsureCommissioningWindowOpen();
  }

  Serial.println(
    "Hold the user button for ~1s to reopen the commissioning window, or for ~5s to decommission."
  );
}

void loop() {
  // Allow the Matter stack to process internal work such as CASE sessions and
  // memory housekeeping. Without this call the stack can run out of buffers
  // which prevents the node from completing the commissioning handshake.
  PumpMatterStack(Matter);
  ServiceRelayPulse();

  // Check Matter Plugin Commissioning state, which may change during execution of loop()
  if (!Matter.isDeviceCommissioned()) {
    EnsureCommissioningWindowOpen();
    Serial.println("");
    Serial.println("Matter Node is not commissioned yet.");
    Serial.println("Initiate the device discovery in your Matter environment.");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
    // waits for Matter Plugin Commissioning.
    uint32_t timeCount = 0;
    while (!Matter.isDeviceCommissioned()) {
      PumpMatterStack(Matter);
      EnsureCommissioningWindowOpen();
      ServiceRelayPulse();
      delay(100);
      if ((timeCount++ % 50) == 0) {  // 50*100ms = 5 sec
        Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
      }
    }
    Serial.printf("Initial state: %s\r\n", OnOffPlugin.getOnOff() ? "ON" : "OFF");
    OnOffPlugin.updateAccessory();  // configure the Plugin based on initial state
    Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
  }

  // Check if the button has been pressed
  const bool buttonPressed = digitalRead(buttonPin) == LOW;
  const uint32_t now = millis();

  if (buttonPressed && !button_state) {
    // deals with button debouncing
    button_time_stamp = now;  // record the time while the button is pressed.
    button_state = true;      // pressed.
    button_long_press_handled = false;
  }

  if (button_state && buttonPressed && !button_long_press_handled) {
    const uint32_t pressDuration = now - button_time_stamp;
    if (pressDuration >= decommissioningTimeout) {
      Serial.println(
        "Decommissioning the Plugin Matter Accessory. It shall be commissioned again."
      );
      OnOffPlugin.setOnOff(false);  // turn the plugin off
      Matter.decommission();
      button_long_press_handled = true;
      button_time_stamp = now;  // avoid running decommissioning again while still pressed
    }
  }

  if (!buttonPressed && button_state) {
    button_state = false;  // released
    const uint32_t pressDuration = now - button_time_stamp;
    if (!button_long_press_handled && pressDuration >= reopenCommissioningTimeout) {
      if (Matter.isDeviceCommissioned()) {
        Serial.println(
          "Re-opening the commissioning window to add another Matter administrator."
        );
      }
      EnsureCommissioningWindowOpen();
    }
  }
}
