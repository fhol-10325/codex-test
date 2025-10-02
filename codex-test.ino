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
#include <platform/PlatformManager.h>
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

void OpenCommissioningWindowWork(intptr_t) {
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

void EnsureCommissioningWindowOpen() {
  const CHIP_ERROR scheduleErr = chip::DeviceLayer::PlatformMgr().ScheduleWork(OpenCommissioningWindowWork, 0);
  if (scheduleErr != CHIP_NO_ERROR) {
    Serial.printf(
      "Failed to schedule commissioning window open (err 0x%08" PRIX32 ")\r\n",
      static_cast<uint32_t>(scheduleErr.AsInteger())
    );
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

// Set your board Power Relay pin here. Pin 2 drives the external relay.
const uint8_t onoffPin = 2;

// Status indicator pin to drive an LED once the node is fully ready.
const uint8_t statusPin = 4;

// board USER BUTTON pin necessary for Decommissioning
const uint8_t buttonPin = BOOT_PIN;  // Set your pin here. Using BOOT Button.

// Button control
uint32_t button_time_stamp = 0;                // debouncing control
bool button_state = false;                     // false = released | true = pressed
bool button_long_press_handled = false;        // tracks whether the long-press action has already run
const uint32_t decommissioningTimeout = 5000;  // keep the button pressed for 5s, or longer, to decommission
const uint32_t reopenCommissioningTimeout = 1000;  // 1s press re-opens the commissioning window for multi-admin

bool relayPulsingEnabled = false;

namespace {
constexpr uint32_t kRelayPulseDurationMs = 1000;
bool relayPulseActive = false;
uint32_t relayPulseStartMs = 0;

enum class StatusLedMode : uint8_t {
  kOff,
  kSolid,
  kFade,
};

constexpr uint8_t kStatusLedFadeMin = 0;
constexpr uint8_t kStatusLedFadeMax = 255;
constexpr uint8_t kStatusLedFadeStep = 5;
constexpr uint32_t kStatusLedFadeIntervalMs = 15;

StatusLedMode statusLedMode = StatusLedMode::kOff;
int statusLedCurrentValue = -1;
uint8_t statusLedSolidValue = 0;
uint8_t statusLedFadeValue = kStatusLedFadeMin;
int statusLedFadeDirection = 1;
uint32_t statusLedLastUpdateMs = 0;

bool lastCommissionedState = false;
bool commissioningInfoLogged = false;
uint32_t lastCommissioningLogMs = 0;
uint32_t lastCommissioningWindowMs = 0;

void WriteStatusLed(uint8_t value) {
  if (statusLedCurrentValue != value) {
    analogWrite(statusPin, value);
    statusLedCurrentValue = value;
  }
}

void SetStatusLedOff() {
  statusLedMode = StatusLedMode::kOff;
  WriteStatusLed(kStatusLedFadeMin);
}

void SetStatusLedSolid(bool on) {
  statusLedMode = StatusLedMode::kSolid;
  statusLedSolidValue = on ? kStatusLedFadeMax : kStatusLedFadeMin;
  WriteStatusLed(statusLedSolidValue);
}

void StartStatusLedFade() {
  statusLedMode = StatusLedMode::kFade;
  statusLedFadeValue = kStatusLedFadeMin;
  statusLedFadeDirection = 1;
  statusLedLastUpdateMs = millis();
  WriteStatusLed(statusLedFadeValue);
}

void ServiceStatusLed() {
  if (statusLedMode != StatusLedMode::kFade) {
    if (statusLedMode == StatusLedMode::kSolid) {
      WriteStatusLed(statusLedSolidValue);
    }
    return;
  }

  const uint32_t now = millis();
  if (now - statusLedLastUpdateMs < kStatusLedFadeIntervalMs) {
    return;
  }

  statusLedLastUpdateMs = now;
  int nextValue = static_cast<int>(statusLedFadeValue) + statusLedFadeDirection * kStatusLedFadeStep;

  if (nextValue >= kStatusLedFadeMax) {
    nextValue = kStatusLedFadeMax;
    statusLedFadeDirection = -1;
  } else if (nextValue <= kStatusLedFadeMin) {
    nextValue = kStatusLedFadeMin;
    statusLedFadeDirection = 1;
  }

  statusLedFadeValue = static_cast<uint8_t>(nextValue);
  WriteStatusLed(statusLedFadeValue);
}

void StartRelayPulse() {
  relayPulseStartMs = millis();
  relayPulseActive = true;
  digitalWrite(onoffPin, HIGH);
}

void ServiceRelayPulse() {
  if (relayPulseActive && (millis() - relayPulseStartMs >= kRelayPulseDurationMs)) {
    relayPulseActive = false;
    digitalWrite(onoffPin, LOW);
  }
}

void DecommissionWork(intptr_t) {
  Matter.decommission();
}

void RequestDecommission() {
  const CHIP_ERROR scheduleErr = chip::DeviceLayer::PlatformMgr().ScheduleWork(DecommissionWork, 0);
  if (scheduleErr != CHIP_NO_ERROR) {
    Serial.printf(
      "Failed to schedule decommission (err 0x%08" PRIX32 ")\r\n",
      static_cast<uint32_t>(scheduleErr.AsInteger())
    );
  }
}
}  // namespace

// Matter Protocol Endpoint Callback
bool setPluginOnOff(bool state) {
  Serial.printf("User Callback :: New Plugin State = %s\r\n", state ? "ON" : "OFF");
  if (relayPulsingEnabled) {
    StartRelayPulse();
  } else {
    // During startup we restore the steady-state without pulsing the relay output.
    digitalWrite(onoffPin, LOW);
  }
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
  digitalWrite(onoffPin, LOW);
  pinMode(statusPin, OUTPUT);
  SetStatusLedOff();

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
  lastCommissionedState = Matter.isDeviceCommissioned();
  if (lastCommissionedState) {
    Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
    Serial.printf("Initial state: %s\r\n", OnOffPlugin.getOnOff() ? "ON" : "OFF");
    OnOffPlugin.updateAccessory();  // configure the Plugin based on initial state
    relayPulsingEnabled = true;
    SetStatusLedSolid(true);
  } else {
#if CONFIG_ENABLE_CHIPOBLE
    Serial.println("Opening commissioning window over BLE...");
#else
    Serial.println(
      "BLE commissioning is disabled in this build; falling back to IP-only discovery."
    );
#endif
    EnsureCommissioningWindowOpen();
    StartStatusLedFade();
    commissioningInfoLogged = false;
    lastCommissioningLogMs = millis();
    lastCommissioningWindowMs = millis();
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
  ServiceStatusLed();

  // Check Matter Plugin Commissioning state, which may change during execution of loop()
  const bool commissioned = Matter.isDeviceCommissioned();
  if (commissioned != lastCommissionedState) {
    lastCommissionedState = commissioned;
    if (commissioned) {
      Serial.printf("Initial state: %s\r\n", OnOffPlugin.getOnOff() ? "ON" : "OFF");
      OnOffPlugin.updateAccessory();  // configure the Plugin based on initial state
      Serial.println("Matter Node is commissioned and connected to the network. Ready for use.");
      relayPulsingEnabled = true;
      SetStatusLedSolid(true);
      commissioningInfoLogged = false;
    } else {
      relayPulsingEnabled = false;
      StartStatusLedFade();
      commissioningInfoLogged = false;
      lastCommissioningLogMs = 0;
      lastCommissioningWindowMs = millis();
      EnsureCommissioningWindowOpen();
    }
  }

  if (!commissioned) {
    const uint32_t commissioningNow = millis();
    if (!commissioningInfoLogged) {
      Serial.println("");
      Serial.println("Matter Node is not commissioned yet.");
      Serial.println("Initiate the device discovery in your Matter environment.");
      Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
      Serial.printf("Manual pairing code: %s\r\n", Matter.getManualPairingCode().c_str());
      Serial.printf("QR code URL: %s\r\n", Matter.getOnboardingQRCodeUrl().c_str());
      commissioningInfoLogged = true;
      lastCommissioningLogMs = commissioningNow;
    }

    if ((commissioningNow - lastCommissioningWindowMs) >= 1000) {
      EnsureCommissioningWindowOpen();
      lastCommissioningWindowMs = commissioningNow;
    }

    if ((commissioningNow - lastCommissioningLogMs) >= 5000) {
      Serial.println("Matter Node not commissioned yet. Waiting for commissioning.");
      lastCommissioningLogMs = commissioningNow;
    }
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
      RequestDecommission();
      button_long_press_handled = true;
      button_time_stamp = now;  // avoid running decommissioning again while still pressed
      SetStatusLedOff();
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
