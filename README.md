#include <Arduino.h>

// ------------------------------------------------------------
// Traffic light controller for a 4-way intersection
// North-South (NS) and East-West (EW) traffic streams
// With pedestrian request handling and safe WALK/DON'T WALK signals
// ------------------------------------------------------------

// ---------------------
// Timing configuration
// ---------------------
const unsigned long GREEN_DURATION_MS = 6000UL;
const unsigned long YELLOW_DURATION_MS = 2000UL;
const unsigned long ALL_RED_DURATION_MS = 1500UL;
const unsigned long WALK_DURATION_MS = 5000UL;
const unsigned long BUTTON_DEBOUNCE_MS = 50UL;

// ---------------------
// Traffic LED pins
// ---------------------
const uint8_t NS_RED_PIN = 4;
const uint8_t NS_YELLOW_PIN = 3;
const uint8_t NS_GREEN_PIN = 2;

const uint8_t SS_RED_PIN = 7;
const uint8_t SS_YELLOW_PIN = 6;
const uint8_t SS_GREEN_PIN = 5;

const uint8_t EW_RED_PIN = 10;
const uint8_t EW_YELLOW_PIN = 9;
const uint8_t EW_GREEN_PIN = 8;

const uint8_t WE_RED_PIN = 13;
const uint8_t WE_YELLOW_PIN = 12;
const uint8_t WE_GREEN_PIN = 11;

// ---------------------
// Pedestrian signals
// Dedicated WALK/DON'T WALK LEDs for each crossing stream
// ---------------------
const uint8_t PED_NS_WALK_PIN = 14;   // A0
const uint8_t PED_NS_DONTWALK_PIN = 15; // A1
const uint8_t PED_EW_WALK_PIN = 16;   // A2
const uint8_t PED_EW_DONTWALK_PIN = 17; // A3

// ---------------------
// Push buttons for pedestrian requests
// INPUT_PULLUP means button pressed = LOW
// ---------------------
const uint8_t PED_NS_BUTTON_PIN = 18; // A4
const uint8_t PED_EW_BUTTON_PIN = 19; // A5

// ---------------------
// State machine
// ---------------------
enum TrafficState {
  NS_GREEN,
  NS_YELLOW,
  ALL_RED,
  EW_GREEN,
  EW_YELLOW,
  NS_WALK,
  EW_WALK
};

TrafficState currentState = NS_GREEN;
unsigned long stateStartMillis = 0;
bool nsPedRequest = false;
bool ewPedRequest = false;

// Track the most recently served green stream so the next all-red can decide
// whether to insert a walk for the just-ended stream or the next stream.
TrafficState lastGreenStream = NS_GREEN;

// Debounce tracking (for button edges)
bool nsButtonLastRaw = HIGH;
bool ewButtonLastRaw = HIGH;
bool nsButtonPressedFlag = false;
bool ewButtonPressedFlag = false;
unsigned long nsButtonLastChange = 0;
unsigned long ewButtonLastChange = 0;

// ------------------------------------------------------------
// Helper functions
// ------------------------------------------------------------

void setAllTrafficRed() {
  digitalWrite(NS_RED_PIN, HIGH);
  digitalWrite(NS_YELLOW_PIN, LOW);
  digitalWrite(NS_GREEN_PIN, LOW);

  digitalWrite(SS_RED_PIN, HIGH);
  digitalWrite(SS_YELLOW_PIN, LOW);
  digitalWrite(SS_GREEN_PIN, LOW);

  digitalWrite(EW_RED_PIN, HIGH);
  digitalWrite(EW_YELLOW_PIN, LOW);
  digitalWrite(EW_GREEN_PIN, LOW);

  digitalWrite(WE_RED_PIN, HIGH);
  digitalWrite(WE_YELLOW_PIN, LOW);
  digitalWrite(WE_GREEN_PIN, LOW);
}

void setAllPedDontWalk() {
  digitalWrite(PED_NS_DONTWALK_PIN, HIGH);
  digitalWrite(PED_NS_WALK_PIN, LOW);
  digitalWrite(PED_EW_DONTWALK_PIN, HIGH);
  digitalWrite(PED_EW_WALK_PIN, LOW);
}

void setPedWalkState(bool nsWalk, bool ewWalk) {
  // DON'T WALK is active HIGH for the dedicated red LED
  digitalWrite(PED_NS_DONTWALK_PIN, nsWalk ? LOW : HIGH);
  digitalWrite(PED_NS_WALK_PIN, nsWalk ? HIGH : LOW);

  digitalWrite(PED_EW_DONTWALK_PIN, ewWalk ? LOW : HIGH);
  digitalWrite(PED_EW_WALK_PIN, ewWalk ? HIGH : LOW);
}

void updateTrafficLightsForState() {
  setAllTrafficRed();
  setAllPedDontWalk();

  switch (currentState) {
    case NS_GREEN:
      digitalWrite(NS_GREEN_PIN, HIGH);
      digitalWrite(SS_GREEN_PIN, HIGH);
      digitalWrite(NS_RED_PIN, LOW);
      digitalWrite(SS_RED_PIN, LOW);
      // East-West stays red
      break;

    case NS_YELLOW:
      digitalWrite(NS_YELLOW_PIN, HIGH);
      digitalWrite(SS_YELLOW_PIN, HIGH);
      digitalWrite(NS_RED_PIN, LOW);
      digitalWrite(SS_RED_PIN, LOW);
      break;

    case ALL_RED:
      setAllTrafficRed();
      setAllPedDontWalk();
      break;

    case EW_GREEN:
      digitalWrite(EW_GREEN_PIN, HIGH);
      digitalWrite(WE_GREEN_PIN, HIGH);
      digitalWrite(EW_RED_PIN, LOW);
      digitalWrite(WE_RED_PIN, LOW);
      break;

    case EW_YELLOW:
      digitalWrite(EW_YELLOW_PIN, HIGH);
      digitalWrite(WE_YELLOW_PIN, HIGH);
      digitalWrite(EW_RED_PIN, LOW);
      digitalWrite(WE_RED_PIN, LOW);
      break;

    case NS_WALK:
      setAllTrafficRed();
      setPedWalkState(true, false);
      break;

    case EW_WALK:
      setAllTrafficRed();
      setPedWalkState(false, true);
      break;
  }
}

// Debounced button edge detector.
// Because using INPUT_PULLUP, pressed means LOW.
bool detectButtonPress(uint8_t pin, bool &lastRawState, bool &pressedFlag, unsigned long &lastChangeMillis) {
  const bool raw = digitalRead(pin);

  if (raw != lastRawState) {
    lastRawState = raw;
    lastChangeMillis = millis();
  }

  if ((millis() - lastChangeMillis) >= BUTTON_DEBOUNCE_MS && raw == LOW && !pressedFlag) {
    pressedFlag = true;
    return true;
  }

  if (raw == HIGH) {
    pressedFlag = false;
  }

  return false;
}

void clearPedRequestForStream(TrafficState stream) {
  if (stream == NS_GREEN || stream == NS_YELLOW || stream == NS_WALK) {
    nsPedRequest = false;
  }

  if (stream == EW_GREEN || stream == EW_YELLOW || stream == EW_WALK) {
    ewPedRequest = false;
  }
}

void requestPedestrianCrossing(bool nsRequest, bool ewRequest) {
  if (nsRequest) {
    nsPedRequest = true;
  }

  if (ewRequest) {
    ewPedRequest = true;
  }
}

void enterState(TrafficState nextState) {
  currentState = nextState;
  stateStartMillis = millis();
  updateTrafficLightsForState();
}

void setup() {
  // Traffic LEDs
  pinMode(NS_RED_PIN, OUTPUT);
  pinMode(NS_YELLOW_PIN, OUTPUT);
  pinMode(NS_GREEN_PIN, OUTPUT);

  pinMode(SS_RED_PIN, OUTPUT);
  pinMode(SS_YELLOW_PIN, OUTPUT);
  pinMode(SS_GREEN_PIN, OUTPUT);

  pinMode(EW_RED_PIN, OUTPUT);
  pinMode(EW_YELLOW_PIN, OUTPUT);
  pinMode(EW_GREEN_PIN, OUTPUT);

  pinMode(WE_RED_PIN, OUTPUT);
  pinMode(WE_YELLOW_PIN, OUTPUT);
  pinMode(WE_GREEN_PIN, OUTPUT);

  // Pedestrian LEDs
  pinMode(PED_NS_WALK_PIN, OUTPUT);
  pinMode(PED_NS_DONTWALK_PIN, OUTPUT);
  pinMode(PED_EW_WALK_PIN, OUTPUT);
  pinMode(PED_EW_DONTWALK_PIN, OUTPUT);

  // Buttons with pull-up resistors
  pinMode(PED_NS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PED_EW_BUTTON_PIN, INPUT_PULLUP);

  // Set default state
  currentState = NS_GREEN;
  stateStartMillis = millis();
  updateTrafficLightsForState();
}

void loop() {
  // Read and latch pedestrian requests (software debounced)
  if (detectButtonPress(PED_NS_BUTTON_PIN, nsButtonLastRaw, nsButtonPressedFlag, nsButtonLastChange)) {
    nsPedRequest = true;
  }

  if (detectButtonPress(PED_EW_BUTTON_PIN, ewButtonLastRaw, ewButtonPressedFlag, ewButtonLastChange)) {
    ewPedRequest = true;
  }

  unsigned long elapsed = millis() - stateStartMillis;

  switch (currentState) {
    case NS_GREEN:
      lastGreenStream = NS_GREEN;
      if (elapsed >= GREEN_DURATION_MS) {
        enterState(NS_YELLOW);
      }
      break;

    case NS_YELLOW:
      if (elapsed >= YELLOW_DURATION_MS) {
        enterState(ALL_RED);
      }
      break;

    case ALL_RED:
      if (elapsed >= ALL_RED_DURATION_MS) {
        if (lastGreenStream == NS_GREEN) {
          if (nsPedRequest) {
            enterState(NS_WALK);
          } else if (ewPedRequest) {
            enterState(EW_WALK);
          } else {
            enterState(EW_GREEN);
          }
        } else {
          if (ewPedRequest) {
            enterState(EW_WALK);
          } else if (nsPedRequest) {
            enterState(NS_WALK);
          } else {
            enterState(NS_GREEN);
          }
        }
      }
      break;

    case EW_GREEN:
      lastGreenStream = EW_GREEN;
      if (elapsed >= GREEN_DURATION_MS) {
        enterState(EW_YELLOW);
      }
      break;

    case EW_YELLOW:
      if (elapsed >= YELLOW_DURATION_MS) {
        enterState(ALL_RED);
      }
      break;

    case NS_WALK:
      if (elapsed >= WALK_DURATION_MS) {
        nsPedRequest = false;
        enterState(EW_GREEN);
      }
      break;

    case EW_WALK:
      if (elapsed >= WALK_DURATION_MS) {
        ewPedRequest = false;
        enterState(NS_GREEN);
      }
      break;
  }

  updateTrafficLightsForState();
}
