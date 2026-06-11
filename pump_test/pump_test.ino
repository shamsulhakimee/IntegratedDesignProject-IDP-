/*
  Pump Relay Diagnostics Tool (HW-383 2-Channel Relay)
  
  This simple sketch is designed to test if the relay module and water pump 
  work correctly on GPIO 17 (D17).
  
  Since the HW-383 2-channel relay is an ACTIVE-LOW 5V relay, driving it from a 
  3.3V ESP32 can sometimes cause issues where the relay stays permanently ON.
  This sketch tests both standard active-low logic and the "High-Impedance Input Trick"
  which turns off the optocoupler completely by disabling the pin drive.
  
  View the results in your Serial Monitor (baud rate: 115200).
*/

#define TEST_RELAY_PIN 17

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect
  
  Serial.println("============================================");
  Serial.println("   G7 Solar Robot Pump Relay Test Utility   ");
  Serial.println("============================================");
  Serial.print("Testing Relay on GPIO ");
  Serial.println(TEST_RELAY_PIN);
}

// Helper function to turn pump ON/OFF safely
void setPump(bool turnOn) {
  if (turnOn) {
    pinMode(TEST_RELAY_PIN, OUTPUT);
    digitalWrite(TEST_RELAY_PIN, LOW); // Pull LOW to turn ON
    Serial.println(" -> STATE: ON (Output LOW)");
  } else {
    // Set to INPUT (high impedance) to float the pin and cut current flow,
    // which guarantees a 5V active-low relay turns OFF completely.
    pinMode(TEST_RELAY_PIN, INPUT);
    Serial.println(" -> STATE: OFF (High-Impedance INPUT)");
  }
}

void loop() {
  Serial.println("\n--- Starting Test Cycle ---");
  
  // 1. Turn Pump ON
  Serial.print("Turning Pump ON...");
  setPump(true);
  delay(5000); // Wait 5 seconds
  
  // 2. Turn Pump OFF
  Serial.print("Turning Pump OFF...");
  setPump(false);
  delay(5000); // Wait 5 seconds
}
