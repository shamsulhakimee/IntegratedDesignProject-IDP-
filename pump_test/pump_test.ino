/*
  Pump Relay Diagnostics Tool
  
  This simple sketch is designed to test if the relay module and water pump 
  work correctly on GPIO 17 (D17).
  
  Upload this sketch to your ESP32. It will cycle the relay ON and OFF 
  every 5 seconds. You should hear a physical click from the relay module, 
  and the onboard status LED on the relay module should turn on/off.
  
  View the results in your Serial Monitor (baud rate: 115200).
*/

#define TEST_RELAY_PIN 17

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect
  
  Serial.println("============================================");
  Serial.println("   G7 Solar Robot Pump Relay Test Utility   ");
  Serial.println("============================================");
  Serial.print("Configuring Pin GPIO ");
  Serial.print(TEST_RELAY_PIN);
  Serial.println(" as OUTPUT.");
  
  pinMode(TEST_RELAY_PIN, OUTPUT);
  digitalWrite(TEST_RELAY_PIN, HIGH); // Set initial state
}

void loop() {
  Serial.println("\n--- Starting Test Cycle ---");
  
  // 1. ACTIVE LOW Test (Low signal triggers relay)
  Serial.println("[Test 1: Active LOW] Sending LOW to Pin 17 (Pump should turn ON for 5s)...");
  digitalWrite(TEST_RELAY_PIN, LOW);
  delay(5000);
  
  Serial.println("[Test 1: Active LOW] Sending HIGH to Pin 17 (Pump should turn OFF for 5s)...");
  digitalWrite(TEST_RELAY_PIN, HIGH);
  delay(5000);
  
  // 2. ACTIVE HIGH Test (High signal triggers relay)
  Serial.println("[Test 2: Active HIGH] Sending HIGH to Pin 17 (Pump should turn ON for 5s)...");
  digitalWrite(TEST_RELAY_PIN, HIGH);
  delay(5000);
  
  Serial.println("[Test 2: Active HIGH] Sending LOW to Pin 17 (Pump should turn OFF for 5s)...");
  digitalWrite(TEST_RELAY_PIN, LOW);
  delay(5000);
}
