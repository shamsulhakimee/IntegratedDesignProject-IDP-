# G7 Solar Panel Cleaning Robot - System Documentation

This repository contains the baseline open-loop control firmware and interactive web dashboard for the **G7 Solar Panel Cleaning Robot**. The system features real-time manual remote control (RC), autonomous safety cliff-detection and evasion, preset movement recording and playback, and live system diagnostics.

---

## 🛠️ System Architecture

The hardware architecture revolves around an ESP32 microcontroller that interfaces with four cliff sensors, a water pump relay, and a dual-channel motor driver. The dashboard is hosted directly by the ESP32 and accessed over its local WiFi Access Point.

### Hardware Block Diagram

```mermaid
graph TD
    %% Styling
    classDef esp32 fill:#2c3e50,stroke:#34495e,stroke-width:2px,color:#fff;
    classDef input fill:#27ae60,stroke:#2ecc71,stroke-width:2px,color:#fff;
    classDef output fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#fff;
    classDef client fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#fff;
    classDef power fill:#f1c40f,stroke:#f39c12,stroke-width:2px,color:#000;
    classDef reserved fill:#7f8c8d,stroke:#95a5a6,stroke-width:2px,color:#fff;

    %% Nodes
    subgraph PowerSystem [Power Delivery]
        Battery[12V Battery Pack]
        Regulator[5V/3.3V Step-Down Regulator]
    end

    subgraph Core [Control Unit]
        ESP32[ESP32 Development Board]
    end

    subgraph Inputs [Sensors & Inputs]
        FL[Front-Left Cliff Sensor<br>FS80NK - GPIO 32]
        FR[Front-Right Cliff Sensor<br>FS80NK - GPIO 33]
        BL[Back-Left Cliff Sensor<br>FS80NK - GPIO 27]
        BR[Back-Right Cliff Sensor<br>FS80NK - GPIO 14]
    end

    subgraph Outputs [Actuators & Drivers]
        Cytron[Cytron SPG20HP-34K Motor Driver]
        LMotor[Left Motor]
        RMotor[Right Motor]
        Relay[Relay Board<br>GPIO 17]
        Pump[Water Pump]
    end

    subgraph Communication [Wireless Interface]
        AP[WiFi Access Point<br>SSID: ESP32_Car]
        Client[Web Browser Client<br>Dashboard Dashboard]
    end

    subgraph Expansion [Reserved Interfaces]
        I2C[I2C Bus<br>SDA: GPIO 21 / SCL: GPIO 22]
    end

    %% Connections
    Battery --> Cytron
    Battery --> Regulator
    Regulator --> ESP32
    Regulator --> Inputs
    Regulator --> Relay

    FL -->|Digital Input| ESP32
    FR -->|Digital Input| ESP32
    BL -->|Digital Input| ESP32
    BR -->|Digital Input| ESP32

    ESP32 -->|PWM GPIO 18 & DIR GPIO 19| Cytron
    ESP32 -->|PWM GPIO 25 & DIR GPIO 26| Cytron
    Cytron --> LMotor
    Cytron --> RMotor

    ESP32 -->|Active-LOW / High-Z GPIO 17| Relay
    Relay --> Pump

    ESP32 <-->|WiFi / HTTP REST API| Client
    ESP32 -.->|Reserved Port| I2C

    %% Assign styles
    class ESP32 esp32;
    class FL,FR,BL,BR input;
    class Cytron,LMotor,RMotor,Relay,Pump output;
    class Battery,Regulator power;
    class AP,Client client;
    class I2C reserved;
```

---

## ⚙️ System Hardware Specifications

### 1. Motor Configuration
- **Motors**: Dual Cytron SPG20HP-34K (12V 580RPM DC geared motors).
- **Speed Limits**:
  - `MAX_PWM = 60` (approx. 24% duty cycle) — enforces slow, safe, and controllable tracking speeds on the solar panel arrays.
  - `MIN_PWM = 30` — minimum starting duty cycle to overcome mechanical stiction.
  - Manual joystick controls from the dashboard gamepad (`[-255..255]`) are mapped dynamically to the `[MIN_PWM..MAX_PWM]` speed range.

### 2. Water Pump Relay
- Controlled via an optocoupler-isolated relay module mapped to `GPIO 17`.
- **High-Impedance Trick**: Active-LOW signaling triggers the relay (`LOW` = ON). To ensure the 5V relay turns OFF completely under 3.3V logic (preventing leakage currents), the GPIO pin is configured as a high-impedance `INPUT` when turning the pump OFF, and driven `OUTPUT` `LOW` to turn it ON.
- Managed directly via the dashboard buttons or controller buttons (**Button X** turns pump ON, **Button Y** turns pump OFF).

### 3. Preset Recording & Playback
- **Button A**: Toggles manual movement sequence recording (capture rate: 10 Hz).
- **Button B**: Toggles playback execution of the recorded movement sequence.
- Sequence buffer capacity is up to 3000 steps (300 seconds of runtime), stored in the ESP32 RAM for zero-latency local execution.

---

## 🔌 Wiring and Pin Map

| Component | Pin Function | ESP32 GPIO Pin | Notes |
|---|---|---|---|
| **Left Motor** | PWM Speed Control | **GPIO 18** | Connected to Cytron PWM input (LEDC channel) |
| **Left Motor** | Direction Control | **GPIO 19** | `HIGH` is backward, `LOW` is forward (inverted wiring) |
| **Right Motor**| PWM Speed Control | **GPIO 25** | Connected to Cytron PWM input (LEDC channel) |
| **Right Motor**| Direction Control | **GPIO 26** | `HIGH` is forward, `LOW` is backward |
| **Water Pump** | Relay Signal | **GPIO 17** | Active-LOW (Low = Pump ON, Input/High-Z = Pump OFF) |
| **I2C SDA** | Data line | **GPIO 21** | Untouched / Reserved for future expansion |
| **I2C SCL** | Clock line | **GPIO 22** | Untouched / Reserved for future expansion |
| **Sensor FL** | Front-Left Cliff | **GPIO 32** | FS80NK Proximity Input (`INPUT_PULLUP`) |
| **Sensor FR** | Front-Right Cliff| **GPIO 33** | FS80NK Proximity Input (`INPUT_PULLUP`) |
| **Sensor BL** | Back-Left Cliff | **GPIO 27** | FS80NK Proximity Input (`INPUT_PULLUP`) |
| **Sensor BR** | Back-Right Cliff | **GPIO 14** | FS80NK Proximity Input (`INPUT_PULLUP`) |

---

## 💻 Control Loop and Flowchart

The firmware control loop runs in a non-blocking configuration to handle client connections, process safety updates, and execute autonomous sequences concurrently.

```mermaid
flowchart TD
    %% Styling
    classDef process fill:#34495e,stroke:#2c3e50,stroke-width:1.5px,color:#fff;
    classDef decision fill:#d35400,stroke:#e67e22,stroke-width:1.5px,color:#fff;
    classDef startstop fill:#27ae60,stroke:#2ecc71,stroke-width:2px,color:#fff;

    Start([Power On / Reset]) --> Init[Initialize Peripherals]
    Init --> PinModes[Configure GPIO PinModes<br>Motors, Sensors, Relay]
    PinModes --> I2CInit[Initialize I2C Wire Library]
    I2CInit --> WiFiInit[Start WiFi SoftAP<br>SSID: ESP32_Car]
    WiFiInit --> WebInit[Configure HTTP Routes & Start Server]
    WebInit --> LoopStart[Enter Loop]

    LoopStart --> ServerHandle[server.handleClient<br>Process HTTP Requests]
    ServerHandle --> SafetyCheck{Is Safety Mode Active?}
    
    %% Safety Mode Active path
    SafetyCheck -->|Yes| ReadSensors[Read FL, FR, BL, BR Sensors]
    ReadSensors --> EvasionStateCheck{Current Safety State?}

    EvasionStateCheck -->|STATE_NORMAL| DetectCliff{Any Cliff Detected?}
    DetectCliff -->|Yes| TransitionEvade[Determine Evasion Action<br>Disable Manual Control & Playback]
    TransitionEvade --> SetEvadeMotors[Set Motors Direct For Evasion]
    SetEvadeMotors --> LoopEnd
    
    DetectCliff -->|No| PlaybackCheck
    
    EvasionStateCheck -->|STATE_EVADE_TURN / STRAIGHT| TimerCheck{Evasion Phase Timer Elapsed?}
    TimerCheck -->|Yes| NextEvadePhase[Transition to Next Phase<br>e.g., Turn -> Straight -> Stop Wait]
    NextEvadePhase --> LoopEnd
    TimerCheck -->|No| LoopEnd

    EvasionStateCheck -->|STATE_STOP_WAIT| WaitTimer{500ms Elapsed?}
    WaitTimer -->|Yes| AllSafe{All Sensors Safe?}
    AllSafe -->|Yes| RestoreControl[Restore Control<br>State -> STATE_NORMAL]
    RestoreControl --> LoopEnd
    AllSafe -->|No| LoopEnd
    WaitTimer -->|No| LoopEnd

    %% Safety Mode Bypassed path
    SafetyCheck -->|No| ResetState{Safety State != STATE_NORMAL?}
    ResetState -->|Yes| ForceNormal[Reset State to STATE_NORMAL<br>Release Motor Locks]
    ForceNormal --> PlaybackCheck
    ResetState -->|No| PlaybackCheck

    %% Playback Processing
    PlaybackCheck{Is Autonomous Playback Running?}
    PlaybackCheck -->|Yes| PlaybackTimer{100ms Interval Elapsed?}
    PlaybackTimer -->|Yes| StepIndexCheck{Playback Index < Sequence Length?}
    StepIndexCheck -->|Yes| ExecuteStep[Apply Recorded Motor PWM & Pump State<br>Increment Playback Index]
    ExecuteStep --> LoopEnd
    StepIndexCheck -->|No| StopPlayback[Halt Motors<br>Set Playback Stopped]
    StopPlayback --> LoopEnd
    PlaybackTimer -->|No| LoopEnd
    PlaybackCheck -->|No| LoopEnd

    LoopEnd[End of Loop Cycle] --> LoopStart

    class Start,LoopStart startstop;
    class SafetyCheck,EvasionStateCheck,DetectCliff,TimerCheck,WaitTimer,AllSafe,ResetState,PlaybackCheck,PlaybackTimer,StepIndexCheck decision;
    class Init,PinModes,I2CInit,WiFiInit,WebInit,ServerHandle,ReadSensors,TransitionEvade,SetEvadeMotors,NextEvadePhase,RestoreControl,ForceNormal,ExecuteStep,StopPlayback process;
```

---

## 🚦 Safety Evasion State Machine

When Safety Mode is active, manual remote control and autonomous sequence playback are bypassed immediately if any of the FS80NK infrared proximity sensors detect a cliff (air edge). The safety state machine overrides control and performs a predefined evasion maneuver.

### Safety Evasion Logic:
1. **Front Cliff (FL or FR detects)**:
   - Stops motors.
   - Moves backward (`setMotorsDirect(-255, -255)`) for `1200ms`.
   - Stops and waits for safety verification.
2. **Back Cliff (BL or BR detects)**:
   - Stops motors.
   - Moves forward (`setMotorsDirect(255, 255)`) for `1200ms`.
   - Stops and waits for safety verification.
3. **Left Cliff (FL and BL detect)**:
   - Stops motors.
   - Pivots right (`setMotorsDirect(255, -255)`) for `1000ms`.
   - Repositions straight forward (`setMotorsDirect(255, 255)`) for `800ms`.
   - Stops and waits for safety verification.
4. **Right Cliff (FR and BR detect)**:
   - Stops motors.
   - Pivots left (`setMotorsDirect(-255, 255)`) for `1000ms`.
   - Repositions straight forward (`setMotorsDirect(255, 255)`) for `800ms`.
   - Stops and waits for safety verification.

### Safety State Machine Diagram

```mermaid
stateDiagram-v2
    [*] --> STATE_NORMAL : Power On
    
    state STATE_NORMAL {
        [*] --> GamepadDriving
    }

    STATE_NORMAL --> STATE_LEFT_EVADE_TURN : FL & BL detect Cliff (Left Edge)
    STATE_NORMAL --> STATE_RIGHT_EVADE_TURN : FR & BR detect Cliff (Right Edge)
    STATE_NORMAL --> STATE_FRONT_EVADE : FL or FR detects Cliff (Front Edge)
    STATE_NORMAL --> STATE_BACK_EVADE : BL or BR detects Cliff (Back Edge)

    STATE_LEFT_EVADE_TURN --> STATE_LEFT_EVADE_STRAIGHT : Turn Right complete (1000ms)
    STATE_LEFT_EVADE_STRAIGHT --> STATE_STOP_WAIT : Reposition complete (800ms)

    STATE_RIGHT_EVADE_TURN --> STATE_RIGHT_EVADE_STRAIGHT : Turn Left complete (1000ms)
    STATE_RIGHT_EVADE_STRAIGHT --> STATE_STOP_WAIT : Reposition complete (800ms)

    STATE_FRONT_EVADE --> STATE_STOP_WAIT : Reverse complete (1200ms)
    STATE_BACK_EVADE --> STATE_STOP_WAIT : Forward complete (1200ms)

    state STATE_STOP_WAIT {
        [*] --> CheckSensors
        CheckSensors --> ReleaseLock : All Safe (Wait 500ms)
        CheckSensors --> HoldLock : Cliff Still Present
    }

    STATE_STOP_WAIT --> STATE_NORMAL : Control Restored
```

---

## 💻 Web Server & API Endpoints

The ESP32 broadcasts a SoftAP SSID **`ESP32_Car`** (password: `password123`) and hosts the interactive dashboard on `http://192.168.4.1/`.

### Available Endpoints:
- **`GET /`**: Serves the dashboard HTML interface.
- **`GET /drive?l=[speed]&r=[speed]&p=[0/1]`**:
  - Updates motor speeds (scaled from `-255..255`) and water pump relay state (`p=1` for ON, `p=0` for OFF).
  - *Note: These commands are ignored if safety override maneuvers or preset playbacks are running.*
- **`GET /set_safety?active=[0/1]`**:
  - Enables (`active=1`) or disables (`active=0`) the active safety cliff-evasion state machine.
- **`GET /set_pump?active=[0/1]`**:
  - Direct control endpoint to toggle the water pump ON/OFF.
- **`POST /upload_preset`**:
  - Receives a movement sequence formatted as `l,r,p;l,r,p;...` in the POST payload and writes it directly to ESP32 memory.
- **`GET /playback?action=[play/pause/stop/clear]`**:
  - Controls playback actions for autonomous sequence execution.
- **`GET /status`**:
  - Queries real-time telemetry variables from the ESP32:
    ```json
    {
      "fl": false,
      "fr": false,
      "bl": false,
      "br": false,
      "safety": "NORMAL",
      "playStatus": "STOPPED",
      "playIndex": 0,
      "playLen": 0,
      "safetyMode": false,
      "pump": false,
      "uptime": 234850,
      "rssi": -48,
      "clients": 1
    }
    ```
- **`GET /log`**:
  - Returns the rolling event log buffer stored in ESP32 RAM as JSON:
    ```json
    [
      {"t":10240,"msg":"[WiFi] Client connected"},
      {"t":12450,"msg":"Safety mode ENABLED (Cleaning)"},
      {"t":14500,"msg":"Water pump turned ON"}
    ]
    ```
- **`GET /testleft` / `GET /testright`**: Turns left or right motor forward at `MAX_PWM` for hardware wiring debugging.
- **`GET /teststop`**: Emergency halts motors immediately.
- **`GET /estop`**: Emergency stop that halts motors, pump, and halts playback immediately.

---

## 🔍 Troubleshooting and Calibration

1. **Active Logic Tuning**:
   - The FS80NK output is active HIGH (`#define CLIFF_STATE HIGH` means high when cliff is detected). If your sensors output active LOW (LOW for cliff, HIGH for surface), update the macro in [SolarPanelG7RC.ino](file:///c:/Users/Victus/OneDrive/Desktop/IDP/SolarPanelG7RC/SolarPanelG7RC.ino) to `LOW`.
2. **Detection Threshold Adjustment**:
   - Locate the screw potentiometer on the back of each FS80NK sensor. Adjust the potentiometer using a flathead screwdriver until the indicator LEDs turn off cleanly when the robot reaches the edge of the panel.
3. **Safety Override Mode**:
   - If sensors are physically disconnected or damaged, they pull the inputs to HIGH (triggering safety lock). Bypass the safety state machine using the "Bypass Safety" button on the dashboard or by pressing **Button R1 (Right Bumper)** on your controller to restore manual operation.
