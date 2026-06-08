# G7 Solar Panel Cleaning Robot - Walkthrough & System Explanation

This walkthrough covers both the **FS80NK Cliff Detection and Evasion System** and the newly added **GY-85 Inertial Sensor Closed-Loop Feedback Control System**.

---

## 🛠️ System Overview & Architectures

The repository provides two separate control system configurations:
1. **Open-Loop Configuration** ([SolarPanelG7RC.ino](file:///c:/Users/Victus/OneDrive/Desktop/IDP/SolarPanelG7RC/SolarPanelG7RC.ino)): Focuses on manual remote control (RC) and the autonomous safety cliff-detection and evasion system powered by four FS80NK infrared proximity sensors.
2. **Closed-Loop Configuration** (in the [SolarPanelG7RC_ClosedLoop](file:///c:/Users/Victus/OneDrive/Desktop/IDP/SolarPanelG7RC/SolarPanelG7RC_ClosedLoop) folder): Re-integrates the GY-85 IMU to implement straight-line assist heading correction and slope speed compensation, and provides real-time PID slider tuning directly from the web dashboard.

---

## 🚦 Safety Evasion State Machine (Both Versions)

Both versions utilize a non-blocking state machine `updateSafety()` running on the ESP32. If a cliff sensor detects the edge (outputs active `HIGH`), manual RC and presets are instantly bypassed to execute an evasion maneuver:

![Safety Evasion State Machine Flowchart](safety_flowchart.png)

```mermaid
stateDiagram-v2
    [*] --> STATE_NORMAL : Start / Stable
    
    state STATE_NORMAL {
        [*] --> Idle_Or_Driving
    }

    STATE_NORMAL --> STATE_LEFT_EVADE_TURN : FL & BL detect Cliff
    STATE_NORMAL --> STATE_RIGHT_EVADE_TURN : FR & BR detect Cliff
    STATE_NORMAL --> STATE_FRONT_EVADE : FL or FR detects Cliff
    STATE_NORMAL --> STATE_BACK_EVADE : BL or BR detects Cliff

    STATE_LEFT_EVADE_TURN --> STATE_LEFT_EVADE_STRAIGHT : Turn Right complete (1000ms)
    STATE_LEFT_EVADE_STRAIGHT --> STATE_STOP_WAIT : Straight move complete (800ms)

    STATE_RIGHT_EVADE_TURN --> STATE_RIGHT_EVADE_STRAIGHT : Turn Left complete (1000ms)
    STATE_RIGHT_EVADE_STRAIGHT --> STATE_STOP_WAIT : Straight move complete (800ms)

    STATE_FRONT_EVADE --> STATE_STOP_WAIT : Reverse complete (1200ms)
    STATE_BACK_EVADE --> STATE_STOP_WAIT : Forward complete (1200ms)

    state STATE_STOP_WAIT {
        [*] --> Check_Cliffs_Clear
        Check_Cliffs_Clear --> Release_Lock : All Sensors Safe (Wait 500ms)
        Check_Cliffs_Clear --> Stay_Locked : Any Sensor Still at Cliff
    }

    STATE_STOP_WAIT --> STATE_NORMAL : Control Returned
```

---

## 🧭 Closed-Loop Stabilization Loop (`SolarPanelG7RC_ClosedLoop`)

In the closed-loop version, when the safety state is normal, the robot uses the GY-85 IMU to compensate for tilts and rotation:

```mermaid
flowchart TD
    subgraph Sensors [GY-85 9DOF Sensors]
        ADXL345[ADXL345 Accelerometer]
        ITG3205[ITG3205 Gyroscope]
        QMC5883L[QMC5883L Magnetometer]
    end
    
    subgraph Logic [Control Computations]
        Pitch[Calculate Pitch Angle]
        YawRate[Filter Yaw Rate Z-Axis]
        Compass[Normalize Compass Azimuth]
    end
    
    subgraph Correction [Control Corrections]
        SlopeComp[Slope Compensation: Target PWM + Pitch * K_Slope]
        YawAssist[Straight-Line Assist: Target Yaw 0.0 vs Gyro Yaw Rate PI correction]
    end

    ADXL345 --> Pitch
    ITG3205 --> YawRate
    QMC5883L --> Compass
    
    Pitch --> SlopeComp
    YawRate --> YawAssist
```

### 1. Mathematical Logic
* **Slope Compensation**: When driving forward or backward, pitch tilt gravity pull is countered by adjusting the base speed:
  $$V_{\text{compensated}} = V_{\text{target}} + \theta_{\text{pitch}} \times K_{\text{slope}}$$
* **Straight-Line Lock**: When driving straight, yaw rotation rate is locked to `0` using a PI loop:
  $$e(t) = 0 - Y_{\text{rate}}(t)$$
  $$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}$$
  $$L_{\text{output}} = V_{\text{compensated}} + u(t)$$
  $$R_{\text{output}} = V_{\text{compensated}} - u(t)$$

---

## 💻 Web Dashboard Upgrades

The closed-loop version's dashboard is fully upgraded with:
1. **Compass Card**: Integrates a virtual needle reflecting the magnetometer's azimuth.
2. **3D Tilt Orientation Card**: Embeds a CSS 3D transformed disk displaying the physical inclination (pitch/roll) of the robot.
3. **PID Tuning Card**: Adds interactive sliders to modify $K_p$, $K_i$, and $K_d$ values on the ESP32 in real time with a 150ms debouncer.
