# **IMU Sensor Fusion using BNO055**

## **Overview**
This project implements **sensor fusion** using the **Adafruit BNO055 IMU sensor**. The sensor provides acceleration, gyroscope, and magnetometer data, which are combined using a **complementary filter** to estimate the orientation of a device. The filtered roll (`theta`), pitch (`phi`), and yaw (`psi`) angles are computed in real-time and printed via serial output.

## **Hardware Requirements**
- **Microcontroller**: Arduino-compatible board
- **IMU Sensor**: Adafruit BNO055
- **Connections**:
  - **VCC** → 3.3V
  - **GND** → GND
  - **SDA** → SDA (A4 on Arduino Uno)
  - **SCL** → SCL (A5 on Arduino Uno)

## **Software Requirements**
- **Arduino IDE** (Latest Version)
- **Adafruit BNO055 Library**
- **Wire.h Library**

## **Installation & Setup**
### **1️⃣ Install Dependencies**
1. Open **Arduino IDE**
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for and install:
   - **Adafruit BNO055**
   - **Adafruit Unified Sensor**
   - **Wire.h** (already included with Arduino)

### **2️⃣ Upload the Code**
1. Copy the code into an **Arduino sketch**
2. Connect your **Arduino** via USB**
3. Select the correct **board and port**
4. Click **Upload**

### **3️⃣ Open Serial Monitor**
1. Go to **Tools → Serial Monitor**
2. Set baud rate to **115200**
3. Observe sensor readings in real-time

## **Explanation of the Code**
### **🔹 Sensor Data Acquisition**
- Accelerometer (`acc.x, acc.y, acc.z`)
- Gyroscope (`gyr.x, gyr.y, gyr.z`)
- Magnetometer (`mag.x, mag.y, mag.z`)

### **🔹 Filters Implemented**
1. **Low-Pass Filter** (Smoothing accelerometer data)
   ```cpp
   theta_acc_F_new = p1 * theta_acc_F_old + p2 * theta_acc_M;
   phi_acc_F_new = p1 * phi_acc_F_old + p2 * phi_acc_M;
   ```
2. **Gyroscope Angle Integration**
   ```cpp
   theta_gyr_M = theta_gyr_M + gyr.y() * dt;
   phi_gyr_M = phi_gyr_M + gyr.x() * dt;
   ```
3. **Complementary Filter** (Fusion of accelerometer & gyroscope data)
   ```cpp
   theta_overall = (theta_overall - gyr.y() * dt) * p1 + theta_acc_M * p2;
   phi_overall = (phi_overall + gyr.x() * dt) * p1 + phi_acc_M * p2;
   ```
4. **Magnetometer Integration (Yaw Calculation)**
   ```cpp
   psi_mag = atan2(y_mag, x_mag) * 180 / PI;
   ```

## **Serial Output Format**
The data is printed in **CSV format**, containing:
```
acc_x, acc_y, acc_z, accel, gyro, magn, system_calib, theta_acc_M, theta_acc_F_new, phi_acc_M, phi_acc_F_new, theta_gyr_M, phi_gyr_M, theta_overall, phi_overall, psi_mag
```

Example Output:
```
0.01, -0.02, 9.80, 1, 1, 1, 3, 0.56, 0.50, -1.23, -1.10, -0.05, 0.02, 0.40, -0.95, 30.12
```

## **Further Improvements**
- Implement **Kalman Filter** for better fusion
- Apply **Quaternion Representation** for orientation tracking
- Use **Sensor Fusion Libraries** like **Madgwick** or **Mahony**

## **Author**
- **Rajesh B** | GitHub: [rajeshb0202](https://github.com/rajeshb0202)

## **License**
This project is open-source and licensed under the **MIT License**.