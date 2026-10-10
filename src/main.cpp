#include <Arduino.h>

const int NUM_SENSORS = 3;
const int fsrPins[NUM_SENSORS] = {32, 33, 35};  // 三个传感器的模拟引脚

// 阈值：根据你的传感器读数调整（未按压≈4095，按压时下降）
const int unpressedThreshold = 4000;
const int pressedThreshold = 3000;

const unsigned long unpressedDuration = 5000;   // 5秒

// 每个传感器的状态
struct SensorState {
  unsigned long unpressedStartTime;
  unsigned long lastPlayTime;
  bool isUnpressed;
};

SensorState sensors[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(100);
  Serial.println("FSR Test Start (3 sensors)");
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i].unpressedStartTime = 0;
    sensors[i].lastPlayTime = 0;
    sensors[i].isUnpressed = false;
  }
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int fsrValue = analogRead(fsrPins[i]);

    // 输出每个传感器的原始值
    Serial.print("FSR");
    Serial.print(i + 1);
    Serial.print(" Value: ");
    Serial.println(fsrValue);

    // 判断按压状态
    bool currentlyUnpressed = (fsrValue >= unpressedThreshold);
    bool currentlyPressed = (fsrValue < pressedThreshold);

    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" Status: ");
    if (currentlyUnpressed) {
      Serial.println("Not pressed");
    } else if (currentlyPressed) {
      Serial.println("Pressed");
    } else {
      Serial.println("Transition");
    }

    // 周期性发送播放指令
    if (currentlyUnpressed) {
      if (!sensors[i].isUnpressed) {
        // 刚进入未按压状态：重置计时
        sensors[i].isUnpressed = true;
        sensors[i].unpressedStartTime = millis();
        sensors[i].lastPlayTime = 0;
      }

      unsigned long now = millis();
      // 首次触发（5秒后）
      if (sensors[i].lastPlayTime == 0 &&
          (now - sensors[i].unpressedStartTime >= unpressedDuration)) {
        Serial.print("PLAY_AUDIO_");
        Serial.println(i + 1);
        sensors[i].lastPlayTime = now;
      }
      // 之后每隔5秒触发一次
      else if (sensors[i].lastPlayTime != 0 &&
               (now - sensors[i].lastPlayTime >= unpressedDuration)) {
        Serial.print("PLAY_AUDIO_");
        Serial.println(i + 1);
        sensors[i].lastPlayTime = now;
      }
    } else {
      // 有按压：重置该传感器的状态
      sensors[i].isUnpressed = false;
      sensors[i].lastPlayTime = 0;
    }
  }

  delay(100);
}