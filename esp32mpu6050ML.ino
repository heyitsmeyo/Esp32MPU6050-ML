/* Edge Impulse + MPU6050 Continuous Gesture Classification
 * Library: esp32mp_inferencing.h
 */

#include <esp32mp_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Sampling settings (must match what you used in Edge Impulse)
#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / FREQUENCY_HZ)

Adafruit_MPU6050 mpu;

// Buffer that will hold one full window of sensor data
// Edge Impulse expects: ax, ay, az, gx, gy, gz  (or whatever axes you trained with)
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static size_t feature_ix = 0;

static unsigned long last_interval_ms = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("MPU6050 + Edge Impulse Gesture Classifier");

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");

    // Match the settings you used when collecting data in Edge Impulse
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Ready. Move the sensor to classify gestures...");
    delay(500);
}

void loop() {
    if (millis() < last_interval_ms + INTERVAL_MS) {
        return;
    }
    last_interval_ms = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Fill buffer (adjust axes according to your project)
    features[feature_ix++] = a.acceleration.x;
    features[feature_ix++] = a.acceleration.y;
    features[feature_ix++] = a.acceleration.z;

    // Uncomment if you also use gyro:
    features[feature_ix++] = g.gyro.x;
    features[feature_ix++] = g.gyro.y;
    features[feature_ix++] = g.gyro.z;

    if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        feature_ix = 0;

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
            memcpy(out_ptr, features + offset, length * sizeof(float));
            return 0;
        };

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            ei_printf("ERR: %d\n", res);
            return;
        }

        // Find the best class
        float max_value = 0.0f;
        size_t max_idx = 0;
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > max_value) {
                max_value = result.classification[i].value;
                max_idx = i;
            }
        }

        // Only print if confidence is decent (optional)
        if (max_value > 0.60) {          // change 0.60 if you want
            ei_printf(">>> %s  (%.0f%%)\n", 
                      result.classification[max_idx].label, 
                      max_value * 100);
        }

        // Slow down the output so you can read it
        delay(800);   // ← increase this number if you want even slower
    }
}