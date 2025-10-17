// ==========================================
// ESP32 Durian Robot Motor Controller v2.1
// Production-Ready (Minimal Delays)
// ==========================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// === PIN DEFINITIONS ===
#define PWM_LEFT_PIN    15
#define PWM_RIGHT_PIN   2
#define DIR_LEFT_FWD    12
#define DIR_LEFT_BWD    13
#define DIR_RIGHT_FWD   14
#define DIR_RIGHT_BWD   27
#define LED_PIN         25

// === PWM CONFIGURATION ===
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8
#define PWM_CH_LEFT     0
#define PWM_CH_RIGHT    1
#define MAX_PWM         255
#define MIN_PWM         30

// === TIMING ===
#define CONTROL_LOOP_MS 50    // 20 Hz (ต้องเก็บ!)
#define WATCHDOG_MS     5000
#define SERIAL_TIMEOUT  2000

// === MOTOR STATE ===
struct MotorState {
  float speed_left;
  float speed_right;
  uint8_t pwm_left;
  uint8_t pwm_right;
  unsigned long last_cmd_time;
  bool connected;
  bool is_emergency_stop;
} motor_state = {0};

// === SYNCHRONIZATION ===
QueueHandle_t motor_cmd_queue;
SemaphoreHandle_t motor_state_mutex;

// ==========================================
// SETUP & INITIALIZATION
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n=== ESP32 Motor Controller v2.1 ===");
  Serial.println("Production Version\n");
  
  init_hardware();
  init_freertos();
  
  Serial.println("✓ Ready!\n");
}

void init_hardware() {
  // GPIO setup
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_LEFT_FWD, OUTPUT);
  pinMode(DIR_LEFT_BWD, OUTPUT);
  pinMode(DIR_RIGHT_FWD, OUTPUT);
  pinMode(DIR_RIGHT_BWD, OUTPUT);
  
  // PWM setup
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_LEFT_PIN, PWM_CH_LEFT);
  ledcAttachPin(PWM_RIGHT_PIN, PWM_CH_RIGHT);
  
  // Initial state
  motor_state.connected = false;
  motor_state.last_cmd_time = millis();
  motor_state.is_emergency_stop = false;
  
  digitalWrite(LED_PIN, LOW);
  emergency_stop();
}

void init_freertos() {
  motor_state_mutex = xSemaphoreCreateMutex();
  motor_cmd_queue = xQueueCreate(10, sizeof(char) * 64);
  
  xTaskCreatePinnedToCore(
    motor_control_task,
    "MotorControl",
    2048,
    NULL,
    3,  // High priority
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    serial_handler_task,
    "SerialHandler",
    4096,
    NULL,
    2,
    NULL,
    1
  );
  
  xTaskCreatePinnedToCore(
    watchdog_task,
    "Watchdog",
    1024,
    NULL,
    1,
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    status_led_task,
    "StatusLED",
    1024,
    NULL,
    1,
    NULL,
    1
  );
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  delay(1000);  // Idle
}

// ==========================================
// FREERTOS TASKS
// ==========================================

void motor_control_task(void* params) {
  TickType_t last_wake_time = xTaskGetTickCount();
  char cmd_buffer[64];
  
  Serial.println("[MotorControl] Started");
  
  while (1) {
    // Non-blocking queue check
    if (xQueueReceive(motor_cmd_queue, cmd_buffer, 0) == pdTRUE) {
      parse_and_execute_command(cmd_buffer);
    }
    
    // Non-blocking serial check
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.length() > 0 && input.length() < 64) {
        char temp[64];
        input.toCharArray(temp, 64);
        xQueueSendToBack(motor_cmd_queue, temp, 0);
      }
    }
    
    // KEEP THIS! Timing control
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

void serial_handler_task(void* params) {
  Serial.println("[SerialHandler] Started");
  
  while (1) {
    // Just monitor, don't do much
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void watchdog_task(void* params) {
  Serial.println("[Watchdog] Started");
  unsigned long last_cmd_time;
  
  while (1) {
    if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      last_cmd_time = motor_state.last_cmd_time;
      xSemaphoreGive(motor_state_mutex);
      
      unsigned long elapsed = millis() - last_cmd_time;
      
      if (elapsed > SERIAL_TIMEOUT && !motor_state.is_emergency_stop) {
        Serial.println("[Watchdog] ⚠️ Timeout - stopping");
        
        if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          emergency_stop();
          motor_state.is_emergency_stop = true;
          xSemaphoreGive(motor_state_mutex);
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void status_led_task(void* params) {
  Serial.println("[StatusLED] Started");
  bool led_state = false;
  
  while (1) {
    if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool connected = motor_state.connected;
      bool emergency = motor_state.is_emergency_stop;
      xSemaphoreGive(motor_state_mutex);
      
      if (emergency) {
        digitalWrite(LED_PIN, HIGH);  // Solid
        vTaskDelay(pdMS_TO_TICKS(500));
      } else if (connected) {
        // Slow blink
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state);
        vTaskDelay(pdMS_TO_TICKS(500));
      } else {
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }
  }
}

// ==========================================
// COMMAND PARSING
// ==========================================

void parse_and_execute_command(const char* cmd) {
  // M,left,right
  if (cmd[0] == 'M' && cmd[1] == ',') {
    char* ptr = (char*)(cmd + 2);
    float left = strtof(ptr, &ptr);
    
    if (*ptr == ',') {
      ptr++;
      float right = strtof(ptr, NULL);
      
      if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        set_motor_speed(left, right);
        motor_state.last_cmd_time = millis();
        motor_state.connected = true;
        motor_state.is_emergency_stop = false;
        xSemaphoreGive(motor_state_mutex);
      }
    }
  }
  else if (strcmp(cmd, "TEST") == 0) {
    run_motor_test();
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    print_status();
  }
  else if (strcmp(cmd, "STOP") == 0) {
    if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      emergency_stop();
      xSemaphoreGive(motor_state_mutex);
    }
  }
}

// ==========================================
// MOTOR CONTROL
// ==========================================

void set_motor_speed(float left_speed, float right_speed) {
  left_speed = constrain(left_speed, -1.0, 1.0);
  right_speed = constrain(right_speed, -1.0, 1.0);
  
  motor_state.speed_left = left_speed;
  motor_state.speed_right = right_speed;
  
  uint8_t pwm_left = speed_to_pwm(left_speed);
  uint8_t pwm_right = speed_to_pwm(right_speed);
  
  apply_motor_pwm(pwm_left, pwm_right);
  
  Serial.printf("M: L=%.2f(%d) R=%.2f(%d)\n", 
    left_speed, pwm_left, right_speed, pwm_right);
}

uint8_t speed_to_pwm(float normalized_speed) {
  if (normalized_speed == 0.0) return 0;
  
  float abs_speed = fabs(normalized_speed);
  uint8_t pwm = (uint8_t)(abs_speed * (MAX_PWM - MIN_PWM) + MIN_PWM);
  
  return pwm;
}

void apply_motor_pwm(uint8_t pwm_left, uint8_t pwm_right) {
  // Left motor
  if (motor_state.speed_left > 0) {
    digitalWrite(DIR_LEFT_FWD, HIGH);
    digitalWrite(DIR_LEFT_BWD, LOW);
  } else if (motor_state.speed_left < 0) {
    digitalWrite(DIR_LEFT_FWD, LOW);
    digitalWrite(DIR_LEFT_BWD, HIGH);
  } else {
    digitalWrite(DIR_LEFT_FWD, LOW);
    digitalWrite(DIR_LEFT_BWD, LOW);
  }
  
  // Right motor
  if (motor_state.speed_right > 0) {
    digitalWrite(DIR_RIGHT_FWD, HIGH);
    digitalWrite(DIR_RIGHT_BWD, LOW);
  } else if (motor_state.speed_right < 0) {
    digitalWrite(DIR_RIGHT_FWD, LOW);
    digitalWrite(DIR_RIGHT_BWD, HIGH);
  } else {
    digitalWrite(DIR_RIGHT_FWD, LOW);
    digitalWrite(DIR_RIGHT_BWD, LOW);
  }
  
  ledcWrite(PWM_CH_LEFT, pwm_left);
  ledcWrite(PWM_CH_RIGHT, pwm_right);
  
  motor_state.pwm_left = pwm_left;
  motor_state.pwm_right = pwm_right;
}

void emergency_stop() {
  motor_state.speed_left = 0.0;
  motor_state.speed_right = 0.0;
  apply_motor_pwm(0, 0);
  Serial.println("🛑 STOP");
}

// ==========================================
// TEST & DEBUG
// ==========================================

void run_motor_test() {
  Serial.println("\n=== Motor Test ===");
  
  // Quick test (500ms each, not 2000ms)
  test_motor(0.5, 0.5, "Forward", 500);
  test_motor(0.0, 0.5, "Right", 500);
  test_motor(0.5, 0.0, "Left", 500);
  test_motor(0.5, -0.5, "Turn-R", 500);
  test_motor(-0.5, 0.5, "Turn-L", 500);
  
  emergency_stop();
  Serial.println("✓ Test done\n");
}

void test_motor(float left, float right, const char* name, uint32_t duration) {
  Serial.printf("Test: %s\n", name);
  
  if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    set_motor_speed(left, right);
    xSemaphoreGive(motor_state_mutex);
  }
  
  // Non-blocking wait
  vTaskDelay(pdMS_TO_TICKS(duration));
}

void print_status() {
  if (xSemaphoreTake(motor_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println("\n=== Status ===");
    Serial.printf("Connected: %s\n", motor_state.connected ? "✓" : "✗");
    Serial.printf("Emergency: %s\n", motor_state.is_emergency_stop ? "YES" : "NO");
    Serial.printf("Left:  %.2f (PWM: %d)\n", motor_state.speed_left, motor_state.pwm_left);
    Serial.printf("Right: %.2f (PWM: %d)\n", motor_state.speed_right, motor_state.pwm_right);
    Serial.printf("Cmd age: %d ms\n", (int)(millis() - motor_state.last_cmd_time));
    Serial.printf("Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.println();
    xSemaphoreGive(motor_state_mutex);
  }
}