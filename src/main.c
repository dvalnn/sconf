#include <pico.h>
#include <pico/multicore.h>
#include <pico/stdio.h>
#include <pico/sync.h>

#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>

#include <stdio.h>

#define GPIO_WATER_SENS_1_NO 2
#define GPIO_WATER_SENS_1_NC 3

#define GPIO_WATER_SENS_2_NO 4
#define GPIO_WATER_SENS_2_NC 5

#define GPIO_TEMP_SENS_1 26
#define GPIO_TEMP_SENS_2 27

#define GPIO_OUT_RELAY 22

#define TEMPERATURE_MIN 1250
#define TEMPERATURE_MAX 3620

#define WATCHDOG_MS 1000

static struct mutex in_mtx;
static struct mutex out_mtx;

struct DigitalSensor {
  const uint normal_pin;
  const uint inverted_pin;

  bool value;
  bool is_valid;
};

void ds_check_read(struct DigitalSensor *ds) {
  ds->value = gpio_get(ds->normal_pin);
  bool inverted = gpio_get(ds->inverted_pin);
  ds->is_valid = (ds->value != inverted);
}

struct AnalogSensor {
  const uint pin;
  const uint max_valid;
  const uint min_valid;

  uint16_t value;
  bool is_valid;
};

void as_check_read(struct AnalogSensor *as) {
  adc_select_input(as->pin - ADC_BASE_PIN);
  as->value = adc_read();
  as->is_valid = (as->max_valid >= as->value && as->value >= as->min_valid);
}

bool is_within_10_percent(uint16_t reading1, uint16_t reading2) {
  // Calculate the absolute difference
  uint16_t difference =
      (reading1 > reading2) ? (reading1 - reading2) : (reading2 - reading1);

  // Calculate 10% of the average of the two readings
  uint16_t threshold = (uint16_t)(((float)(reading1 + reading2)) / 2 * 0.1);

  // Check if the difference is within the threshold
  return difference <= threshold;
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min,
             uint16_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup_hardware() {
  watchdog_enable(WATCHDOG_MS, false);
  stdio_init_all();

  // GPIO initialization
  gpio_init(GPIO_WATER_SENS_1_NO);
  gpio_set_dir(GPIO_WATER_SENS_1_NO, GPIO_IN);
  gpio_pull_down(GPIO_WATER_SENS_1_NO);

  gpio_init(GPIO_WATER_SENS_1_NC);
  gpio_set_dir(GPIO_WATER_SENS_1_NC, GPIO_IN);
  gpio_pull_down(GPIO_WATER_SENS_1_NC);

  gpio_init(GPIO_WATER_SENS_2_NO);
  gpio_set_dir(GPIO_WATER_SENS_2_NO, GPIO_IN);
  gpio_pull_down(GPIO_WATER_SENS_2_NO);

  gpio_init(GPIO_WATER_SENS_2_NC);
  gpio_set_dir(GPIO_WATER_SENS_2_NC, GPIO_IN);
  gpio_pull_down(GPIO_WATER_SENS_2_NC);

  adc_init();

  adc_gpio_init(GPIO_TEMP_SENS_1);
  adc_gpio_init(GPIO_TEMP_SENS_2);

  gpio_init(GPIO_OUT_RELAY);
  gpio_set_dir(GPIO_OUT_RELAY, GPIO_OUT);
  gpio_put(GPIO_OUT_RELAY, false);
};

void safety_func(uint16_t temperature, bool water, bool malfunction) {
  char *water_msg = water ? "ON" : "OFF";
  if (malfunction) {
    gpio_put(GPIO_OUT_RELAY, false);
    printf("[Core 0] malfunction detected!\n");
  } else if (!water && temperature >= 70) {
    gpio_put(GPIO_OUT_RELAY, false);
    printf("[Core 0] unsafe! temp: %d, water: %s\n", temperature, water_msg);
  } else {
    gpio_put(GPIO_OUT_RELAY, true);
    printf("[Core 0] safe! temp: %d, water: %s\n", temperature, water_msg);
  }
}

void core_1_main() {

  while (1) {
    tight_loop_contents();
  }
};

void core_0_main() {
  struct DigitalSensor water1 = {
      .normal_pin = GPIO_WATER_SENS_1_NO,
      .inverted_pin = GPIO_WATER_SENS_1_NC,
  };

  struct DigitalSensor water2 = {
      .normal_pin = GPIO_WATER_SENS_2_NO,
      .inverted_pin = GPIO_WATER_SENS_2_NC,
  };

  struct AnalogSensor temp1 = {
      .pin = GPIO_TEMP_SENS_1,
      .min_valid = TEMPERATURE_MIN,
      .max_valid = TEMPERATURE_MAX,
  };

  struct AnalogSensor temp2 = {
      .pin = GPIO_TEMP_SENS_2,
      .min_valid = TEMPERATURE_MIN,
      .max_valid = TEMPERATURE_MAX,
  };

  for (;;) {
    bool malfunction = false;

    ds_check_read(&water1);
    ds_check_read(&water2);

    as_check_read(&temp1);
    as_check_read(&temp2);

    // check water consistency
    if (!water1.is_valid) {
      printf("[Core 0] water1 failure\n");
      malfunction = true;
    }

    if (!water2.is_valid) {
      printf("[Core 0] water2 failure\n");
      malfunction = true;
    }

    if (water1.value != water2.value) {
      printf("[Core 0] water reading inconsistent\n");
      malfunction = true;
    }

    // check temperature consistency
    if (!temp1.is_valid) {
      printf("[Core 0] temp1 failure: %d\n", temp1.value);
      malfunction = true;
    }

    if (!temp2.is_valid) {
      printf("[Core 0] temp2 failure: %d\n", temp2.value);
      malfunction = true;
    }

    if (!is_within_10_percent(temp1.value, temp2.value)) {
      printf("[Core 0] temperature reading inconsistent\n");
      malfunction = true;
    }

    uint16_t temp_real_1 =
        map(temp1.value, temp1.min_valid, temp1.max_valid, 0, 100);
    uint16_t temp_real_2 =
        map(temp2.value, temp2.min_valid, temp2.max_valid, 0, 100);
    uint16_t temperature = MAX(temp_real_1, temp_real_2);

    safety_func(temperature, water1.value, malfunction);

    stdio_flush();
    watchdog_update();

    sleep_ms(100);
  }
}

int main() {
  setup_hardware();

  if (watchdog_caused_reboot()) {
    printf("watchdog reboot\n");
  } else {
    printf("normal boot\n");
  }
  stdio_flush();

  mutex_init(&in_mtx);
  mutex_init(&out_mtx);

  /* multicore_launch_core1(core_1_main); */

  core_0_main();
}
