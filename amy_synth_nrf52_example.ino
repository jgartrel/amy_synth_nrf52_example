#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <bluefruit.h>
#include <nrf_i2s.h>
#include <nrfx_i2s.h>
#include <AMY-Arduino.h>

#define PIN_I2S_BCLK D2   // 017 on the supermini
#define PIN_I2S_LRCLK D3  // 020 on the supermini
#define PIN_I2S_SDOUT D4  // 022 on the supermini
#define PIN_PWR_SPK D5    // 024 on the supermini

#define NRF_I2S_AUDIO_PRIORITY      6         ///< requested priority of the I2S peripheral

/* ---------------- Audio config ---------------- */
#define AUDIO_BLOCK_FRAMES AMY_BLOCK_SIZE
#define AUDIO_CHANNELS     2
#define AUDIO_SAMPLES      (AUDIO_BLOCK_FRAMES * AUDIO_CHANNELS)

/* ---------------- Audio buffers ---------------- */
static int16_t i2s_buffer_0[AUDIO_SAMPLES];
static int16_t i2s_buffer_1[AUDIO_SAMPLES];

/* ---------------- Buffer / underrun state ---------------- */
static volatile bool buffer_0_active;
static volatile bool render_buf0;
static volatile bool render_buf1;

static volatile bool buf0_ready;
static volatile bool buf1_ready;

static volatile uint32_t iteration_count;
static volatile uint32_t underrun_count;
static volatile uint32_t i2s_write_count;
static volatile uint32_t i2s_write_bytes;
static volatile uint32_t amy_task_count;
static volatile uint32_t amy_update_count;
static volatile uint32_t zero_buffer_count;

extern unsigned char __HeapBase[];
extern unsigned char __HeapLimit[];

/* ---------------- Playback state ---------------- */
static bool playback_active;
static uint32_t playback_start_ms;

// Allocation of handles and memory for tasks, queues, and semaphores
#define TASK_STACK_SIZE_DFLT 1024
xTaskHandle g_pAmyTaskHandle = NULL;

// Allocation static memory for tasks
StackType_t g_amyTaskStackMem[TASK_STACK_SIZE_DFLT];
StaticTask_t g_amyTaskTCB;

bool g_debug_task = false;
bool g_force_wdfail = false;
uint32_t update_duration_ema = 0;
uint32_t update_duration_max = 0;

/* ---------------- I2S handler (NRFX style) ---------------- */
static void i2s_event_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status)
{
  if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    return;

  BaseType_t xHigherPriorityTaskWoken;
  nrfx_i2s_buffers_t next;

  if (buffer_0_active) {
    if (!buf1_ready) underrun_count++;
    next.p_tx_buffer = (const uint32_t*)i2s_buffer_1;
    render_buf1 = true;
    buf1_ready = false;
  } else {
    if (!buf0_ready) underrun_count++;
    next.p_tx_buffer = (const uint32_t*)i2s_buffer_0;
    render_buf0 = true;
    buf0_ready = false;
  }

  next.p_rx_buffer = nullptr;
  buffer_0_active = !buffer_0_active;
  nrfx_i2s_next_buffers_set(&next);
  // Notify Amy task to render next sample block
  if ( g_pAmyTaskHandle != NULL ) {
    xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(g_pAmyTaskHandle, 1, eIncrement, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/* ---------------- I2S helpers ---------------- */
static void i2s_init() {

  nrfx_i2s_config_t config =
  {
    .sck_pin      = PIN_I2S_BCLK,
    .lrck_pin     = PIN_I2S_LRCLK,
    .mck_pin      = NRFX_I2S_PIN_NOT_USED,
    .sdout_pin    = PIN_I2S_SDOUT,
    .sdin_pin     = NRFX_I2S_PIN_NOT_USED,
    .irq_priority = NRF_I2S_AUDIO_PRIORITY,
    .mode         = NRF_I2S_MODE_MASTER,
    .format       = NRF_I2S_FORMAT_I2S,
    .alignment    = NRF_I2S_ALIGN_LEFT,
    .sample_width = NRF_I2S_SWIDTH_16BIT,
    .channels     = NRF_I2S_CHANNELS_STEREO,
    .mck_setup    = NRF_I2S_MCK_32MDIV32,   // 31.250KHz ~(32MDIV32 = 1.000MHz / 32)
    .ratio        = NRF_I2S_RATIO_32X
  };

  nrfx_i2s_init(&config, i2s_event_handler);
}

static void i2s_start() {
  // Prime buffers with amy_simple_fill_buffer + memcpy
  int16_t *buf0 = amy_simple_fill_buffer();
  memcpy(i2s_buffer_0, buf0, AUDIO_SAMPLES * sizeof(int16_t));

  int16_t *buf1 = amy_simple_fill_buffer();
  memcpy(i2s_buffer_1, buf1, AUDIO_SAMPLES * sizeof(int16_t));

  buf0_ready = true;
  buf1_ready = true;
  buffer_0_active = true;

  nrfx_i2s_buffers_t initial;
  initial.p_tx_buffer = (const uint32_t*)i2s_buffer_0;
  initial.p_rx_buffer = nullptr;

  // Power on the SPK
  pinMode(PIN_PWR_SPK, INPUT_PULLUP);
  //delay(13);

  nrfx_i2s_start(&initial, AUDIO_BLOCK_FRAMES, 0);
}

static void i2s_stop() {
  nrfx_i2s_stop();
  // Power off the SPK
  pinMode(PIN_PWR_SPK, INPUT_PULLDOWN);
  delay(13);
  nrfx_i2s_uninit();
}

void stop_midi() {
}

void run_midi() {
}

void amy_platform_init() {
  i2s_init();
  i2s_start();
}

void amy_platform_deinit() {
  i2s_stop();
}

void amy_update_tasks() {
  amy_execute_deltas();
}

int16_t *amy_render_audio() {
    amy_render(0, AMY_OSCS, 0);
    int16_t *block = amy_fill_buffer();
    return block;
}

size_t amy_i2s_write(const uint8_t *buffer, size_t nbytes) {
  i2s_write_count++;

  // Ensure we always copy a safe number of bytes
  if (nbytes > sizeof(i2s_buffer_0)) {
    Serial.print("amy_i2s_write bytes exceeded: "); Serial.println(nbytes);
    nbytes = sizeof(i2s_buffer_0);
  }

  if (nbytes < sizeof(i2s_buffer_0)) {
    Serial.print("amy_i2s_write bytes truncated: "); Serial.println(nbytes);
  }

  // Check the buffer for any non-zero values
  bool zero_flag = true;
  for(size_t i=0;i<nbytes;i++) {
    if ( buffer[i] != 0 ) {
      zero_flag = false;
    }
  }
  if (zero_flag) zero_buffer_count++;

  if (render_buf0) {
    render_buf0 = false;
    memcpy(i2s_buffer_0, buffer, nbytes);
    buf0_ready = true;
    i2s_write_bytes += nbytes;
    return nbytes;
  }

  if (render_buf1) {
    render_buf1 = false;
    memcpy(i2s_buffer_1, buffer, nbytes);
    buf1_ready = true;
    i2s_write_bytes += nbytes;
    return nbytes;
  }

  Serial.print("amy_i2s_write bytes ignored: "); Serial.println(nbytes);
  return 0;
}

void loopAmy(uint16_t iteration = 0)
{
  // Wait indefinitely for a poke to render AMY audio
  uint32_t val = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  if (val > 1 || g_debug_task) {
    Serial.print(F("T - Amy Render: "));
    Serial.println(iteration);
    Serial.print("Render Notifications: ");
    Serial.println(val);
    Serial.flush();
  }
  amy_task_count++;
  // Only run amy_update, if we need to fill a buffer
  if ( render_buf0 || render_buf1 ) {
    amy_update_count++;
    uint32_t update_start_us = micros();
    //show_debug(99);
    amy_update();
    uint32_t update_duration_us = micros() - update_start_us;
    // Calculate the exponential moving average
    update_duration_ema = (update_duration_us >> 3) + (update_duration_ema-(update_duration_ema >> 3));
    if ( update_duration_us > update_duration_max ) {
      update_duration_max = update_duration_us;
    }
  }
}

void taskAmy(void *NotUsed)
{
  uint16_t loop_count = 0;
  while (1) {
    loopAmy(loop_count);
    loop_count++;
    yield();
  }
  // End ourselves
  if (g_debug_task) {
    Serial.println(F("T - Amy Render - Exiting"));
  }
  g_pAmyTaskHandle = NULL;
  vTaskDelete(NULL);
}

void display_info()
{
  Serial.print("Board ID       : 0x");
  Serial.print(NRF_FICR->DEVICEADDR[1], HEX);
  Serial.print(NRF_FICR->DEVICEADDR[0], HEX);
  Serial.print(NRF_FICR->DEVICEID[1], HEX);
  Serial.print(NRF_FICR->DEVICEID[0], HEX);
  Serial.println();
}

void example_synth_chord(uint32_t start, uint16_t patch_number, uint8_t polyphony) {
  // Like example_voice_chord, but use 'synth' to avoid having to keep track of voices.
  amy_event e = amy_default_event();
  e.time = start;
  e.patch_number = patch_number;
  e.num_voices = polyphony;
  e.synth = 0;
  amy_add_event(&e);
  start += 250;

  e = amy_default_event();
  e.velocity = 0.125;
  e.synth = 0;
  e.time = start;

  uint8_t chord_notes[] = { 50, 54, 57 }; // Dmaj
  for (int i = 0; i < polyphony; i++) {
    uint8_t octive = i/sizeof(chord_notes);
    uint8_t position = i%sizeof(chord_notes);

    e.time += (i == 0) ? 0 : 500;
    e.midi_note = chord_notes[position] + 12 * octive;
    amy_add_event(&e);
  }

  e.time += 2000;
  e.velocity = 0;

  // Voices are referenced only by their note, so have to turn them off individually.
  for (int i = 0; i < polyphony; i++) {
    uint8_t octive = i/sizeof(chord_notes);
    uint8_t position = i%sizeof(chord_notes);
    e.midi_note = chord_notes[position] + 12 * octive;
    amy_add_event(&e);
  }
}

void setup() {
  Bluefruit.begin();

  // Setup LEDs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !LED_STATE_ON);

  // Setup serial and delay for 10 secs
  Serial.begin(115200);
  delay(10000);

  // Display Board ID:
  display_info();
  delay(10000);

  // Crate AMY Render Task
  if ( g_pAmyTaskHandle == NULL) {
    g_pAmyTaskHandle = xTaskCreateStatic(taskAmy,
                                         "Amy Render",
                                         TASK_STACK_SIZE_DFLT,
                                         NULL,
                                         TASK_PRIO_LOW,
                                         g_amyTaskStackMem,
                                         &g_amyTaskTCB);
  }

  // Initialize loop iteration count
  iteration_count = 0;
}

void loop() {
  Serial.println(F("T - Loop"));
  iteration_count++;
  digitalWrite(LED_BUILTIN, LED_STATE_ON);
  delay(25);
  digitalWrite(LED_BUILTIN, !LED_STATE_ON);
  dbgMemInfo();
  Serial.print("Iteration count : "); Serial.println(iteration_count);
  delay(1000);

  // Reset state
  render_buf0 = false;
  render_buf1 = false;
  buf0_ready = false;
  buf1_ready = false;
  buffer_0_active = true;
  underrun_count = 0;
  i2s_write_count = 0;
  i2s_write_bytes = 0;
  amy_task_count = 0;
  amy_update_count = 0;
  zero_buffer_count = 0;
  update_duration_ema = 500;
  update_duration_max = 0;
  playback_active = false;

  // Checkpoint heap free
  uint32_t heap_used_before = dbgHeapUsed();

  // Init AMY
  amy_config_t amy_config = amy_default_config();
  amy_config.audio = AMY_AUDIO_IS_I2S;
  amy_start(amy_config);

  uint32_t event_setup_start = micros();

  example_synth_chord(amy_sysclock(),0,4);

  uint32_t event_setup_duration = micros() - event_setup_start;

  // Render for 4 seconds
  playback_start_ms = millis();
  playback_active = true;
  while (playback_active) {
    if (millis() - playback_start_ms >= 4000) {
      playback_active = false;
      uint32_t heap_used = dbgHeapUsed();
      Serial.print("Iteration count : "); Serial.println(iteration_count);
      Serial.print("Underrun count  : "); Serial.println(underrun_count);
      Serial.print("I2S write count : "); Serial.println(i2s_write_count);
      Serial.print("I2S write bytes : "); Serial.println(i2s_write_bytes);
      Serial.print("I2S write zeros : "); Serial.println(zero_buffer_count);
      Serial.print("Amy task count  : "); Serial.println(amy_task_count);
      Serial.print("Amy update cnt  : "); Serial.println(amy_update_count);
      Serial.print("Amy update ema  : "); Serial.println(update_duration_ema);
      Serial.print("Amy update max  : "); Serial.println(update_duration_max);
      Serial.print("Event setup     : "); Serial.println(event_setup_duration);
      Serial.print("Amy sysclock    : "); Serial.println(amy_sysclock());
      Serial.print("Amy sample rate : "); Serial.println(AMY_SAMPLE_RATE);
      Serial.print("Heap used       : "); Serial.println(dbgHeapUsed() - heap_used_before);
    }
    yield();
  }

  // Stop audio
  amy_stop();

  // Check heap
  uint32_t heap_used_after = dbgHeapUsed();
  uint32_t heap_free = ((uint32_t) __HeapLimit) - ((uint32_t) __HeapBase) - heap_used_after;
  Serial.print("Heap leaked     : "); Serial.println((int32_t)heap_used_after - (int32_t)heap_used_before);
  Serial.print("Heap free       : "); Serial.println(heap_free);

  // Optional underrun inspection
  if (underrun_count > 0) {
    Serial.print("WARNING Underrun: "); Serial.println(underrun_count);
  }

  // Idle 20s
  Serial.flush();
  delay(5000);
}
