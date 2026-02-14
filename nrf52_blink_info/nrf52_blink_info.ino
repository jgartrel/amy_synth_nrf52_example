#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <bluefruit.h>

float voltageLevel(uint16_t level) {
  float voltage = level * 3.6 / 4096.0;
  return voltage;
}

uint32_t analogRead_internal2( uint32_t psel)
{
  volatile int16_t value = 0;

  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }
  NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass        << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                            | ((SAADC_CH_CONFIG_RESP_Bypass      << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                            | ((SAADC_CH_CONFIG_GAIN_Gain1_6     << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                            | ((SAADC_CH_CONFIG_REFSEL_Internal  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                            | ((SAADC_CH_CONFIG_TACQ_3us         << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                            | ((SAADC_CH_CONFIG_MODE_SE          << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
                            | ((SAADC_CH_CONFIG_BURST_Disabled   << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);
  NRF_SAADC->CH[0].PSELN = psel;
  NRF_SAADC->CH[0].PSELP = psel;

  NRF_SAADC->RESULT.PTR = (uint32_t)&value;
  NRF_SAADC->RESULT.MAXCNT = 1; // One sample

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

  NRF_SAADC->TASKS_START = 0x01UL;

  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;

  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0x00UL;

  NRF_SAADC->TASKS_STOP = 0x01UL;

  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0x00UL;

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);

  // Disable channel and ensure config is set to disable ladder resistors
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELP_PSELP_NC;
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_NC;
  NRF_SAADC->CH[0].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass        << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                            | ((SAADC_CH_CONFIG_RESP_Bypass      << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                            | ((SAADC_CH_CONFIG_GAIN_Gain1_6     << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                            | ((SAADC_CH_CONFIG_REFSEL_Internal  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                            | ((SAADC_CH_CONFIG_TACQ_10us        << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                            | ((SAADC_CH_CONFIG_MODE_SE          << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
                            | ((SAADC_CH_CONFIG_BURST_Disabled   << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

  if (value < 0) {
    value = 0;
  }

  return value;
}

void setup() {
  Bluefruit.begin();          // Sleep functions need the softdevice to be active.

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !LED_STATE_ON);    

  Serial.begin(115200);
  delay(10);
}

void loop() {
  //digitalToggle(LED_BUILTIN); // turn the LED on (HIGH is the voltage level)
  ledOn(LED_BUILTIN);
  dbgPrintVersion();
  dbgMemInfo();
  Serial.printf("NRF_FICR->INFO.PART      : 0x%08X\n", NRF_FICR->INFO.PART);
  Serial.printf("NRF_FICR->INFO.VARIANT   : 0x%08X\n", NRF_FICR->INFO.VARIANT);
  Serial.printf("NRF_FICR->INFO.PACKAGE   : 0x%08X\n", NRF_FICR->INFO.PACKAGE);
  Serial.printf("NRF_FICR->INFO.RAM       : 0x%08X\n", NRF_FICR->INFO.RAM);
  Serial.printf("NRF_FICR->INFO.FLASH     : 0x%08X\n", NRF_FICR->INFO.FLASH);
  Serial.printf("NRF_POWER->RESETREAS     : 0x%08X\n", NRF_POWER->RESETREAS);
  Serial.printf("NRF_POWER->MAINREGSTATUS : 0x%08X\n", NRF_POWER->MAINREGSTATUS);
  Serial.printf("NRF_POWER->USBREGSTATUS  : 0x%08X\n", NRF_POWER->USBREGSTATUS);
  Serial.printf("NRF_UICR->REGOUT0        : 0x%08X\n", NRF_UICR->REGOUT0);
  Serial.printf("NRF_USBD->ENABLE         : 0x%08X\n", NRF_USBD->ENABLE);
  Serial.printf("NRF_USBD->USBPULLUP      : 0x%08X\n", NRF_USBD->USBPULLUP);
  Serial.printf("NRF_USBD->DPDMVALUE      : 0x%08X\n", NRF_USBD->DPDMVALUE);
  Serial.printf("VDD  : %d\n", analogRead_internal2(SAADC_CH_PSELP_PSELP_VDD));
  Serial.printf("VDD  : %0.2fV\n", voltageLevel(analogRead_internal2(SAADC_CH_PSELP_PSELP_VDD)));
  Serial.printf("VDDH : %d\n", analogRead_internal2(SAADC_CH_PSELP_PSELP_VDDHDIV5));
  Serial.printf("VDDH : %0.2fV\n", 5 * voltageLevel(analogRead_internal2(SAADC_CH_PSELP_PSELP_VDDHDIV5)));
  delay(1000);
  ledOff(LED_BUILTIN);
  delay(10000);                // wait for a second
}
