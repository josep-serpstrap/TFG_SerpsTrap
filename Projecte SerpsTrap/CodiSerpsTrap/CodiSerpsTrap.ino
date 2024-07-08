/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preconfigured (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate a DevAddr, NwkSKey and
   AppSKey. Each device should have their own unique values for these
   fields.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/


#include <basicmac.h>
#include <hal/hal.h>
#include <SPI.h>

volatile bool DEBUG = true;

/***********************************************************************************************************************************************/
//TRANSCEIVER CONFIGURATION//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************************************************************************/

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x1F, 0xDF, 0x1C, 0x91, 0xA1, 0x10, 0xDE, 0x46, 0xB1, 0xAF, 0x28, 0xF9, 0x95, 0xDA, 0x6C, 0x9F };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xA5, 0xC4, 0xC0, 0x87, 0x4B, 0x9F, 0x52, 0x52, 0x87, 0xEE, 0xBE, 0xAD, 0xE1, 0x61, 0x75, 0xFA };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR =  0x260B8D20 ; // <-- Change this address for every node! For example, our device address is 26022DEB. We will need to replace "DEVICE_ADDRESS_HERE" as 0x26022DEB.

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getJoinEui (u1_t* /* buf */) { }
void os_getDevEui (u1_t* /* buf */) { }
void os_getNwkKey (u1_t* /* buf */) { }

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) {
  return LMIC_regionCode(0);
}

//// Schedule TX every this many milliseconds (might become longer due to duty
//// cycle limitations).
//const unsigned TX_INTERVAL = 20000;

// When this is defined, a standard pinmap from standard-pinmaps.ino
// will be used.  If you need to use a custom pinmap, comment this line
// and enter the pin numbers in the lmic_pins variable below.
//#define USE_STANDARD_PINMAP

//#if !defined(USE_STANDARD_PINMAP)
// All pin assignments use Arduino pin numbers (e.g. what you would pass
// to digitalWrite), or LMIC_UNUSED_PIN when a pin is not connected.
const lmic_pinmap lmic_pins = {
  // NSS input pin for SPI communication (required)
  .nss = D36,
  // If needed, these pins control the RX/TX antenna switch (active
  // high outputs). When you have both, the antenna switch can
  // powerdown when unused. If you just have a RXTX pin it should
  // usually be assigned to .tx, reverting to RX mode when idle).
  //
  // The SX127x has an RXTX pin that can automatically control the
  // antenna switch (if internally connected on the transceiver
  // board). This pin is always active, so no configuration is needed
  // for that here.
  // On SX126x, the DIO2 can be used for the same thing, but this is
  // disabled by default. To enable this, set .tx to
  // LMIC_CONTROLLED_BY_DIO2 below (this seems to be common and
  // enabling it when not needed is probably harmless, unless DIO2 is
  // connected to GND or VCC directly inside the transceiver board).
  .tx = LMIC_UNUSED_PIN,
  .rx = LMIC_UNUSED_PIN,
  // Radio reset output pin (active high for SX1276, active low for
  // others). When omitted, reset is skipped which might cause problems.
  .rst = D44,
  // DIO input pins.
  //   For SX127x, LoRa needs DIO0 and DIO1, FSK needs DIO0, DIO1 and DIO2
  //   For SX126x, Only DIO1 is needed (so leave DIO0 and DIO2 as LMIC_UNUSED_PIN)
  //.dio = {/* DIO0 */ 2, /* DIO1 */ 3, /* DIO2 */ 4},

  .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ D40, /* DIO2 */ LMIC_UNUSED_PIN},
  // Busy input pin (SX126x only). When omitted, a delay is used which might
  // cause problems.
  .busy = D39,
  // TCXO oscillator enable output pin (active high).
  //
  // For SX127x this should be an I/O pin that controls the TCXO, or
  // LMIC_UNUSED_PIN when a crystal is used instead of a TCXO.
  //
  // For SX126x this should be LMIC_CONTROLLED_BY_DIO3 when a TCXO is
  // directly connected to the transceiver DIO3 to let the transceiver
  // start and stop the TCXO, or LMIC_UNUSED_PIN when a crystal is
  // used instead of a TCXO. Controlling the TCXO from the MCU is not
  // supported.
  .tcxo = LMIC_UNUSED_PIN,
};
//#endif // !defined(USE_STANDARD_PINMAP)

void onLmicEvent (ev_t ev) {
  if(DEBUG){
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
      case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
      case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
      case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
      case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
      case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
      case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        break;
      case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
      case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
      case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
      case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
          Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
          Serial.print(F("Received "));
          Serial.print(LMIC.dataLen);
          Serial.println(F(" bytes of payload"));
        }
        break;
      case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
      case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
      case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
      case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
      case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
      case EV_SCAN_FOUND:
        Serial.println(F("EV_SCAN_FOUND"));
        break;
      case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
      case EV_TXDONE:
        Serial.println(F("EV_TXDONE"));
        break;
      case EV_DATARATE:
        Serial.println(F("EV_DATARATE"));
        break;
      case EV_START_SCAN:
        Serial.println(F("EV_START_SCAN"));
        break;
      case EV_ADR_BACKOFF:
        Serial.println(F("EV_ADR_BACKOFF"));
        break;
      default:
        Serial.print(F("Unknown event: "));
        Serial.println(ev);
        break;
    }
  }
}

#ifdef __cplusplus
  extern "C" {
#endif
// C++ compilations must declare the ISR as a "C" routine or else its name will get mangled
// and the linker will not use this routine to replace the default ISR
void my_isr();

#ifdef __cplusplus
  }
#endif

/***********************************************************************************************************************************************/
//GLOBAL VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************************************************************************/

const uint8_t SNAKE_FSR_PIN = A0;
const uint8_t SNAKE_MAG_PIN = 12;
const uint8_t RAT_PIR_PIN = 13;
const uint8_t RAT_FSR1_PIN = A2;
const uint8_t RAT_FSR2_PIN = A4;

const uint8_t EN_FSR_PIN = 9;
const uint8_t EN_MAG_PIN = 8;
const uint8_t EN_FSR1_PIN = 6;
const uint8_t EN_FSR2_PIN = 7;
const uint8_t EN_PIR_PIN = 5;

uint16_t minutes = 0;

uint16_t fsr_snake = 0;

uint8_t magnetic = 1;

uint8_t escala1_canvis = 0;
uint16_t fsr_escala1 = 0;
uint16_t fsr_escala1_aux = 0;
uint8_t llindar_escala1 = 15;

uint8_t escala2_canvis = 0;
uint16_t fsr_escala2 = 0;
uint16_t fsr_escala2_aux = 0;
uint8_t llindar_escala2 = 60;

uint8_t infrared = 0;

unsigned long int_time = 0;

//Interval for some actions in minutes
uint8_t int_pir = 15;
uint8_t int_lora = 180;

/***********************************************************************************************************************************************/
//ISR TIMER//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************************************************************************/

volatile bool timer_flag = false;

extern "C" void timer()
{
  timer_flag = true;
  minutes++;
  if(minutes > int_lora){
    minutes = 1;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only.
  }
  pinMode(SNAKE_FSR_PIN, INPUT);
  pinMode(SNAKE_MAG_PIN, INPUT);
  pinMode(RAT_PIR_PIN, INPUT);
  pinMode(RAT_FSR1_PIN, INPUT);
  pinMode(RAT_FSR2_PIN, INPUT);
  
  pinMode(EN_FSR_PIN, OUTPUT);
  pinMode(EN_MAG_PIN, OUTPUT);
  pinMode(EN_FSR1_PIN, OUTPUT);
  pinMode(EN_FSR2_PIN, OUTPUT);
  pinMode(EN_PIR_PIN, OUTPUT);

  digitalWrite(EN_PIR_PIN, LOW);
  digitalWrite(EN_MAG_PIN, LOW);
  digitalWrite(EN_FSR_PIN, LOW);
  digitalWrite(EN_FSR1_PIN, LOW);
  digitalWrite(EN_FSR2_PIN, LOW);
  

  if(DEBUG){
    Serial.println();
    Serial.println();
    Serial.println(F("Starting"));
    Serial.println();
  }

  // LMIC init
  os_init(nullptr);
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // These are defined by the LoRaWAN specification
  enum {
    EU_DR_SF12 = 0,
    EU_DR_SF11 = 1,
    EU_DR_SF10 = 2,
    EU_DR_SF9 = 3,
    EU_DR_SF8 = 4,
    EU_DR_SF7 = 5,
    EU_DR_SF7_BW250 = 6,
    EU_DR_FSK = 7,
  };

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7_BW250)); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(EU_DR_FSK,  EU_DR_FSK));      // g2-band

  // TTN uses SF9 at 869.525Mhz for its RX2 window (frequency is
  // default LoRaWAN, SF is different, but set them both to be
  // explicit).
  LMIC.dn2Freq = 869525000;
  LMIC.dn2Dr = EU_DR_SF9;

  // Set data rate for uplink
  LMIC_setDrTxpow(EU_DR_SF7, KEEP_TXPOWADJ);
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  // TODO: How to configure these channels? LMIC had LMIC_selectSubBand,
  // but it seems BasicMac only has LMIC_disableChannel.

  // Attempt to disable unused channels except channel 8 (903.900024)
  // Note, this is the only channel that the SparkFun Single-channel Gateway
  // listens on, using the provided ESP32 gateway and example code from this tutorial:
  // https://learn.sparkfun.com/tutorials/sparkfun-lora-gateway-1-channel-hookup-guide/single-channel-lorawan-gateway
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

/***********************************************************************************************************************************************/
//TIMER CONFIGURATION////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************************************************************************************************************************/

  am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
                              AM_HAL_CTIMER_LFRC_1HZ |
                              AM_HAL_CTIMER_FN_REPEAT |
                              AM_HAL_CTIMER_INT_ENABLE);

  am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 60, 1);

  am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

  NVIC_EnableIRQ(CTIMER_IRQn);

  am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
  am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);  
  am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERA0, timer);
  
}


void loop() {
  
  os_runstep();

  if(timer_flag){
    
    if(minutes % int_pir == 0){
      digitalWrite(EN_PIR_PIN, HIGH);
    }
    int_time = millis();

    digitalWrite(EN_FSR_PIN, HIGH);
    digitalWrite(EN_MAG_PIN, HIGH);
    digitalWrite(EN_FSR1_PIN, HIGH);
    digitalWrite(EN_FSR2_PIN, HIGH);
    
    fsr_snake = 1023 - analogRead(A0);
    if(DEBUG) Serial.println(fsr_snake);
    fsr_escala1 = 1023 - analogRead(A2);
    if(DEBUG) Serial.println(fsr_escala1);
    fsr_escala2 = 1023 - analogRead(A4); 
    if(DEBUG) Serial.println(fsr_escala2);   

    if(((fsr_escala1 - fsr_escala1_aux) > llindar_escala1) || (fsr_escala1_aux - fsr_escala1 > llindar_escala1)){
      escala1_canvis++;
    }
    if(((fsr_escala2 - fsr_escala2_aux) > llindar_escala2) || (fsr_escala2_aux - fsr_escala2 > llindar_escala2)){
      escala2_canvis++;
    }
    fsr_escala1_aux = fsr_escala1;
    fsr_escala2_aux = fsr_escala2;

    magnetic = digitalRead(SNAKE_MAG_PIN);
    if(DEBUG) Serial.println(magnetic);   

    digitalWrite(EN_FSR_PIN, LOW);
    digitalWrite(EN_MAG_PIN, LOW);
    digitalWrite(EN_FSR1_PIN, LOW);
    digitalWrite(EN_FSR2_PIN, LOW);

    if(minutes % int_pir == 0){
      while((millis() - int_time) < 5000){}
      if(digitalRead(RAT_PIR_PIN)){
        infrared++;
      }
      if(DEBUG) Serial.println(infrared);   
      digitalWrite(EN_PIR_PIN, LOW);
    }
    
    timer_flag = false;
    if(minutes % int_lora == 0){
      send_packet();
      int env = millis();
      while(millis() - env < 3000){os_runstep();}
    }
    
  }
  
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
 
}

void send_packet() {
  // Prepare upstream data transmission at the next possible time.
  uint8_t mydata[6] = {0, 0, 0, 0, 0, 0};
  mydata[1] |= fsr_snake;
  mydata[0] |= (fsr_snake >> 8);
  mydata[2] = magnetic;

  mydata[3] |= escala1_canvis;
  mydata[4] |= escala2_canvis;
  mydata[5] = infrared;  
  
  LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
  if(DEBUG) Serial.println(F("Packet queued"));
}
