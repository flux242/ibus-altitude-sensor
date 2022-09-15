
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#ifdef SENSOR_BMP085
#include "bmp085/bmp085.h"
#endif

#ifdef SENSOR_BMP280
#include "bmp280/bmp280.h"
#endif

#include "ibus/ibus.h"

#define BATTERY_VOLTAGE_INTEGRATOR_FACTOR 1  // will be rshifted this amount
#define ALTITUDE_INTEGRATOR_FACTOR 32        // will be divided this amount

// this multiplier is for 10k/1k voltage divider and __AVR_ATmega32U4__ target (2.56V ref voltage)
#ifdef __AVR_ATmega32U4__
#define BATTERY_VOLTAGE_MULTIPLIER 11u   // (R1+R2)/R2 = (10k+1k)/1k = 11
                                         // BatVoltage(mV) = 11*100*2.56*ADCraw/1024 = 11*(ADCraw>>2)
#endif

// Due to the fact that voltage divider resistors aren't 
// exactly 10k and 1k there is a gain error. At 12V battery 
// voltage the error in my case was half a volt already so
// compensation is necessary. I'm not doing floating point
// multiplication but use a trick described in the 
// https://ww1.microchip.com/downloads/en/Appnotes/Atmel-2559-Characterization-and-Calibration-of-the-ADC-on-an-AVR_ApplicationNote_AVR120.pdf
// At first ADC_COMPENSATION is udefined and ADC_CALIBRATION is defined:
// 1. 0V input voltage is measured to get the ADC_OFFSET.
//    Multiply value that is shown on the transmitter with 100 and
//    substract 100. That's the ADC_OFFSET.
// 2. Measure battery voltage. Multiply the value shown by the transmitter
//    with 100 and divide it with the battery voltage measured with multimiter.
//    That's the ADC_GAIN
#undef ADC_CALIBRATION
#define ADC_COMPENSATION

#ifdef ADC_COMPENSATION
#define ADC_OFFSET 1           // raw adc offset value
#define ADC_GAIN 0.96309f
//static const uint32_t ADC_GAIN_FACTOR = (uint32_t)(ADC_GAIN*(float)(2<<14));
//static const uint32_t ADC_CORRECTION = (uint32_t)(ADC_GAIN*(float)ADC_OFFSET*(2<<14))+(uint32_t)(0.5f*(float)(2<<14));
// compiler seems to be 2 stupid to calculate consts at compile time
static const uint32_t ADC_GAIN_FACTOR = 31558;
static const uint32_t ADC_CORRECTION =  47942; //31558+16384;
#endif


#define PACKET_BUF_SIZE 16  // the very same buffer is used for rx and tx because we have half-duplex uart

static uint8_t packet_buffer[PACKET_BUF_SIZE];

typedef struct {
  uint8_t id;
  union {
    uint16_t value16;
    uint32_t value32;
  };
} ibus_sensor_t;

ibus_sensor_t sensors[] = {
  { 0u, {0u} },  // index 0 is not used
  { IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE, {0u} },
  { IBUS_SENSOR_TYPE_TEMPERATURE, {0u} },
  { IBUS_SENSOR_TYPE_CLIMB_RATE, {0u} },
  { IBUS_SENSOR_TYPE_ALT, {0} },
  { IBUS_SENSOR_TYPE_ALT_MAX, {0} }
};

#define EXTERNAL_SENSOR_VOLTAGE_INDEX 1u
#define EXTERNAL_SENSOR_TEMPERATURE_INDEX 2u
#define EXTERNAL_SENSOR_CLIMB_RATE_INDEX 3u
#define EXTERNAL_SENSOR_ALTITUDE 4u
#define EXTERNAL_SENSOR_ALTITUDE_MAX 5u

const size_t NumberOfSensors = sizeof(sensors)/sizeof(sensors[0]);


// function prototypes
static void handle_rx_packet(uint8_t);


/* ------------ on-board LEDs ------------- */
#ifdef __AVR_ATmega328P__
static inline void led_init(void)
{
  DDRB |= _BV(PB5); // Rx LED
}

static inline void led1_on(void)
{
  PORTB |= _BV(PB5);
}

static inline void led1_off(void)
{
  PORTB &= ~_BV(PB5);
}
static inline void led1_invert(void)
{
  if (PORTB & _BV(PB5)) {
    led1_on();
  }
  else {
    led1_off();
  }
}
static inline uint8_t is_led1_on()
{
  return ((PORTB & _BV(PB5))>0);
}
static inline void led2_on(void) { }
static inline void led2_off(void) { }
static inline void led2_invert(void) {}

#endif

#ifdef __AVR_ATmega32U4__
static inline void led_init(void)
{
  DDRB |= _BV(PB0); // Rx LED
  DDRD |= _BV(PD5); // Tx LED
}

static inline void led1_on(void)
{
  PORTB &= ~_BV(PB0);
}

static inline void led1_off(void)
{
  PORTB |= _BV(PB0);
}
static inline void led1_invert(void)
{
  if (PORTB & _BV(PB0)) {
    led1_on();
  }
  else {
    led1_off();
  }
}
static inline uint8_t is_led1_on(void)
{
  return ((PORTB & _BV(PB0))==0);
}

static inline void led2_on(void)
{
  PORTD &= ~_BV(PD5);
}

static inline void led2_off(void)
{
  PORTD |= _BV(PD5);
}
static inline void led2_invert(void)
{
  if (PORTD & _BV(PD5)) {
    led2_on();
  }
  else {
    led2_off();
  }
}
#endif

/* ----------------- USART ----------------- */

// I-Bus uses 115200n8
#define UART_BAUD       115200
#define UBRR_VAL        ((F_CPU + 8UL * UART_BAUD) / (16UL*UART_BAUD) - 1)

#ifdef __AVR_ATmega328P__
static void serial_init(void)
{
  UBRR0 = UBRR_VAL;

  UCSR0A = 0;
  UCSR0B = _BV(RXEN0) | _BV(RXCIE0) | _BV(UDRIE0);
  UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);
}

static void serial_enable_rx(void)
{
  UCSR0B &= ~(_BV(TXEN0) | _BV(TXCIE0));
  UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);
}

static void serial_enable_tx(void)
{
  UCSR0B &= ~_BV(RXEN0);
  UCSR0B |= _BV(TXEN0) | _BV(UDRIE0);
}

static void serial_notify_tx_end(void)
{
  UCSR0B &= ~_BV(UDRIE0);
  UCSR0B |= _BV(TXCIE0);
}

#define serial_rx_vect USART_RX_vect
#define serial_tx_vect USART_TX_vect
#define serial_udre_vect USART_UDRE_vect

#define serial_data UDR0

#endif

#ifdef __AVR_ATmega32U4__
static void serial_init(void)
{
  UBRR1 = UBRR_VAL;

  UCSR1A = 0;
  UCSR1B = _BV(RXEN1) | _BV(RXCIE1) | _BV(UDRIE1);
  UCSR1C = _BV(UCSZ11)|_BV(UCSZ10);
}

static void serial_enable_rx(void)
{
  UCSR1B &= ~(_BV(TXEN1) | _BV(TXCIE1));
  UCSR1B |= _BV(RXEN1) | _BV(RXCIE1);
}

static void serial_enable_tx(void)
{
  UCSR1B &= ~_BV(RXEN1);
  UCSR1B |= _BV(TXEN1) | _BV(UDRIE1);
}

static void serial_notify_tx_end(void)
{
  UCSR1B &= ~_BV(UDRIE1);
  UCSR1B |= _BV(TXCIE1);
}

#define serial_rx_vect USART1_RX_vect
#define serial_tx_vect USART1_TX_vect
#define serial_udre_vect USART1_UDRE_vect

#define serial_data UDR1

#endif

static void recv_restart(void)
{
  serial_enable_rx();
}

static volatile uint8_t transmitBufferIndex = 0u;
static void tx_start(void)
{
  serial_enable_tx();
}

// USART receive interrupt
ISR(serial_rx_vect)
{
  uint8_t val = serial_data;
  handle_rx_packet(val);
}

// Next Tx byte wanted
ISR(serial_udre_vect)
{
  if (transmitBufferIndex < packet_buffer[0])
  {
    serial_data = packet_buffer[transmitBufferIndex++];
  }
  else
  {
    serial_notify_tx_end();
  }
}
// Tx finished
ISR(serial_tx_vect)
{
  transmitBufferIndex = 0u;
  recv_restart();
}


#define TIMER1MS 63536
// 1e-3*16000000/8 = 2000; 65536 - 2000 = 63536
volatile uint16_t system_counter_ms = 0;
// 1 ms timer interrupt
ISR (TIMER1_OVF_vect)    // Timer1 ISR
{
  TCNT1 = TIMER1MS;      // for 1ms at 16 MHz
  ++system_counter_ms;
}

static void system_timer_init(void)
{
  TCNT1 = TIMER1MS;        // for 1ms at 16 MHz

  TCCR1A = 0x00;
  TCCR1B = (1<<CS11);      // Timer mode with 8 prescler
  TIMSK1 = (1 << TOIE1) ;  // Enable timer1 overflow interrupt(TOIE1)
}

/* ---- A/D converter for battery voltage ---- */
#ifdef __AVR_ATmega328P__
static void adc_init(void)
{
  ADCSRA = _BV(ADEN) |  // enable ADC
           _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // CLK/128 = 125 kHz
  ADMUX =  _BV(REFS1) | _BV(REFS0) | // internal 1.1V reference
           _BV(MUX2) | _BV(MUX1) | _BV(MUX0); // ADC7
}
#endif

#ifdef __AVR_ATmega32U4__
static void adc_init(void)
{
  DIDR0 |= _BV(ADC4D); // disable digital input on ADC4
  ADCSRA = _BV(ADEN) |  // enable ADC
           _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // CLK/128 = 125 kHz
  ADMUX =  _BV(REFS1) | _BV(REFS0) | // internal 2.56V reference
           _BV(MUX2); // ADC4 on pin PF4
}
#endif


inline static void adc_start_conversion(void)
{
  ADCSRA |= _BV(ADSC); // start the conversion
}

inline static size_t adc_is_conversion_running(void)
{
  return (ADCSRA & _BV(ADSC)) != 0u;
}

inline static size_t adc_is_conversion_ready(void)
{
  return (ADCSRA & _BV(ADIF)) != 0;
}

inline static uint16_t  adc_get_conversion_result(void)
{
  uint16_t retval;

  retval = ADCW;
  ADCSRA |= _BV(ADIF); // clear the interrupt flag

  return retval;
}

inline static uint16_t read_adc_sync(void)
{
  if (!adc_is_conversion_running()) {
    adc_start_conversion();
  }
  // wait for the result
  while (!adc_is_conversion_ready()) {}
  return adc_get_conversion_result();
}

/****************
 * @brief Returns battery voltage in mV
 */
static uint16_t getBatteryVoltage(void)
{
  uint16_t retValue = 0u;
  if (!adc_is_conversion_ready())
  {
    if (!adc_is_conversion_running())
    {
      adc_start_conversion();
    }
    while(!adc_is_conversion_ready()){}
  }
  retValue = adc_get_conversion_result();
  adc_start_conversion(); // starting next converson even now
#ifdef ADC_COMPENSATION
  uint32_t retValueCompensated = (uint32_t)retValue * ADC_GAIN_FACTOR - ADC_CORRECTION;
  retValue = (uint16_t)(retValueCompensated >> 15);
#elif defined ADC_CALIBRATION
  retValue += 100u;
#endif
  return  (retValue * BATTERY_VOLTAGE_MULTIPLIER) >> 2; 
}

uint16_t integrateBatteryVoltage(uint16_t newVoltage)
{
  static uint16_t integratedVoltage = 0u;
  if (newVoltage>integratedVoltage)
  {
    integratedVoltage += ((newVoltage-integratedVoltage ) >> BATTERY_VOLTAGE_INTEGRATOR_FACTOR); // cumulative moving average
  }
  else
  {
    integratedVoltage -= ((integratedVoltage-newVoltage) >> BATTERY_VOLTAGE_INTEGRATOR_FACTOR); // cumulative moving average
  }
  return integratedVoltage;
}

int32_t integrateAltitude(int32_t newAltitude)
{
  static uint8_t isInitialized = 0u;
  static int32_t integratedAltitude = 0u;
  if (isInitialized != 0u)
  {
    integratedAltitude += ((newAltitude-integratedAltitude )/ALTITUDE_INTEGRATOR_FACTOR); // cumulative moving average
  }
  else
  {
    integratedAltitude = newAltitude;
    isInitialized = 1u;
  }
  return integratedAltitude;
}
static inline size_t is_device_index_valid(uint8_t id)
{
  return ( (id > 0u) && (id < NumberOfSensors));
}

static uint16_t calc_packet_crc(uint8_t *packet_buffer)
{
  uint16_t packet_sum = 0xFFFFu;
  size_t packet_len = packet_buffer[0];

  for (size_t i = 0; i<packet_len-2; ++i) {
    packet_sum-= packet_buffer[i];
  }
 
  return packet_sum; 
}

static void transmit_packet(uint8_t *packet_buffer)
{
  uint16_t packet_sum = calc_packet_crc(packet_buffer);
  size_t packet_len = packet_buffer[0];
  packet_buffer[packet_len-2] = (uint8_t)packet_sum; // crc lower part 
  packet_buffer[packet_len-1] = (uint8_t)(packet_sum >> 8u); // crc upper part 
  tx_start();
}
static void handle_ibus_request(uint8_t *packet_buffer)
{
  uint16_t crc = calc_packet_crc(packet_buffer);

   size_t packet_len = packet_buffer[0];

  if ( (packet_buffer[packet_len-2] == (uint8_t)(crc & 0x00FF)) && (packet_buffer[packet_len-1] == (uint8_t)(crc >> 8u)) )
  { // crc is ok
    uint8_t device_index = packet_buffer[1] & 0x0F;
    switch(packet_buffer[1] & 0xF0)
    {
      case 0x80:
        if ( is_device_index_valid(device_index) ) {
          // send back the received request
          transmit_packet(packet_buffer);
        }
        break;
      case 0x90:
        if ( is_device_index_valid(device_index) ) {
          // send back the sensor type
          packet_buffer[0] = 0x06;
          // packet_buffer[1] is the same is the incomming byte
          packet_buffer[2] = sensors[device_index].id;
          packet_buffer[3] = (packet_buffer[2] < 0x80) ? 0x02 : 0x04;
          //packet_buffer[3] = 0x02;
          transmit_packet(packet_buffer);
        }
        break;
      case 0xA0:
led2_invert();
        if ( is_device_index_valid(device_index) ) {
          if (sensors[device_index].id < 0x80) {
            packet_buffer[0] = 0x06;
            // packet_buffer[1] is the same is the incomming byte
//            *((uint16_t*)(&packet_buffer[2])) = sensors[device_index].value16;
            packet_buffer[2] = (uint8_t)sensors[device_index].value16;
            packet_buffer[3] = (uint8_t)(sensors[device_index].value16 >> 8);
          }
          else {
            packet_buffer[0] = 0x08;
            // packet_buffer[1] is the same is the incomming byte
//            *((uint32_t*)(&packet_buffer[2])) = sensors[device_index].value32;
            packet_buffer[2] = (uint8_t)sensors[device_index].value32;
            packet_buffer[3] = (uint8_t)(sensors[device_index].value32 >> 8);
            packet_buffer[4] = (uint8_t)(sensors[device_index].value32 >> 16);
            packet_buffer[5] = (uint8_t)(sensors[device_index].value32 >> 24);
          }
          transmit_packet(packet_buffer);
        }
        break;
      default:
        break;
    } 
  }
}

typedef enum  {
  WAIT_COMMAND_START,
  WAIT_COMMAND_COMPLETE,
} uart_rx_state_e;
static uart_rx_state_e uart_rx_state = WAIT_COMMAND_START;

static void handle_rx_packet(uint8_t nextByte)
{
  static uint8_t packet_buffer_index = 0u;

    packet_buffer[packet_buffer_index++] = nextByte;
    if (packet_buffer_index>0)
    { // someting is in the buffer
      switch (uart_rx_state)
      {
        case WAIT_COMMAND_START:
          if (packet_buffer[0] != 4) {
            packet_buffer_index = 0u;
          }
          else {
            uart_rx_state =  WAIT_COMMAND_COMPLETE;
          }
          break;
        case WAIT_COMMAND_COMPLETE:
          if (packet_buffer_index > packet_buffer[0u])
          {
            handle_ibus_request(&packet_buffer[0]);
            packet_buffer_index = 0u;
            uart_rx_state = WAIT_COMMAND_START;
          }
          break;
        default:
          break;
      }
    }
}


static int32_t initial_altitude = 0u;
static int32_t old_altitude = 0u;
static uint16_t old_time = 0u;

int main(void)
{
#ifdef SENSOR_BMP085
  bmp085_init();
#endif
#ifdef SENSOR_BMP280
  bmp280_init();
  bmp280_set_config(0, 3, 0); // 0.5 ms delay, 8x filter, no 3-wire SPI
#endif
  adc_init();
  led_init();
  led1_off();
  led2_off();

  serial_init();
  recv_restart();
  system_timer_init();

  sei();

  while (1)
  {
    led1_invert();

    // measurements

    // baterry voltage
    uint16_t batVoltage = integrateBatteryVoltage(getBatteryVoltage());
    if (batVoltage < 100 ) { // when unconnected, don't send the noise
      batVoltage  = 0;
    }
    sensors[EXTERNAL_SENSOR_VOLTAGE_INDEX].value16 = batVoltage;

    // temperature
    sensors[EXTERNAL_SENSOR_TEMPERATURE_INDEX].value16 = (uint16_t)(bmp085_gettemperature()+400);

    // altitude and climb rate
    int32_t altitude = integrateAltitude((int32_t)(100.0f*bmp085_getaltitude()));
    int16_t current_time = system_counter_ms;
    uint16_t time_diff = 0u;
    if (current_time < old_time)
    { // counter overflow situation
      time_diff = (65535-old_time) + current_time;
    }
    else {
      time_diff  = current_time - old_time;
    }
    if (time_diff != 0) {
      int32_t altitude_diff = altitude - old_altitude;
      int32_t climb_rate = altitude_diff*1000/time_diff;
      sensors[EXTERNAL_SENSOR_CLIMB_RATE_INDEX].value16 = (uint16_t)climb_rate; 
    }
    old_time = current_time;
    old_altitude = altitude;

    sensors[EXTERNAL_SENSOR_ALTITUDE].value32 = (uint32_t)altitude; 
    if ( (initial_altitude == 0) && (system_counter_ms > 5000))
    { // set initial altitude after 5 seconds first. Let the sensors warm up a little
      initial_altitude = altitude;
    }
    if (initial_altitude != 0) {
      sensors[EXTERNAL_SENSOR_ALTITUDE_MAX].value32 = (uint32_t)((altitude-initial_altitude)/100);
    }
  }
}

