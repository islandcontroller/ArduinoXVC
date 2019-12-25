/*!***************************************************************************
 * @file
 * ArduinoXVC.ino
 * 
 * Arduino UNO/Leonardo  implementation  of  the  XVC  Server  documented  at
 * https://github.com/Xilinx/XilinxVirtualCable.
 * Requires an  ethernet-to-serial bridge  in order to work with  programming
 * tools,   e.g.   https://github.com/pyserial/pyserial/blob/master/examples/
 * tcp_serial_redirect.py.
 * 
 * @copyright
 * Copyright 2019 inselc
 * 
 * Licensed   under  the  Apache  License,   Version  2.0   (the  "License");
 * you may not use this file except in compliance with the  License.  You may
 * obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless  required by  applicable law  or  agreed  to  in  writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,  WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See  the
 * License for the specific language  governing permissions  and  limitations
 * under the License.
 *****************************************************************************/

/*- Symbolic Constants ------------------------------------------------------*/
/*! Open Drain output type                                                   */
#define OPENDRAIN       INPUT

/*! Message buffer size                                                      */
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_LEONARDO)
#define DATA_BUF_SIZE   632
#elif defined(ARDUINO_AVR_MEGA)
#define DATA_BUF_SIZE   1468
#else
#define DATA_BUF_SIZE   128
#endif
#define DATA_VEC_LEN    (DATA_BUF_SIZE * 8)

/*! Pin definitions for JTAG connection
 *  @{                                                                       */
#define TCK_PIN         2
#define TMS_PIN         3
#define TDI_PIN         4
#define TDO_PIN         5
/*! @}                                                                       */

/*! Status LED                                                               */
#define LED_PIN         13

/*! Macro for min/max limit                                                  */
#define limit(x, l, h)  ((x < l) ? l : ((x > h) ? h : x))


/*- Type definitions --------------------------------------------------------*/
/*!***************************************************************************
 * @brief
 * State coding for JTAG interface handler FSM
 *****************************************************************************/
typedef enum {
  /*! Start transmission                                                     */
  EN_JTAG_STATE_START,

  /*! Generate TCK rising edge                                               */
  EN_JTAG_STATE_RISINGEDGE,

  /*! Generate TCK falling edge                                              */
  EN_JTAG_STATE_FALLINGEDGE,

  /*! Transfer complete                                                      */
  EN_JTAG_STATE_DONE
} teJtagState;


/*- Variables accessed by interrupts ----------------------------------------*/
/*! Timer 1 counter reload value                                             */
volatile uint16_t uiTimer1Reload;

/*! Timer 2 counter reload value                                             */
volatile uint8_t ucTimer2Reload;

/*! TMS data vector                                                          */
volatile uint8_t aucDataTMS[DATA_BUF_SIZE];

/*! TDI/TDO data vector                                                      */
volatile uint8_t aucDataTDx[DATA_BUF_SIZE];

/*! JTAG vector length (bits)                                                */
volatile uint16_t uiDataLen;

/*! Bit index in both JTAG data vectors                                      */
volatile uint16_t uiDataIndex;

/*! Interface handler state variable                                         */
volatile teJtagState eState;


/*- Local Functions ---------------------------------------------------------*/
/*!***************************************************************************
 * @brief
 * Emulate Open Drain output
 * 
 * @param[in] pin   Same as pin for digitalWrite
 * @param[in] value "HIGH" will turn pin High-Z, "LOW" will pull pin low
 *****************************************************************************/
static void digitalWrite_OD(uint8_t pin, uint8_t value)
{
  pinMode(pin, (value == LOW) ? OUTPUT : OPENDRAIN);
  digitalWrite(pin, value);
}

/*!***************************************************************************
 * @brief
 * Set Timer period for TCK generation
 * 
 * @param[in] period_us   Requested period in us
 * @return    uint32_t    Actual period in us
 *****************************************************************************/
static uint32_t setTimer1Period(uint32_t period_us)
{
  #define MAX_PERIOD_US   4194240UL   /* 4194.24 ms                          */
  #define MIN_PERIOD_US   1UL       /* 100 us                              */
  
  const uint8_t prescalerValues[] = {
    0 /* 1 */,
    3 /* 8 */,
    6 /* 64 */,
    8 /* 256 */,
    10 /* 1024 */
  };

  uint8_t prescalerSel;
  uint32_t reloadValue;
  bool valid = false;

  /* Limit TCK period range                                                  */
  period_us = limit(period_us, MIN_PERIOD_US, MAX_PERIOD_US);

  /* Calculate prescaler and reset value based on frequency in F_CPU         */
  for (prescalerSel = 0; prescalerSel < 5; ++prescalerSel)
  {
    reloadValue = ((F_CPU / 1000000UL) * period_us) >> prescalerValues[prescalerSel];
    if ((reloadValue > 0UL) && (reloadValue < 65535UL))
    {
      /* Found possible configuration                                        */
      valid = true;
      break;
    }
  }

  if (valid)
  {
    uiTimer1Reload = 65535 - reloadValue;

    /* Critical section - disable Timer 1 interrupts                         */
    TIMSK1 &= ~(1 << TOIE1);

    TCCR1A = 0;                       /* Normal mode                         */
    TCCR1B = (prescalerSel + 1) & 0x07; /* Prescaler                         */
    TCNT1 = uiTimer1Reload;           /* Preload counter value               */

    /* End of critical section - reenable Timer 1 interrupts                 */
    TIMSK1 |= 1 << TOIE1;
  }

  return ((1UL << prescalerValues[prescalerSel]) * (65535UL - uiTimer1Reload)) / (F_CPU / 1000000UL);

  #undef MAX_PERIOD_US
  #undef MIN_PERIOD_US
}

/*!***************************************************************************
 * @brief
 * Read bit value from multibyte vector 
 *
 * @param[in] *vector   Bit vector
 * @param[in] bit       Bit index (0-indexed)
 * @return    bool      Bit value
 *****************************************************************************/
static bool getBitFromVector(const uint8_t* vector, uint16_t bit)
{
  bool value = vector[bit >> 3] & (1 << (bit & 0x7));
  return value;
}

/*!***************************************************************************
 * @brief
 * Set bit value in multibyte vector
 * 
 * @param[inout] *vector  Bit vector
 * @param[in] bit         Bit index (0-indexed)
 * @param[in] value       New bit value
 *****************************************************************************/
static void setBitInVector(uint8_t* vector, uint16_t bit, bool value)
{
  if (value)
  {
    vector[bit >> 3] |= 1 << (bit & 0x7);
  }
  else
  {
    vector[bit >> 3] &= ~(1 << (bit & 0x7));
  }
}

/*!***************************************************************************
 * @brief
 * Setup routine
 *****************************************************************************/
void setup()
{
  /* Start serial port at 115200 baud, no parity, 8 bit words and 1 stop bit */
  Serial.begin(115200);

  /* Initialise digital GPIO pins                                            */
  pinMode(TMS_PIN, OPENDRAIN);
  pinMode(TCK_PIN, OPENDRAIN);
  pinMode(TDO_PIN, OPENDRAIN);
  pinMode(TDI_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  /* Initialise state machine                                                */
  eState = EN_JTAG_STATE_DONE;
  uiDataLen = 0;
  uiDataIndex = 0;
  memset((uint8_t*)aucDataTMS, 0, sizeof(aucDataTMS));
  memset((uint8_t*)aucDataTDx, 0, sizeof(aucDataTDx));

  /* Initialise TCK timer                                                    */
  setTimer1Period(5UL);
}

/*!***************************************************************************
 * @brief
 * Background program loop
 *****************************************************************************/
void loop()
{
  String cmd = Serial.readStringUntil(':');
  if (cmd.compareTo("getinfo") == 0)
  {
    /* Get protocol version and max. vector length                           */
    Serial.print("xvcServer_v1.0:");
    Serial.print(DATA_BUF_SIZE, DEC);
    Serial.print("0\n");
  }
  else if (cmd.compareTo("settck") == 0)
  {
    /* Set TCK period (in ns)                                                */
    uint32_t period_req;
    uint32_t period_act;
    Serial.readBytes((uint8_t*)&period_req, 4);

    /* Period must be twice as fast for rising and falling edges             */
    period_act = setTimer1Period(period_req / 2000UL) * 500UL;

    Serial.write((const uint8_t*)&period_act, 4);
  }
  else if (cmd.compareTo("shift") == 0)
  {
    /* Shift data in/out of the device                                       */
    uint32_t num;
    uint32_t num_bytes;
    Serial.readBytes((uint8_t*)&num, 4);

    num_bytes = (num + 7) >> 3;  /* (n + 7) / 8 */
    if (num_bytes <= DATA_BUF_SIZE)
    {
      /* Prepare data and start transfer                                     */
      Serial.readBytes((uint8_t*)aucDataTMS, num_bytes);
      Serial.readBytes((uint8_t*)aucDataTDx, num_bytes);
      memset((uint8_t*)aucDataTDx + num_bytes, 0, DATA_BUF_SIZE - num_bytes - 1);
      uiDataLen = num;
      eState = EN_JTAG_STATE_START;

      /* Wait until transfer is complete                                     */
      while (eState != EN_JTAG_STATE_DONE);

      /* Reply with TDO vector                                               */
      Serial.write((const char*)aucDataTDx, num_bytes);
    }
    else
    {
      /* Error - shift out zero-bytes                                        */
      do {
        Serial.write((uint8_t)'\0');
        --num_bytes;
      } while (num_bytes > 0);
    }
  }
}

/*!***************************************************************************
 * @brief
 * Handle JTAG data transfer synchronously to TCK in Timer 1 OVF interrupt
 *****************************************************************************/
ISR(TIMER1_OVF_vect)
{
  TCNT1 = uiTimer1Reload;

  /* JTAG interface handler FSM                                              */
  switch (eState)
  {
    case EN_JTAG_STATE_START:
      /* Start new transfer
       * Set up initial TDI/TMS/TCK values                                   */
      uiDataIndex = 0;
      digitalWrite_OD(TCK_PIN, LOW);
      digitalWrite(LED_PIN, HIGH);
      digitalWrite_OD(TMS_PIN, getBitFromVector((const uint8_t*)aucDataTMS, uiDataIndex));
      digitalWrite_OD(TDI_PIN, getBitFromVector((const uint8_t*)aucDataTDx, uiDataIndex));
      eState = EN_JTAG_STATE_RISINGEDGE;
      break;

    case EN_JTAG_STATE_RISINGEDGE:
      /* Generate rising edge on TCK
       * Shift in TDO                                                        */
      if (uiDataIndex < uiDataLen)
      {
        setBitInVector((uint8_t*)aucDataTDx, uiDataIndex, digitalRead(TDO_PIN));
      }

      if (uiDataIndex >= uiDataLen)
      {
        /* Shifted in last bit - transfer complete                           */
        digitalWrite(LED_PIN, LOW);
        eState = EN_JTAG_STATE_DONE;
      }
      else
      {
        digitalWrite_OD(TCK_PIN, HIGH);
        eState = EN_JTAG_STATE_FALLINGEDGE;
      }
      break;

    case EN_JTAG_STATE_FALLINGEDGE:
      /* Generate falling edge on TCK
       * Set up new TMS/TDI                                                  */
      digitalWrite_OD(TCK_PIN, LOW);
      ++uiDataIndex;
      if (uiDataIndex < uiDataLen)
      {
        digitalWrite_OD(TMS_PIN, getBitFromVector((const uint8_t*)aucDataTMS, uiDataIndex));
        digitalWrite_OD(TDI_PIN, getBitFromVector((const uint8_t*)aucDataTDx, uiDataIndex));
      }
      eState = EN_JTAG_STATE_RISINGEDGE;
      break;

    case EN_JTAG_STATE_DONE:
    default:
      ;
  }
}