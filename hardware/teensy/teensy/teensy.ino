#define ICP_PIN 10 // Input Capture Pin
#define LED_PIN 11
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz 0x00
#define TIMER3_CLOCK_PRESCALAR 8
#define WHEEL_GEAR_N_TEETH 39
#define WHEEL_DIAMETER 0.07291
#define SAMPLING_TIME 0.01

void printMeasuremets();

class SpeedMesaurement {
public:
  void onTimerOverflowInterrupt() {
    if (tickCounter == 0)
    {
      speed = 0;
      prevCapture = 0;
    } else {
      double tickPerSecond = tickCounter / accum_time;
      speed = tickPerSecond * SpeedMesaurement::TICK_PER_SECOND_TO_METER_PER_SECOND_FACTOR;
      tickCounter = 0;
      accum_time = 0;
    }
    printMeasuremets();
  }

  void onInputCaptureInterrupt() {
    unsigned int currentCapture = (ICR3L) | (ICR3H << 8);
    tickCounter += 1;
    accum_time += (currentCapture - prevCapture) * SpeedMesaurement::TICK_TO_SECOND_FACTOR;;
    prevCapture = currentCapture;
  }
  void begin() {
    pinMode(ICP_PIN, INPUT);  // INPUT_PULLUP , INPUT
    pinMode(LED_PIN, OUTPUT);
    cli();          // disable global interrupts
    TCCR3A = 0x00;
    // (TCNT3L) |(TCNT3H << 8) timer values
    // OCR3xH && OCR3xL output compare registers, x = A, B, C
    unsigned int outputCompare = F_CPU / TIMER3_CLOCK_PRESCALAR * SAMPLING_TIME;
    // outputCompare = 20000
    OCR3AH = (outputCompare >> 8);
    OCR3AL = (outputCompare | 0xFF);
    // ICR3H && ICR3L Input capture values
    // TIMSK3 interrupt mask
    TIFR3 = (1 << ICF3) | (1 << OCF3A) | (1 << TOV3); /* clear input capture flag */
    TIMSK3 = (1 << ICIE3) | (1 << OCIE3A); // TOIE3: overflow Int, ICIE3 Input Capture Interrupt, OCIE3A: OC A interrupt
    TCCR3B = (1 << ICES3) | (1 << CS31) | (1 << WGM32);  // | (1 << CS30) (1 << ICNC3) noise canceller & rising edge & clock on (8 prescalar)
    sei(); // enable global interrupts
  }
public:
  static double TICK_TO_SECOND_FACTOR;
  static double TICK_PER_SECOND_TO_METER_PER_SECOND_FACTOR;
  unsigned int prevCapture;
  bool isPrevCaptureValid;
  unsigned int tickCounter = 0;
  double accum_time = 0;
  double speed = 0;
};

double SpeedMesaurement::TICK_TO_SECOND_FACTOR = 1.0 / (F_CPU / TIMER3_CLOCK_PRESCALAR);
double SpeedMesaurement::TICK_PER_SECOND_TO_METER_PER_SECOND_FACTOR = (M_PI / WHEEL_GEAR_N_TEETH) * (WHEEL_DIAMETER/2);

SpeedMesaurement speedMeas;

void setup() {
  CPU_PRESCALE(CPU_16MHz);
  speedMeas.begin();
}

void printMeasuremets()
{
  Serial.println(speedMeas.speed);
  if (speedMeas.speed == 0) {
    digitalWrite(LED_PIN, false);
  }
}

void loop() {
}

ISR(TIMER3_CAPT_vect) {
  // Timer 3 input capture interrupt
  speedMeas.onInputCaptureInterrupt();
  digitalWrite(LED_PIN, true);
}

ISR(TIMER3_COMPA_vect) {
  // Timer 3 Output Compare A Interrupt
  // Expected every 0.01 sec (100 times per seconds)
  speedMeas.onTimerOverflowInterrupt();
}