#include <Wire.h>
#include "A4990ValveInterface.h"

/*
  NOTE ON VALVE COMMAND SIGNALS
  -----------------------------
  Valve commands are on the interval [-400,400].
  Negative numbers mean vent and positive mean fill.
  If the valve is flipped, the wires need to be switched.

*/

/*
  IMPORTANT NOTE ON TIMER FREQUENCIES
  ------------------------------------
  On the Arduino Nano Every (ATmega 4809 chipset), there are two timers related to the PWM pins we
  using: TCA for D5 and D9 and TCB for D3 and D6.

  The default pwm freq for these pins is 975 Hz = fclk_per/64 (where fclk_per is the peripheral clock,
  and is set to 62.4kHz by default. This freq causes a really annoying hum when the valves are being
  controlled. To get rid of this, we prescale the timers to 31.2kHz = fclk_per/2 in speedupPWM() because
  31.2 kHz is well outside the range of what a human can hear.

  TCB can be changed like this without causing any issues. TCA, on the other hand, is used by the
  built-in delay() and millis() functions. Since we changed the freq by a factor of 32 (PRESCALER),
  this means that millis() now reports 32 times the actual time passed (e.g. millis()/PRESCALER is the
  actual number of milliseconds that have passed).

  Another consequence of changing TCA is that delay() now (for some reason) can only delay up to
  32,768 (2^16/2) which corresponds to an actual delay of 32768/32 = 1024 ms. So, there is a custom
  delay function defined custom_delay() which allows you to delay for longer than 1024 ms. Since
  delay() function isn't used in the control loop, this is adequate for our needs.
*/

A4990ValveInterface valves;

float p[4] = {0, 0, 0, 0};
float pcmd[4] = {0, 0, 0, 0};
float pbias[4] = {0, 0, 0, 0};

// arrays of 8 bytes used for i2c comms. Each pressure uses 2 bytes (16 bits).
const int BYTES_PER_PRESSURE = 2;
byte pchar[BYTES_PER_PRESSURE * 4];
byte pcmdchar[BYTES_PER_PRESSURE * 4];

int valve_cmd[4] = {0, 0, 0, 0};
int VENT_CMD[4] = { -400, -400, -400, -400};
int FILL_CMD[4] = {400, 400, 400, 400};

//'Global' variables that are important for control
float myTime = 0.0;
float prevTime = 0.0; // Last time the loop was entered
float integrator[4] = {0, 0, 0, 0};
float pdot[4] = {0, 0, 0, 0};
float prevError[4] = {0, 0, 0, 0}; // error at the previous timestep
float prevP[4] = {0, 0, 0, 0};
float errorDot[4] = {.0, .0, .0, .0}; // Derivative of the error
float awl = 1.0;                      // Anti-Windup limit.
float kp = 10.0;
float ki = 0.000;
float kd = 0.00;
float sigma = .05;    // Dirty Derivative Bandwidth = 1/sigma
float deadband = 0.0; // kpa, controller will not act on error less than deadband

float error = 0.0;
float dt = 0;

// motor driver error pins
const int EF1_A = 10;
const int EF2_A = 11;
const int EF1_B = 20;
const int EF2_B = 21;

// conversion factor from psi to kpa (ie kpa = psi * PSI2KPA)
double const PSI2KPA = 6.8947572932;
double const P_MAX = 100 * PSI2KPA;

// timer stuff
int const PRESCALER = 32;
int const ONE_SECOND = 32000;

//step tests
int const steps = 62;
float millisperstep = 5000;
//int valve_trajectory[steps] = {0, 100, -200, 400, -300, 250, -400, 400, -400, 0};
//int valve_trajectory[steps] = {0, 100, 200, 300, 400, 300, 200, 100, 400, 0};
//int valve_trajectory[steps] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 0};
//int valve_trajectory[steps] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 0};
//need to be less than 140.
int valve_trajectory[steps] = {0, 114, -137,   77,  117,  105,   76,   26,   58,   57,  -42,   86,
         16,   32,   17, -112, -100,   93, -119,  108,  118,  123, -104,
       -135,  -76,  127,  101,  131,  -58,   79,  -63,   54,   27,   71,
       -138, -114,  -17,  -61,  -95, -119,  -41,   46,   82,  -72,   -2,
          8, -121,   -6,  -60,   68,  -26,   41, -128, -136,  102,   86,
        113,  103,   30,  -43,  -44, 0};

void setup(void)
{

  speedupPWM();

  // comment out if not debugging
  Serial.begin(115200);

  // set up fault pins, input pullup bc no external pull is used.
  pinMode(EF1_A, INPUT_PULLUP);
  pinMode(EF2_A, INPUT_PULLUP);
  pinMode(EF1_B, INPUT_PULLUP);
  pinMode(EF2_B, INPUT_PULLUP);


//  //wait for command to start
//  while (Serial.available() == 0) {
//  }

}

void loop(void)
{

  for (int i = 0; i < steps; i++) {
    float mytime = millis();

    while ((millis() - mytime) / PRESCALER < millisperstep) {

      valves.setValve0Speed(valve_trajectory[i]);

      //serial print pressure0 readings
      p[0] = filter(p[0], readPressure(A0));
      Serial.println(p[0] / PSI2KPA);

      //serial print valve0 command
      Serial.println(valve_trajectory[i]);
    }
  }

  exit(0);

}


float filter(float prev, float input)
{
  /*
      This function implements a first order low pass filter
      with a cutoff frequency of 50 Hz. First order hold discrete implementation with dt=.001.
      First order filter is of form:
      a / (z - b)
  */

  float a = .2696;
  float b = .7304;

  return b * prev + a * input;
}

double readPressure(int analogPin)
{
  /*
     ADC is 10 bit, so voltage is read as bin numbers ranging between 0 and 1023
     Analog reference is 5V, so 0=0v and 1023=5v
     Conversion is taken from Fig. 3 (transfer function A) in pressure sensor datasheet
     https://www.mouser.com/datasheet/2/187/honeywell-sensing-basic-board-mount-pressure-abp-s-1224358.pdf
  */
  double v_sup = 5;

  // convert bin number to a voltage
  double v_out = analogRead(analogPin) * 5.0 / 1024.0;

  // return applied pressure in kPa
  return ((v_out - 0.1 * v_sup) / (0.8 * v_sup)) * P_MAX;
}

void speedupPWM()
{
  cli(); // Disable Interrupts

  /* see section 20.5.1 of http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega4808-4809-Data-Sheet-DS40002173A.pdf
     for this register settings. Note that by default, fclk_per = 62.4 kHz.
     Humans can hear 20 Hz to 20 kHz, so a 31.2kHz freq should be out of hearing range.
     Also see section 5.4.1 for syntax used here.
     | is "bitwise or" commonly used to set multiple bits at once in a register.
     TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm --> 0,0,1,0 OR 0,0,0,1 = 0,0,1,1

     READ TIMER FREQUENCY NOTE AT TOP
  */

  // TCB for D3 and D6
  TCB0.CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  // TCA for D5 and D9
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;

  sei(); // Enable Interrupts
}

void custom_delay(int seconds)
{
  for (int s = 0; s < seconds; s++)
  {
    delay(ONE_SECOND);
  }
}
