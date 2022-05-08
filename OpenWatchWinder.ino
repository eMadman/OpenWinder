// Configuration
#define CYCLES 32
#define ROT_R 5  //default 2
#define ROT_L 5  //default 2
#define PAUSE_MIN 43  //default 35

// Pins
#define LED_PIN 5
#define SW_PIN 4

// Motor
#define ROT_SPEED 1000
#define ROT_ACCEL 800
#define ROT_PAUSE 4000
#define ROT_STEPS 4096

#include <jled.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

enum StateType
{
  W_IDLE,  // 0
  W_CYCLE, // 1
  W_RIGHT, // 2
  W_LEFT,  // 3
  W_STOP,  // 4
  W_PAUSE  // 5
};

AccelStepper winder(8, 8, 10, 9, 11, false);
JLed pwr_led = JLed(LED_PIN).FadeOn(1000);


StateType WState = W_IDLE;
int Rotations = CYCLES;
int Start = false;
int Continue = false;
int Stop = false;
int LedOn = true;
int LastMinute = false;
int TargetPos = 0;
long StartTime = 0;


void setup()
{
  Serial.begin(115200);
  pinMode(SW_PIN, INPUT_PULLUP);

  
  winder.setMaxSpeed(ROT_SPEED);
  winder.setAcceleration(ROT_ACCEL);
  WState = W_IDLE;

  Serial.println("<< Winder: Ready");

  Serial.println(">> Click: Start Winding");
  Stop = false;
  Start = true;
  Continue = true;
  WState = W_IDLE;
}

void loop()
{
  StateType old_state = WState;

  switch (WState)
  {
  case W_IDLE:
    if (Start || Continue)
    {
      Start = false;
      StartTime = millis();
      winder.enableOutputs();
      Rotations = CYCLES;
      WState = W_CYCLE;
    }
    break;

  case W_CYCLE:
    if (LedOn)
    {
      pwr_led.Reset();
      pwr_led.Blink(1000, 200).Forever();
    }

    if (Stop)
    {
      WState = W_STOP;
    }
    else if ((Rotations--) > 0)
    {
      Serial.print("## Cycles until Pause: ");
      Serial.println(Rotations);
      WState = W_RIGHT;
      TargetPos -= (ROT_R * ROT_STEPS);
      winder.moveTo(TargetPos);
    }
    else
    {
      WState = W_STOP;
    }
    break; // case W_CYCLE

  case W_RIGHT:
    if (winder.distanceToGo() != 0)
    {
      winder.run();
    }
    else if (Stop)
    {
      WState = W_STOP;
    }
    else
    {
      WState = W_LEFT;
      TargetPos += (ROT_L * ROT_STEPS);
      winder.moveTo(TargetPos);
    }
    break; // case W_RIGHT

  case W_LEFT:
    if (winder.distanceToGo() != 0)
    {
      winder.run();
    }
    else if (Stop)
    {
      WState = W_STOP;
    }
    else
    {
      WState = W_CYCLE;
    }
    break; // case W_LEFT

  case W_STOP:
    Stop = false;
    winder.disableOutputs();
    StartTime = millis();
    if (Continue)
    {
      Serial.println("<< Winder: Stopped, waiting for next cycle");
      WState = W_PAUSE;
      if (LedOn)
      {
        pwr_led.Reset();
        pwr_led.Breathe(5000).Forever();
      }
    }
    else
    {
      Serial.println("<< Winder: Stopped, will not restart");
      if (LedOn)
      {
        pwr_led.Reset();
        pwr_led.On();
      }
      WState = W_IDLE;
    }
    break; // case W_STOP

  case W_PAUSE:
    long temp = millis() - StartTime;
    long delta = 60L * 1000L;

    if (Start)
    {
      WState = W_IDLE;
    }
    // run once, 1 minute before PAUSE_MIN elapses
    else if ((!LastMinute) && (temp > (delta * long(PAUSE_MIN - 1))))
    {
      Serial.println("<< Winder: Restarting in 1 minute");
      if (Continue && LedOn)
      {
        pwr_led.Reset();
        pwr_led.Breathe(1000).Forever();
      }
      LastMinute = true;
    }
    // PAUSE_MIN has elapsed && Continue
    else if (Continue && temp > (delta * long(PAUSE_MIN)))
    {
      LastMinute = false;
      WState = W_IDLE;
    }
    break; // W_PAUSE

  default:
    winder.disableOutputs();
    break;

  } // switch

  if (WState != old_state)
  {
    Serial.print("-- Changing State: ");
    Serial.print(old_state);
    Serial.print(" -> ");
    Serial.println(WState);
  }

  pwr_led.Update();

} // loop
