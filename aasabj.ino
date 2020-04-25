/*********************************************************************************************************************************/
#include <SevSeg.h>



/*********************************************************************************************************************************/
// PINOUT for 5641AS (4 digit 7 segment common cathode)
const byte pin_1_segment_E_arduino_pin = 2;
const byte pin_2_segment_D_arduino_pin = 3;
const byte pin_3_segment_DP_arduino_pin = 4;
const byte pin_4_segment_C_arduino_pin = 5;
const byte pin_5_segment_G_arduino_pin = 6;
const byte pin_6_digit_4_common_arduino_pin = 26;
const byte pin_7_segment_B_arduino_pin = 22;
const byte pin_8_digit_3_common_arduino_pin = 27;
const byte pin_9_digit_2_common_arduino_pin = 28;
const byte pin_10_segment_F_arduino_pin = 23;
const byte pin_11_segment_A_arduino_pin = 24;
const byte pin_12_digit_1_common_arduino_pin = 25;

// PINOUT for input switches
const byte pin_switch_A = 29;
const byte pin_switch_B = 30;

/*********************************************************************************************************************************/
// physics
const unsigned long DEFAULT_KERF_STEPS = 100;
const unsigned long DEFAULT_FINGER_STEPS = 2000;

// electronics
const unsigned long DEBOUNCE_TIME_MS = 200;

// input
const unsigned long INPUT_LONG_PRESS_MS = 1000;
const unsigned long INPUT_IDLE_TIME_MS = 5000;

/*********************************************************************************************************************************/
#define DEBUG_SERIAL
     
#ifdef DEBUG_SERIAL
#define DPRINT(...)    Serial.print(__VA_ARGS__)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DPRINTF(...)    Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__))

#else
#define DPRINT(...)     //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTF(...)    //blank line
#define DPRINTLNF(...)  //blank line

#endif

/*********************************************************************************************************************************/
class DebouncedButton {
protected:
  const byte digitalInputPin;
  const bool restingState;
  bool storedState;
  const unsigned short debounceMs;
  unsigned long previousChangeMs;
  
public:
  DebouncedButton(const byte& digitalInputPin, const bool& restingState = HIGH, const unsigned short& debounceMs = DEBOUNCE_TIME_MS)
    : digitalInputPin(digitalInputPin), restingState(restingState), storedState(restingState), debounceMs(debounceMs), previousChangeMs(0)
  {
  }
  
  void setup()
  {
    pinMode(digitalInputPin, INPUT_PULLUP);
  }
  
  void update()
  {
    unsigned long currentMs = millis();
    bool currentState = digitalRead(digitalInputPin);
    if (currentState != storedState)
    {
      if (currentMs >= previousChangeMs + debounceMs)
      {
        storedState = currentState;
        resetPreviousChangeMs(currentMs);
      }
    }
  }

  bool getState() const
  {
    return storedState;
  }

  bool isPressed() const
  {
    if (storedState == restingState)
    {
      return false;
    }
    else
    {
      return true;
    }    
  }

  void resetPreviousChangeMs(const unsigned long& currentMs = 0)
  {
    DPRINTLN("reset previous change ms");
    if (currentMs == 0)
    {
      previousChangeMs = millis();
    }
    else
    {
      previousChangeMs = currentMs;
    }
    DPRINTLN(previousChangeMs, DEC);
  }

protected:
  unsigned long getLastChangeMs() const
  {
    return previousChangeMs;
  }
};

/*********************************************************************************************************************************/
class MultiFunctionButton : public DebouncedButton {

private:
  bool previousState;
  const unsigned short longPressMs;
  bool stateChanged;
  
public:
  MultiFunctionButton(const byte& digitalInputPin, const bool& restingState = HIGH, const unsigned short& debounceMs = DEBOUNCE_TIME_MS, const unsigned short& longPressMs = INPUT_LONG_PRESS_MS)
    : DebouncedButton(digitalInputPin, restingState, debounceMs), previousState(restingState), longPressMs(longPressMs), stateChanged(false)
  {
  }
  
  void update()
  {
    DebouncedButton::update();
    bool currentState = getState();
    if (previousState != currentState)
    {
      stateChanged = true;
      previousState = currentState;
    }
    else
    {
      stateChanged = false;
    }
  }

  bool hasChanged() const
  {
    return stateChanged;
  }

  bool isLongPress() const
  {
    unsigned long currentMs = millis();
    unsigned long sinceLastChangeMs = currentMs - getLastChangeMs();
    if (sinceLastChangeMs >= longPressMs)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

/*********************************************************************************************************************************/
class MyStateMachine
{
public:
  typedef enum {
    MSM_IDLE = 100,
    MSM_MOVE_KERF,
    MSM_MOVE_FINGER,
    MSM_SET_KERF,
    MSM_SET_FINGER,
    MSM_RESET_POSITION,
  } MsmState;

private:
  MsmState state;

public:
  MyStateMachine()
    : state(MSM_IDLE)
  {
  }

  MsmState getState() const
  {
    return state;
  }

  void setState(const MsmState& next)
  {
    state = next;
    DPRINTLN("setState");
    DPRINTLN(state, DEC);
  }

  void handleDoubleLongPress()
  {
    DPRINTLN("handleDoubleLongPress");
    switch(state)
    {
      case MSM_IDLE:
        DPRINTLN("idle");
        setState(MSM_RESET_POSITION);
        break;
        
      default:
        break;
    }
  }
  
  void handleLongPressA()
  {
    DPRINTLN("handleLongPressA");
    switch(state)
    {
      case MSM_IDLE:
        DPRINTLN("idle");
        setState(MSM_SET_KERF);
        break;
        
      default:
        break;
    }
  }

  void handleLongPressB()
  {
    DPRINTLN("handleLongPressB");
    switch(state)
    {
      case MSM_IDLE:
        DPRINTLN("idle");
        setState(MSM_SET_FINGER);
        break;
        
      default:
        break;
    }
  }

  void handlePressA()
  {
    DPRINTLN("handlePressA");
    switch(state)
    {
      default:
        break;
    }
  }

  void handlePressB()
  {
    DPRINTLN("handlePressB");
    switch(state)
    {
      default:
        break;
    }
  }

  void handleReleaseA()
  {
    DPRINTLN("handleReleaseA");
    switch(state)
    {
      case MSM_IDLE:
        setState(MSM_MOVE_KERF);
        break;
        
      default:
        break;
    }
  }

  void handleReleaseB()
  {
    DPRINTLN("handleReleaseB");
    switch(state)
    {
      case MSM_IDLE:
        setState(MSM_MOVE_FINGER);
        break;
        
      default:
        break;
    }
  }

  void handleIdleInput()
  {
    DPRINTLN("handleIdleInput");
    switch(state) {
      case MSM_SET_KERF:
          DPRINTLN("MSM_SET_KERF");
          setState(MSM_IDLE);
          break;
      case MSM_SET_FINGER:
          DPRINTLN("MSM_SET_FINGER");
          setState(MSM_IDLE);
          break;

      default:
        break;
    }
  }
};

/*********************************************************************************************************************************/

// components
SevSeg sevseg;
MultiFunctionButton buttonA(pin_switch_A);
MultiFunctionButton buttonB(pin_switch_B);

// physics
int stepsKerf = DEFAULT_KERF_STEPS;
int stepsFinger = DEFAULT_FINGER_STEPS;

// logic
MyStateMachine msm;

// interface
unsigned long lastActionMs = 0;

// debug
unsigned long lastDebugMs = 0;

/*********************************************************************************************************************************/
void setup()
{
  Serial.begin(9600);

  // input switches
  buttonA.setup();
  buttonB.setup();
  
  // setup sevseg
  const byte hardwareConfig = COMMON_CATHODE;
  const byte numDigits = 4;
  const bool disableDecPoint = false;
  const bool leadingZeros = false;
  const bool resistorsOnSegments = false;
  const bool updateWithDelays = false;
  byte digitPins[4] = {
    pin_12_digit_1_common_arduino_pin,
    pin_9_digit_2_common_arduino_pin,
    pin_8_digit_3_common_arduino_pin,
    pin_6_digit_4_common_arduino_pin
  };
  byte segmentPins[8] = {
    pin_11_segment_A_arduino_pin,
    pin_7_segment_B_arduino_pin,
    pin_4_segment_C_arduino_pin,
    pin_2_segment_D_arduino_pin,
    pin_1_segment_E_arduino_pin,
    pin_10_segment_F_arduino_pin,
    pin_5_segment_G_arduino_pin,
    pin_3_segment_DP_arduino_pin
  };
  sevseg.begin(
      hardwareConfig,
      numDigits,
      digitPins,
      segmentPins,
      resistorsOnSegments,
      updateWithDelays,
      leadingZeros,
      disableDecPoint
    );
}

/*********************************************************************************************************************************/
void handleInputs()
{
  buttonA.update();
  buttonB.update();

  bool actionOccured = true;

  // double long press
  if (buttonA.isPressed() && buttonB.isPressed() && buttonA.isLongPress() && buttonB.isLongPress())
  {
    DPRINTLN("double long press");
    buttonA.resetPreviousChangeMs();
    buttonB.resetPreviousChangeMs();
    msm.handleDoubleLongPress();
  }

  // long press A
  else if (buttonA.isPressed() && buttonA.isLongPress() && !buttonB.isPressed())
  {
    DPRINTLN("long press A");
    buttonA.resetPreviousChangeMs();
    buttonB.resetPreviousChangeMs();
    msm.handleLongPressA();
  }

  // long press B
  else if (buttonB.isPressed() && buttonB.isLongPress() && !buttonA.isPressed())
  {
    DPRINTLN("long press B");
    buttonA.resetPreviousChangeMs();
    buttonB.resetPreviousChangeMs();
    msm.handleLongPressB();
  }

  // release A
  else if (buttonA.hasChanged() && !buttonA.isPressed())
  {
    DPRINTLN("release A");
    msm.handleReleaseA();
  }

  // release B
  else if (buttonB.hasChanged() && !buttonB.isPressed())
  {
    DPRINTLN("release B");
    msm.handleReleaseB();
  }

  // press A
  else if (buttonA.hasChanged() && buttonA.isPressed())
  {
    DPRINTLN("press A");
    msm.handlePressA();
    
    switch(msm.getState())
    {
      case MyStateMachine::MSM_SET_KERF:
        stepsKerf++;
        break;
        
      case MyStateMachine::MSM_SET_FINGER:
        stepsFinger++;
        break;

      default:
        break;
    }
  }

  // press B
  else if (buttonB.hasChanged() && buttonB.isPressed())
  {
    DPRINTLN("press B");
    msm.handlePressB();
    
    switch(msm.getState())
    {
      case MyStateMachine::MSM_SET_KERF:
        stepsKerf--;
        break;
        
      case MyStateMachine::MSM_SET_FINGER:
        stepsFinger--;
        break;

      default:
        break;
    }
  }

  // no input
  else
  {
    actionOccured = false;
  }
  
  // handle idling
  if (actionOccured)
  {
    DPRINTLN("actionOccured");
    lastActionMs = millis();
    DPRINTLN(lastActionMs, DEC);
  }
  else
  {
    unsigned long curMs = millis();
    if (curMs > lastActionMs + INPUT_IDLE_TIME_MS)
    {
      DPRINTLN("no actionOccured");
      msm.handleIdleInput();
      lastActionMs = curMs;
    }
  }
}

/*********************************************************************************************************************************/
void blinkDisplay(SevSeg& svsg, int speed)
{
  unsigned long now = millis();
  unsigned long blinkMask = (1<<speed) - 1;
  unsigned long blinkThreshold = 1 << (speed - 1);
  unsigned long modulo = now & blinkMask;

  if (modulo > blinkThreshold)
  {
    svsg.refreshDisplay();
  }
  else
  {
    svsg.blank();
  }
}

/*********************************************************************************************************************************/
void handleDisplay()
{
  const unsigned long blink_slow = 8;
  const unsigned long blink_fast = blink_slow - 1;
  
  switch(msm.getState())
  {
    case MyStateMachine::MSM_IDLE:
      sevseg.setChars((char*) "IDLE");
      sevseg.refreshDisplay();
      break;

    case MyStateMachine::MSM_MOVE_KERF:
      sevseg.setChars((char*) "KERF");
      sevseg.refreshDisplay();
      break;
    
    case MyStateMachine::MSM_MOVE_FINGER:
      sevseg.setChars((char*) "FING");
      sevseg.refreshDisplay();
      break;

    case MyStateMachine::MSM_SET_KERF:
      sevseg.setNumber(stepsKerf);
      blinkDisplay(sevseg, blink_fast);
      break;
    
    case MyStateMachine::MSM_SET_FINGER:
      sevseg.setNumber(stepsFinger);
      blinkDisplay(sevseg, blink_slow);
      break;

    case MyStateMachine::MSM_RESET_POSITION:
      sevseg.setChars((char*) "RSET");
      sevseg.refreshDisplay();
      break;
  }
}

/*********************************************************************************************************************************/
void loop()
{
  handleInputs();
  handleDisplay();
  
  if (millis() > lastDebugMs + 1000) {
    lastDebugMs = millis();
    DPRINT("debug state : ");
    DPRINTLN(msm.getState());
  }
}
