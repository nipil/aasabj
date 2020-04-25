/*********************************************************************************************************************************/
#include <Stepper.h>
#include <SevSeg.h>

/*********************************************************************************************************************************/
//#define DEBUG_SERIAL

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

// PINOUT for buttons
const byte PIN_BUTTON_A = 29;
const byte PIN_BUTTON_B = 30;

// PINOUT for direction setting
const byte PIN_CONFIG_DIRECTION = 31;

// PINOUT for reset position
const byte PIN_RESET_LIMIT_SWITCH = 32;

// PINOUT for stepper driver and pin order
const byte PIN_STEPPER_DRIVER[] = { 10, 11, 12, 13 };
const byte PIN_STEPPER_DRIVER_ORDER[] = { 0, 2, 1, 3 }; // 28BYJ-48

/*********************************************************************************************************************************/
// physics (preference)
const unsigned long DEFAULT_KERF_STEPS = 100;
const unsigned long DEFAULT_FINGER_STEPS = 2000;
const unsigned long STEPS_PER_LOOP = 1;

// physics (motor)
const unsigned short STEPPER_STEPS_PER_REV = /* natural */ 32 * 64 /* gear reduction */; // for 28BYJ-48
const byte STEPPER_ROTATION_PER_MINUTE = 15; // for 28BYJ-48

// electronics
const unsigned long DEBOUNCE_TIME_MS = 200;

// input (preference)
const unsigned long INPUT_LONG_PRESS_MS = 1000;
const unsigned long INPUT_IDLE_TIME_MS = 2000;

/*********************************************************************************************************************************/
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
class DebouncedSwitch {
	protected:
		const byte digitalInputPin;
		const bool restingState;
		bool storedState;
		const unsigned short debounceMs;
		unsigned long previousChangeMs;

	public:
		DebouncedSwitch(const byte& digitalInputPin, const bool& restingState = HIGH, const unsigned short& debounceMs = DEBOUNCE_TIME_MS)
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
			//    DPRINTLN("reset previous change ms");
			if (currentMs == 0)
			{
				previousChangeMs = millis();
			}
			else
			{
				previousChangeMs = currentMs;
			}
			//    DPRINTLN(previousChangeMs, DEC);
		}

	protected:
		unsigned long getLastChangeMs() const
		{
			return previousChangeMs;
		}
};

/*********************************************************************************************************************************/
class MultiFunctionButton : public DebouncedSwitch {

	private:
		bool previousState;
		const unsigned short longPressMs;
		bool stateChanged;

	public:
		MultiFunctionButton(const byte& digitalInputPin, const bool& restingState = HIGH, const unsigned short& debounceMs = DEBOUNCE_TIME_MS, const unsigned short& longPressMs = INPUT_LONG_PRESS_MS)
			: DebouncedSwitch(digitalInputPin, restingState, debounceMs), previousState(restingState), longPressMs(longPressMs), stateChanged(false)
		{
		}

		void update()
		{
			DebouncedSwitch::update();
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

		void handleResetLimitSwitch()
		{
			switch(state) {
				case MSM_RESET_POSITION:
					DPRINTLN("handleResetLimitSwitch");
					DPRINTLN("MSM_RESET_POSITION");
					setState(MSM_IDLE);
					break;

				default:
					break;
			}
		}
};

/*********************************************************************************************************************************/
class DirectionFlag
{
	private:
		const byte digitalInputPin;

	public:
		DirectionFlag(const byte& pin)
			: digitalInputPin(pin)
		{
		}

		void setup()
		{
			pinMode(digitalInputPin, INPUT_PULLUP);
		}

		int getDirection() const
		{
			if (digitalRead(digitalInputPin))
			{
				return +1;
			}
			else
			{
				return -1;
			}
		}
};

/*********************************************************************************************************************************/

// components
SevSeg sevseg;
MultiFunctionButton buttonA(PIN_BUTTON_A);
MultiFunctionButton buttonB(PIN_BUTTON_B);
DebouncedSwitch resetLimit(PIN_RESET_LIMIT_SWITCH);

Stepper screwStepper(STEPPER_STEPS_PER_REV,
		PIN_STEPPER_DRIVER[PIN_STEPPER_DRIVER_ORDER[0]],
		PIN_STEPPER_DRIVER[PIN_STEPPER_DRIVER_ORDER[1]],
		PIN_STEPPER_DRIVER[PIN_STEPPER_DRIVER_ORDER[2]],
		PIN_STEPPER_DRIVER[PIN_STEPPER_DRIVER_ORDER[3]]);

// physics
unsigned long stepsKerf = DEFAULT_KERF_STEPS;
unsigned long stepsFinger = DEFAULT_FINGER_STEPS;
unsigned long remainingSteps = 0;
DirectionFlag directionConfig(PIN_CONFIG_DIRECTION);

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

	// direction setting configuration
	directionConfig.setup();

	// stepper
	screwStepper.setSpeed(STEPPER_ROTATION_PER_MINUTE);

	// input switches
	buttonA.setup();
	buttonB.setup();
	resetLimit.setup();

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
void handleInteractiveInputs()
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

	// long press A : set kerf steps
	else if (buttonA.isPressed() && buttonA.isLongPress() && !buttonB.isPressed())
	{
		DPRINTLN("long press A");
		buttonA.resetPreviousChangeMs();
		buttonB.resetPreviousChangeMs();
		msm.handleLongPressA();
	}

	// long press B : set finger steps
	else if (buttonB.isPressed() && buttonB.isLongPress() && !buttonA.isPressed())
	{
		DPRINTLN("long press B");
		buttonA.resetPreviousChangeMs();
		buttonB.resetPreviousChangeMs();
		msm.handleLongPressB();
	}

	// release A : move to next kerf
	else if (buttonA.hasChanged() && !buttonA.isPressed())
	{
		DPRINTLN("release A");
		msm.handleReleaseA();
		if (msm.getState() == MyStateMachine::MSM_MOVE_KERF)
		{
			DPRINT("prepare to move kerf steps total ");
			remainingSteps = stepsKerf;
			DPRINTLN(remainingSteps, DEC);
		}
	}

	// release B : move to next finger
	else if (buttonB.hasChanged() && !buttonB.isPressed())
	{
		DPRINTLN("release B");
		msm.handleReleaseB();
		if (msm.getState() == MyStateMachine::MSM_MOVE_FINGER)
		{
			DPRINT("prepare to move finger steps total ");
			remainingSteps = stepsFinger;
			DPRINTLN(remainingSteps, DEC);
		}
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
void handleNonInteractiveInputs()
{
	resetLimit.update();
	//  DPRINTLN("handleNonInteractiveInputs");
	if (resetLimit.isPressed())
	{
		//    DPRINTLN("resetLimit isPressed");
		msm.handleResetLimitSwitch();
	}
}

/*********************************************************************************************************************************/
void blinkDisplay(SevSeg& svsg, int speed)
{
	unsigned long now = millis();
	unsigned long blinkMask = (1<<speed) - 1;
	unsigned long blinkThreshold = 1 << (speed - 1);
	unsigned long modulo = now & blinkMask;

	if (modulo > blinkThreshold)  {
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
void handleMotors()
{
	long stepperSteps, loopSteps;

	switch(msm.getState())
	{
		case MyStateMachine::MSM_MOVE_KERF:
			loopSteps = min(STEPS_PER_LOOP, remainingSteps);
			stepperSteps = +directionConfig.getDirection() * loopSteps;
			if (remainingSteps > 0)
			{
				DPRINT("move kerf step FORWARD ");
				DPRINTLN(stepperSteps, DEC);
				screwStepper.step(stepperSteps);
				remainingSteps = remainingSteps - loopSteps;
				if (remainingSteps == 0)
				{
					msm.setState(MyStateMachine::MSM_IDLE);
				}
			}
			break;

		case MyStateMachine::MSM_MOVE_FINGER:
			if (remainingSteps > 0)
			{
				loopSteps = min(STEPS_PER_LOOP, remainingSteps);
				stepperSteps = +directionConfig.getDirection() * loopSteps;
				DPRINT("move finger step FORWARD ");
				DPRINTLN(stepperSteps, DEC);
				screwStepper.step(stepperSteps);
				remainingSteps = remainingSteps - loopSteps;
				if (remainingSteps == 0)
				{
					msm.setState(MyStateMachine::MSM_IDLE);
				}
			}
			break;

		case MyStateMachine::MSM_RESET_POSITION:
			stepperSteps = -directionConfig.getDirection() * STEPS_PER_LOOP;
			DPRINT("move reset step REVERSE ");
			DPRINTLN(stepperSteps, DEC);
			screwStepper.step(stepperSteps);
			break;

		default:
			break;
	}
}

/*********************************************************************************************************************************/
void loop()
{
	//  unsigned long t = micros();

	handleInteractiveInputs();
	handleNonInteractiveInputs();
	handleDisplay();
	handleMotors();

	if (millis() > lastDebugMs + 1000) {
		lastDebugMs = millis();
		DPRINT("debug state : ");
		DPRINTLN(msm.getState());
		//    Serial.println(micros() - t);
	}
}
