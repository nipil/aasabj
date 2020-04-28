# aasabj
Another arduino screw advance box jig

# components used

- 1x 5641AS common cathode 7 segment display
- 4x 1k resistors to current drop the 7 segment display leds
- 2x NO switch for inputs (button "A" and "B")
- 1x NO limit switch for reset and homing (limit switch)
- 1x stepper driver with 4 input pins
- 1x stepper motor

# configuration and setup

Open the arduino file and adapt the following to your needs :

- The pinout section to adapt to your circuit
- The PIN_STEPPER_DRIVER_ORDER array, to adapt to your motor step order
- The STEPPER_STEPS_PER_REV constant, to adapt to your motor (include it's reduction, i.e. set the number of steps required to do a real-world "one-turn" !)
- The MAX_STEPPER_ROTATION_PER_MINUTE constant, to correspond to your desired real-world max speed (watch for torque)
- The DEFAULT_KERF_STEPS, to get a "ball-park" value for your saw kerf
  - see the comments in the code to get the method to derive your real values into the number of steps
  - you can then do some tests and adjust the value on the fly running your circuit (see below)
  - finally, when you get the fit you want, read the value on the display and write it in the code
  - upload the modified code, and voila! it is the default value every time you boot the circuit

# use of the program and circuit

A short press on the "A" button will move the amount of steps required to move the jig by a saw-kerf

A long press on button "A" will enter "kerf step edit" mode. When in that mode :

- a short press on button "A" will increase the kerf steps value by one
- a short press on button "B" will decrease the kerf steps value by one
- no activity for a few seconds will exit "kerf step edit" mode

A long press on button "B" will enter "finger edit" mode. When in that mode :

- a short press on button "A" will increase the number of kerfs to make a finger by one
- a short press on button "B" will decrease the number of kerfs to make a finger by one
- no activity for a few seconds will exit "finger edit" mode

A short press on the "B" button will move the amount of steps required to move the jig by a finger, i.e. a few kerf.

The direction pin will dictate in which way the screw turns :

- the "move kerf" action and "move finger" turns one way
- the "reset" action turns the opposite way
- if you can leave the direction pin unconnected these will turn in a fashion
- if you connect the direction pin to the ground these action will turn in the opposite fashion
- first, try without connecting the direction pin
- then do one "move kerf" action, and see if it turns/moves the way you want
- if yes, leave it unconnected ; if not, connect the direction pin to the ground

A long press on *BOTH* the "A" and "B" button will enter "reset" mode :

- in "reset" mode, the screw turns infinitely
- it will only stop when the "limit switch" is closed (home position)
- this is used to pull back the jig before/after cutting all the fingers in a wood-piece

# prototype

See the photo in the project for a overall presentation of the protoype breadboard
