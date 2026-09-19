# ALITRAY CONTROLLER
[Engineering Notebook](https://drive.google.com/drive/folders/1m8bGWwMpdiL9uz0l-eo00PWUPM0APUVN?usp=drive_link)
This controller includes 5 main features *most* of which are handled in degrees.
* movement queuing
* gear reduction based calculations
* cartesian movement
* relative movement
* absolute movement

This library is written with an Arduino and CNC shield v4 in mind however it can still be used
with any micro controller that can run nema17 motors. The current setup is based on how my arm
is setup (the pins, gear ratio, etc.) but this can be further customized easily by changing some
variables.

## Movements
The main movements are going to be:
```
arm.moveToPoint(x, y, z);
stepper.moveSteps(steps);
steper.moveToAngle(angle);
stepper.moveByDeg(degree);
arm.stop();
arm.run()
```
where arm is all the motors combined and stepper is targetting individual motors. Move to point
handles the inverse kinematics inside of the function and the origin coordinate is set by the user

arm.run() or stepper.run() need to be placed inside the while loop since this is what polls the motor
to take a step in the needed direction.

**This project has been considered finished due to having achieved my goal and a driver burning up however I plan to remake it with a 6 motor version**
