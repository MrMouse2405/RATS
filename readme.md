# RATS

Rats is an in-house robotics challenge using Pololu3pi32U4 Robot.

We programmed a Robot capable of not just accurately "finding" anomalies,
but also do it at a high speed.

## Challenge

RATS has arrived at a new planet. Its transport ship has completed its scans from orbit and
Mining Operations has identied a promising area for mineral extraction. Now it’s time for
detailed measurements on the ground. RATS will be sent to the surface to execute a preplanned survey.
As RATS Software Engineers, you must ensure RATS has the programming necessary to
complete its mission. This includes the requirements to:

1. calibrate its reectance sensors and accelerometer to actual surface conditions;
2. follow a pre-planned survey path;
3. keep track of its position at all times;
4. take alternate routes when obstacles are encountered;
5. at specied points, measure ground reectance and surface pitch and roll angles; and
6. identify the location of any magnetic anomalies encountered en route.

Accuracy and speed are both essential. Competing mining teams may be arriving soon, so
the survey must be completed as quickly as possible so a claim can be staked.

## Problems and Solutions

1. **Line "Path" Following**: Accomplished using a PID controller that uses IR Sensors to "see".
2. **Anomaly Detection & Orientation of Robot**: Solved using IMU controller.
3. **Obstacle Avoidance**: Bump Sensors or "IR Sensors tracking the position of skirt in front of the robot"

## Speed vs Accuracy: The Difficult Optimization Problem

Accurately accomplishing the primary robot tasks was easy,
the challenge was making it **_FASTER_**.

The speed of the robot is compromised when running multiple sensors.
A trade off was needed between running multiple sensors and slow movement speed,
or few sensors with high movement speed.

The robot is moving at 1.5 m/s and is using IR sensor where each sample takes 2000µs.
The more samples we take, the more accurately our PID controller can move robot.

If we take too many samples, we are overfitting (it will be accurate, but we are doing it too much).
If we takke too few samples, we are introducing lag which will make the robot movement jittery and too sharp.

## Solution: The "Game Loop" Design Pattern

We made few observations in our solution:

1. Only "Path" following and "Anomaly Detection" needed to run ALWAYS.
2. Few Sensors can be activated at certain conditions, and then deactivated.
3. Data can be collected, and then analyzed at the end.
4. We can split up work between samples, this means detection might not be
   "instant", but the error is negligeable.

The Game Loop Design Pattern helps us solve all of this. We created a simple
loop that **runs tasks for fixed number of ms**.

### 1. The Task
