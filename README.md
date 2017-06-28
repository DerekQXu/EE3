# EE3-Project-NATCAR
Final project for EE3 - Introduction to Electrical Engineering. Designed and created an autonomous line following robot (black electrical tape on white background), using infrared sensors. Utilized PID algorithm and H-bridge controls to implement path following. Extra Credit: code also lights up LED when magnet detected using hall-effect sensor. 
# Reconfiguration for Micro-Mouse
This code can be reconfigured for a micromouse, a robot with simillar H-bridge and IR sensor layouts, used to traverse mazes.
One would remove anything related to the hall-effect sensors (for speed) and the following sensor fusion line:
```C
pError = (analogRead(REC_CENTER)*((analogRead(REC_LEFT))/100-(analogRead(REC_RIGHT))/100))-baseLine_total;
. . .
err = (analogRead(REC_CENTER)*((analogRead(REC_LEFT))/100-(analogRead(REC_RIGHT))/100))-baseLine_total;
```

The new 'error' and 'previous error', for PID, will be:
```C
pError = analogRead(REC_LEFT) - baseline_l - analogRead(REC_RIGHT) + baseline_r;
. . .
err = analogRead(REC_LEFT) - baseline_l - analogRead(REC_RIGHT) + baseline_r;
```

Note, in order ensure the mouse properly and smoothly in corners, make the following modifications:
```C
int16_t turningFactor;
. . .
turningFactor = map(analogRead(REC_CENTER), 0, 1023, -100, 100);
- correction = weightP * err + weightI * integral + weightD * derivative;
+ correction = turningFactor / 100.0 * (weightP * err + weightI * integral + weightD * derivative);
. . .
```
This code uses diagonal turning. This means that the mouse will go through diagonals whenever required, and turn 90 degrees corners without slowing down much. Regarding the path, the mouse will traverse down straights, and turn whenever it reaches a dead end. Note, after modifications, algorithms, such as floodfill, used to 'solve' the maze are not included. This will up to be left to the specific user to implement.
# Results
The NATCAR successfully navigated a 10 foot straight in ~10 seconds and 3 foot radius loop in ~9 seconds.
The reconfigured micromouse successfully navigated a 16x16 maze.
