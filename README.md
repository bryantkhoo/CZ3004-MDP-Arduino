# CZ3004-MDP-Arduino
Code used for our Arduino robot in our last leaderboard run. This is the combined effort of our Arduino team!

<h2>Hardware Components</h2>
1x Arduino Uno R3 DEV‐01080
1x PROTO Arduino Uno shield
1x Ardumoto ‐ Motor Driver Shield VHN5019
1x Sharp GP2Y0A02YK IR Sensor (Long range)
5x Sharp GP2Y0A21YK IR Sensor (Short range)
2x Gearmotors w/ encoders
1x Battery, SLA 6V, TLA633
<h2>Assembled Robot</h2>
<p align="center">
  <img src="images/robot front.jpeg" width="350"/>
  <img src="images/robot top.jpeg" width="350"/>
</p>
<h2><strong><strong>Sensors</strong></strong></h2>
We utilise a total of <strong>5 Short-Range (SR)</strong> IR sensors and <strong>1 Long-Range (LR)</strong> IR sensor.
<h3>Sensor Readings &amp; Feedback</h3>
We take the mean of 10 raw sensor readings for our sensor readings.

The <strong><em>getDistanceOutput</em></strong> function allows us to get feedback on the distance between the robot and its obstacles based on amount of grids. We manually test the sensor ranges of each grid, taking into account of the fluctuation of the sensor's reading and the motor's movement.

From the example code below, if the mean is between 1 to 16 cm, Arduino will be sending a <strong>feedback of "1"</strong> back to the Algorithm, indicating that there is <strong>an obstacle in the 1st grid.</strong> For average between 16 to 27.5cm, a <strong>feedback of  "2"</strong>  is sent back to the Algorithm, indicating an <strong>obstacle in the 2nd grid.</strong> Since any readings <strong>beyond that of 2 grids are inaccurate</strong>, "-1" will be feedback to indicate no obstacle within a 2 grid range.
<pre>String getDistanceOutput(int sensor) {
...
    if (sensor == 1) {
        // Get the sum of 10 values
        for (int i = 0; i &lt; 10; i++) {
            sum = sum + sr1.distance();
        }
        average = sum / 10;
       
        if (1 &lt;= average &amp;&amp; average &lt;= 16) { return "1"; }
        else if (average &gt;= 16 &amp;&amp; average &lt;= 27.5) { return "2"; }
        else { return "-1"; }
    }
...</pre>
<h4><strong>Sensor Position on Top Plate</strong></h4>
On the <strong>top plate</strong>, we have placed <strong>3 SR sensors</strong> facing the <strong>front</strong> and one <strong>LR sensor</strong> towards the <strong>left</strong> as shown in the diagram below.

The front sensors are able to determine if the grids in blue (1 or 2 grids away) are <strong>free from obstacles</strong>. Further distances cannot be determined consistently, and the Arduino will return "-1" for those values.

The Left LR sensor is able to consistently determine if there are obstacles 3 or 4 grids away from the robot. Grids 1 and 2 are bad ranges for the LR sensor, and we return 0 for any distance in that range. We return -1 for any distances further than 4 grids.

<a href="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.37.53-AM-2azw50t.png"><img class="wp-image-194 aligncenter" src="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.37.53-AM-2azw50t.png" alt="" width="556" height="309" /></a>
<h4><strong>Sensor Position on Bottom Plate</strong></h4>
On the <strong>bottom plate</strong>, we place<strong> 2 SR sensors</strong> pointing <strong>left and right</strong> as per the diagram below. They able to determine if the grids in blue (1, 2 grids away) are free from obstacles. Further distances cannot be determined consistently, and the Arduino will return "-1" for those values.

<a href="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.47.27-AM-18ch3rg.png"><img class="wp-image-195 aligncenter" src="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.47.27-AM-18ch3rg.png" alt="" width="558" height="312" /></a>
<h3>Overall Sensor Readings</h3>
<h3><a href="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.52.49-AM-28v56ha.png"><img class="wp-image-196 aligncenter" src="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-30-at-2.52.49-AM-28v56ha.png" alt="" width="558" height="313" /></a></h3>
<h2><strong>Movement Control</strong></h2>
<h3><strong>Application of PID</strong></h3>
The code below shows the computation of the PID and the relationship between K1, KP, KI, KD and K2, KP, KD and K3, KD which are obtained from the lab document titled <em>Implementing PID Controller for 2-wheel drive robots.</em>
<pre>void PIDCompute(float KP_ML, float KI_ML, float KD_ML, float KP_MR, float KI_MR, float KD_MR,double Setpoint) {
 error_ML = (Setpoint - Input_ML) / 130.0;
 ...
 // Compute k1, k2, k3 based on kp, ki, kd
 double k1_ML = KP_ML + KI_ML + KD_ML;
 double k2_ML = -KP_ML - 2 * KD_ML;
 double k3_ML = KD_ML;
 ...
 // Compute PID Output
 Output_ML = pre_Input_ML + k1_ML * error_ML + k2_ML * lastErr_ML + k3_ML * secLastErr_ML;
 ...
 // Record of the past errors to ensure a more stable PID output
 secLastErr_ML = lastErr_ML;
 lastErr_ML = error_ML;
 pre_Input_ML = Output_ML;
 ...
}</pre>
We configured our <em>Setpoint</em> to our target RPM of 62, and used Arduino's serial plotter to observe the RPM of both motors.

Plotting RPM values against time, the blue and red graphs represent the RPM of our left and right motors respectively.  From the graph, we observe that both motors are able to accelerate to 62 RPM and remain stable within a desirable time period.
<strong><a href="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-27-at-8.05.32-pm-21x3e4b.png">
<img class="wp-image-64 aligncenter" src="https://blogs.ntu.edu.sg/scemdp-201617s2-g02/files/2017/03/Screen-Shot-2017-03-27-at-2.01.24-AM-s50s35.png" alt="" width="441" height="308" /></a></strong>

In order to derive at our KP, KI and KD values, we tuned the values through trial and error, ensuring that we are able to obtain a stable output with the serial plotter. The code below shows our KP, KI and KD values for each motor. The left motor and right motor are represented by ML and MR respectively.
<pre>float KP_ML = 4;  float KI_ML = 0.27;  float KD_ML = 5;</pre>
<h3><strong><strong>Moving Straight</strong></strong></h3>
<em>goStraightOneTime()</em> calls <em>PIDCompute()</em> to obtain the PID output, which is then multiplied by our preset speed. The updated speed is used to stabilise the output of our motors<i>, </i>ensuring that our robot is able to move in a straight line.

<em>goStraightOneGrid()</em> calls <em>goStraightOneTime()</em> and accumulates the RPM readings of both motors in <em>totalDis, </em>and brakes the robot once the <em>totalDis</em> has reached our pre-configured value. The brake condition is adjusted based on our distance requirement, from 1 grid (10 cm) to 10 grids (100 cm).
<pre>int goStraightOneGrid(long value) {
    while (1) {
        if (totalDis &gt;= value) 
            // Reset totalDis and brakes motor
        else 
            // Call goStraightOneTime() and accumulate total RPM
    }
}</pre>
<h4><strong>Turning Left / Right</strong></h4>
Similar to the straight movement, the <em>left/right</em> function is based on a pre-defined speed, with the only difference being the following:
<ul>
 	<li>Left: Set negative Left motor speed, positive Right motor speed</li>
 	<li>Right: Set negative Right motor speed, positive Left motor speed</li>
</ul>
<h3>CALIBRATION IMPLEMENTATION</h3>
The Arduino calls CaliAngle() followed by Calibrate() when a instruction to calibrate is received.

Once the<strong> </strong><em><strong>CaliAngle(</strong>)</em> function is called by the algorithm, this function will compare the distance between the front left and right short range sensors and move individual motors front or back based on 4 conditions, until the difference between both sensors is less than 0.15.
<pre>// Pseudocode for Calibrating Angle
CaliAngle(){
    targetDist = 10.5;
    while(Distance of both sensors &lt; 30) {
        Calculate Difference between L and R sensor reading
        if(0.15 &lt;= Difference &lt;= 0.15){
            if (0.15 &lt;= Difference &lt;= 7 &amp;&amp; Left &gt;= targetDist) 
                Move Left Motor forward
            else if (-0.15 &lt;= Difference &lt;= -7 &amp;&amp; Left &lt; targetDist)
                Move Left Motor Backwards
            else if (0.15 &lt;= Difference &lt;= 7 &amp;&amp; Left &lt; targetDist)
                Move Right Motor Backwards
            else if (-0.15 &lt;= Difference &lt;= -7 &amp;&amp; Left &lt; targetDist)
                Move Right Motor Forward
        } else {
            Brake Motors
            break;
        }
    }
}</pre>
<h4><strong>Calibrate Distance</strong></h4>
Once <strong><em>CaliAngle() </em></strong>has completed and the robot is straight, <strong><em>calibrate()</em></strong> will be called and the robot will either move forward or backwards until it reaches its target distance of between 10.65 to 11 cm.
<pre>void calibrate() {
   // Declare initial speed
   while (getDistance(1) &lt; 30 &amp;&amp; getDistance(2) &lt; 30) {
       if ((getDistance(1) &gt;= 10.65 &amp;&amp; getDistance(1) &lt; 11) || (getDistance(2) &gt;= 10.65 &amp;&amp; getDistance(2) &lt; 11)) {
           // Stop the motor and exit the while loop
       } else if (getDistance(1) &lt; 10.65 || getDistance(2) &lt; 10.65) {
           // Move robot backward
       } else {
          // Move robot forward
       }
   }
   // Stop the motor
}</pre>
