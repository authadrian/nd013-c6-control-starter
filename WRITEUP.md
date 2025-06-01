## Carla Screenshots - Step 1

![Step 1](./images_for_writeup/Step1_Carla.png "Car is not moving.")

Step 1 works as expected. Code compiles and runs successfully. Car is not moving.

## Step 2 - Throttle PID tuning
error during tuning runs saved in throttle_pid_data.txt and steer_pid_data.txt are saved in "./project/tuning_run_logs"

### Run 1
I did run the the Throttle PID with conservative parameters: kpt = 0.1, kit = 0.00, kdt = 0.01

I have seen:
- consistent steady-state error around 1.8-2.0
- no oscillations
- throttle output values around 0.2, around 10% of the error

Next:
- try increasing kpt (to 0.15) as my proportional gain is clearly too low. The ego does not reach target speeds.
- try adding small kit (0.01) to accumulate and eliminate steady-state error over time.
- keeping kdt at 0.01 as there were no oscillations.

### Run 2

...


## Step 3 - Steer PID tuning

### Run 1
I did run the the Steer PID with conservative parameters: kps = 0.25, kis = 0.00, kds = 0.01

I have seen:
-

Next:
-

### Run 2

...

## Plot the results of the simulation.
_The plots are provided in the report._

{add the plots printed. Throttle + Steer. These are AFTER tuning = with the best parameters}
{errors are saved for the SAME version in the .txt files - check it}

## Analyzing a plot.

_A few lines explaining the plots and describing what is plotted: the variables, the phenomenon shown._

## Recognize the action of each part of the PID controller.
_What is the effect of the PID according to the plots, how each part of the PID affects the control command?_
_Explain the different parts of the PID. (a.k.a. P / I / D )_

## critical analysis of the controller.

_How would you design a way to automatically tune the PID parameters? (This is an open question, the coherence and justification of the answer is valued.)_

{answer here}

_PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller? Find at least 2 pros and cons for model free versus model based._

{answer here}