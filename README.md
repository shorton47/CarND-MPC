
# CarND-MPC Controller Project (Term 2 - Project #5)
---

This project was to create a Model Prediction Control (MPC) for **steering angle & throttle control** operating the Udacity Simulator vehicle traveling on the loop track using a 6-element state vector [x, y, psi, v, cte, epsi].  I programmed a MPC controller (using the Udacity template & lessons) to adjust vehicle actuation (steering angle & throttle) in real-time with a realistic latency of command execution.  The process, methods, and results are described below.  I was able to successfully control the vehicle up to 60 mph over multiple loops of the track without hitting any of the guard rails with a first set of cost functions.

[//]: # (Image References)

[image1]: ./outputimage/MPCOutputRunat50Still001.jpg "Result"
[image2]: ./outputimage/MPCOutputRunat50Still002.jpg "Result"
[image3]: ./outputimage/MPCOutputRunat50Still003.jpg "Result"
[image4]: ./outputimage/MPCOutputRunat50Still004.jpg "Result"

[image5]: ./outputimage/MPCOutputRunat50ErrorStill001.jpg "Result"
[image6]: ./outputimage/MPCOutputRunat50ErrorStill002.jpg "Result"




After the MPC controller was working, I obtained the following results for the first part of the track, where with a throttle setting of .5, the car reached at top speed of 50 mph:



| Figure 1 - Vehicle Starting Up    | Figure 2 - Vehicle on first curve | 
| :---:                             |:---:                              |
| ![alt text][image1]               |  ![alt text][image2]              |





Later in the track, a couple of sharp curves are introduced. The first curve has an open rail (leading to a dirt path) and angle corrections must respond quickly. Finally, the last right turn is dramatic and the vehicle comes pretty close to the gaurd rail but does not hit it:






| Figure 3 - Sharp Left Curve            | Figure 4 - Sharp Right Curve    | 
| :---:                                  |:---:                            |
| ![alt text][image3]                    |  ![alt text][image4]            |



It was very helpful that we could plot vehicle paths as part of this project. The Yellow line was the smoothed polynomial fit planned path points (waypoints) in vehicle space and pointing at vehicle angle that came from the Simulator **AND** the green line way the optimal non-linear estimate from the MPC controller based on the given model and cost equations.


---


## Environment to Develop Project

This project was written in C++ on a Macbook Pro with the following configuration:

* macOS Sierra 10.11
* Xcode 8.2.1
* Using uWebsockets for communications w/ Udacity Simulator (v1.4.5)
* uWebsockets needs Openssl 1.0.2
* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* Eigen open source math library

---

## Development of MPC Solution [Updated per Reviewer]

This MPC model uses the global kinematic vehicle model to track the current vehicle location plus error estimates to create actuate predictions by minimizing a global cost function in the presence of a series of constraint equations.

We start with the basic vehicle state of [x,y,ψ,v] which contains x,y position, vehicle angle, and vehicle velocity. Next we augment with the 2 error states of cross track error (cte) and angle error based on the tangential angle of the fitted polynomial of x at time step t. This gives us the 6 element state vector [x,y,ψ,v,cte,eψ] and the model update equations derived in the lecture are:


    xt+1 = xt + vt∗cos(ψt)∗dt
    yt+1 = yt + vt∗sin(ψt)∗dt
    ψt+1 = ψt + (Lf/vt)∗δ∗dt
    vt+1 = vt + at∗dt
    ctet+1 = f(xt)−yt+(vt∗sin(eψt)∗dt)  
    eψt+1 = ψt−ψdest+(Lf/vt)∗δt∗dt)


Here Lf is a constant physical characteristic of the vehicle (front of the car to center of gravity).  Also, added are the 2 car actuations/control inputs [δ,a] which is the delta steering angle adjustment and the throttle which are the parameters that we will estimate and track.  Hence, the final state vector is [x,y,ψ,v,cte,eψ,δ,a]. 

It is also important to note that there are physical constraints on the vehicle control for steering and throttle.  Not only that but vehicles cant and shouldnt make turns greater than certain degrees and the throttle has a maximum acceleration and maximum braking as well.  We will write these as:


    δ=[-25,25] in degrees
    a=[-1,1]  normalized maximum braking (-1) and maximum acceleration (1)


The next step is to convert this into CPP code in MPC class for the automatic differentiator.  For each math function, a matching function has to be overloaded for the automatic differentiator (AD). Overrides for standard math operations +,-,/,* are provided but for the trig functions, we have to specify CppAD:sin() for example*.  Hence, the above equations become:  



```
    // Global Kinematic Model
    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```



The other key to the MPC model is the cost function to minimize. The cost function establishes the behavior of the model during driving - how stable it will be, how smooth the motion will ve, and how it handles fast curves, etc... Starting with the lesson example, I implemented the standard cost equation:


```
    // Reference State Cost
    for (int t = 0; t < N; t++) {
        fg[0] += cte_w * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
      
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
        fg[0] += CppAD::pow(vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
      
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
        fg[0] += delta_w * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```


For my first pass, I have only added weight coefficients for the Delta actuator and the cross-track error (CTE). I will discuss the results of tuning these in a following section.


---

## Main Processing Flow

With the main pieces of the MPC class put together, it was time to get the solution operational by setting up the operational flow of processing the data coming in from the vehicle.  It was quite challenging to figure out the correct flow when a message came in from the vehicle, process in the right order, and perform the correct calculations. As this wasn't fully laid out in the lessons, I just used trial and error to try to get my MPC working. After many attempts, for each message I came up with:


* **1. Load all incoming data from Websocket Message**  
    *         Vehicle data [x, y, psi, v]                      
    *         Planned Path/Waypoints [ptsx & ptsy arrays]
                                               
* **2. Convert Planned Path (Waypoints) from World/Map space to Vehicle space (w/ vehicle angle)**  
    *         Input : ptsx,ptsy,psi,px,py 
    *         Output: ptsx_v,ptsy_v 
                                               
* **3. Fit a smooth polynomial (3rd order) to Planned Path (now in vehicle coordinates)**
    *         Must convert std::vector ptsx,ptsy into Eigen::VectorXd for polyfit
    
* **4. Calculate & update errors (cross-track & steering angle) for MPC model solver**
    *         cte & espi are relative from vehicle to the lane centerline
    
* **5. Solve Model Predicition Control (MPC) w/ current state vector & coefficients**
    *         Input : state, coeffs 
    *         Output: vars that have predicted actuations (steering angle, throttle)
    
* **6. Display & store results and send message back to Simulator Server**


---

## Latency [Updated per Reviewer] 

In order to create a more realistic vehicle simulation, a latency should be introduced between the time that the actuation parameters are estimated from the MPC solver, and the time the vehicle physically actuates the device. On a modern PC or laptop, these commands are virtually instantaneous relative to the actual physics of executing electrical commands and physical actuation of devices on a car.

Per suggestion of the rubric, I introduced a 100 millisecond latency delay or thread sleep on the main thread just after calculation and just before the message is sent back to the server on line 247 of main.ccp: 

```
    // Latency - the purpose is to mimic real driving conditions where the car does NOT actuate the commands
    // instantly. This value should be set to drive around the track with 100ms latency.
    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
    this_thread::sleep_for(chrono::milliseconds(100));
```

I experimented with the latency on and off and it certainly made a difference. It was harder for the model to respond optimally to quick changes in the road conditions with the latency set at 100 ms. For all of the tuning exercises below and for submittal, I set the latency to 100 ms. 


---

## MPC Hyper-parameters, Tuning & Results 

I was able to get my MPC controller operational using the following process to iteratively improve it's performance:

* Start with default N,dt settings from previous starter lesson (N=75, dt=0.05)
* Used very slow (.01), fixed throttle and just worked on getting steering angle estimates that were reasonable
* Wired-up throttle with the v_ref set very low
* Then, started to increase v_ref to see what instabilities and poor behavior was occuring.



### N & dt [Updated per Reviewer]

The second phase of tuning was to now look at N and dt and set those to more optimized values. The yellow and green lines plotting out in the the Simulator were very helpful in this step! You could directly see the behavior of setting different values.

Essentially, N & dt together create a time "look-ahead" buffer of predicted steps that the MPC solver will use to calculate the actuation estimates. I quickly saw that with N=75 & dt=0.05, that created a N*dt=3.75 second look-ahead estimation that was **too far ahead** for stable behavior*. This was so far ahead based on reasonable speeds, that when the car reached the curves, the polynomial started to contort in very bad, overley curvey paths that sent the car flying off the road at almost any speed.  I also tried very short look ahead times of <.25 seconds and this also did a poor job of preparing the car for sharp turns at any rates of speed over 10 mph.  Hence, I now had a range [.25, 3.0] of N/*dt values to try to find an optimal balance*.  

I wanted my sampling rate to be faster than the latency delay by 3-5x (rule of thumb from control theory) so I wanted dt to have a fine resolution of .02 to .03. I started with N=50 and noticed that I was still getting some spurious wiggly behavior in the polynomial estimate. Also an N of this size is computationally more expensive (on some hardware) so I cut it in half to N=25. This gave a time buffer "look-ahead" of N*dt of 25*.03=.75 seconds.  This seemed to be a good balance of performance, computational expense, and resolution and the vehicle was staying on the track up to 30-50 mph. 


### Cost Equations Weights Tuning
The final phase of tuning that I did was to look at the cost equations to improve the overall behavior of the model and see how I could improve the results.  Based on hints in the lessons, I looked at the weight on the cte and the delta actuation. The cte weight was very sensitive and I saw a little improvement increasing this to 2x but after that, it seemed to get worse.  When I changed the delta weight (to minimize jumpiness in actuation or jerk), I saw continually improvements to up over 500,000! I finally settled on 600,000 and the vehicle performed well up to 60 mph.      

My final hyper-parameters:

* N=25
* dt=0.03
* cte_w = 2.0
* delta_w = 600000.0


I note that the car can handle much greater speeds on the straightaways and slight curves and the constraint in my current implementation is that sharp curves need to be handled as more of a special case.  I belive that car controlled smoothly up to 60 mph and could go as high as 80 mph before seriously flying off the track.

---


## Other Notes on this Project

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).  This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./mpc

Note! The files that are needed to compile this project are:

Source:
* src/main.cpp
* src/MPC.cpp

Header:
* src/MPC.h
* src/json.hpp
* src/utils.hpp


---
## Further Developments!

I see that more creative things can be done with the cost functions to further optimize performance. I would like to experiment further with faster throttle on everything but the sharp curves and reduce speed for the sharp curves more. 

Second, I still see some slight ripple at higher speeds even on the straightaways and so it would be good to investigate this and see if I could minimize that while keeping good performance everywhere else. 


---
## Data Glitch??

I believe there is a glitch in the Simulator at one specific point. I noticed in the last project one point just after the vehicle comes over the bridge and is about to execute a left turn but I did not have time to explore. This glitch in the data appeared, even more pronounced in this project, when one displays 



| Figure 1 - Vehicle (Normal) Just Over The Bridge | Figure 2 - Vehicle ONE FRAME LATER has big Waypoint Error | 
| :---:                                            |:---:                                                      |
| ![alt text][image5]                              |  ![alt text][image6]                                      |


It turns out that with enough points, one bad point doesnt perturb the result but when I lowered the number of sampling points, it caused problems. Of course, in real life, I am sure this is something that comes up all the time. Were we supposed to find this? I think I did!!


Thanks!  Steve


```python

```
