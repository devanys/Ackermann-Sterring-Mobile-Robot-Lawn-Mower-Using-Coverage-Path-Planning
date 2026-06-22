# Robot Navigation & Pure Pursuit Control - Mathematical Model

This document outlines the mathematical equations underlying the robot control implementation, including Ackermann steering odometry, the Pure Pursuit path-tracking algorithm, kinematic updates, and actuator mapping used in the Python source code.

---

## 1. Ackermann Steering

The robot utilizes an Ackermann steering geometry, where the rear wheels provide propulsion (tracked via encoders) and the front wheels provide steering. For control and odometry, this is approximated using the kinematic bicycle model.

### a. Rear Wheel Linear Velocity

The linear velocity of each rear wheel is calculated from the encoder RPM, wheel radius $r$, and gear ratio $G$:

$$ v_{left} = \frac{RPM_{left} \cdot 2\pi r}{60 \cdot G} $$

$$ v_{right} = \frac{RPM_{right} \cdot 2\pi r}{60 \cdot G} $$

### b. Robot Linear Velocity

The overall linear velocity $v$ of the vehicle at the center of the rear axle is the average of the rear wheel velocities:

$$ v = \frac{v_{left} + v_{right}}{2} $$

### c. Ackermann Steering Kinematics (Bicycle Model)

In an Ackermann-steered vehicle, the steering angle $\delta$ and the linear velocity $v$ dictate the vehicle's angular velocity $\omega$. Using the bicycle model approximation (where $L$ is the wheelbase):

$$ \omega = \frac{v}{L} \tan(\delta) $$

*Note: True Ackermann geometry requires the inner and outer front wheels to turn at slightly different angles ($\delta_{in}$ and $\delta_{out}$) to prevent tire slip during turns. These are defined geometrically by the turn radius $R$, wheelbase $L$, and track width $W$:*

$$ \delta_{in} = \arctan\left(\frac{L}{R - \frac{W}{2}}\right) $$

$$ \delta_{out} = \arctan\left(\frac{L}{R + \frac{W}{2}}\right) $$

*However, for computational simplicity in the control loop, these are abstracted into a single effective steering angle $\delta$ which is calculated by the Pure Pursuit algorithm and mapped to a single servo motor.*

---

## 2. Pure Pursuit Algorithm

This algorithm calculates the required steering angle $\delta$ to keep the robot following a predefined path.

### a. Lookahead Point Search (Circle-Line Intersection)

The robot searches for a target point (*lookahead point*) on the path that is exactly $L_d$ (*lookahead distance*) away from its current position $(p_x, p_y)$. Mathematically, this is the intersection between a circle of radius $L_d$ centered at the robot and a path segment from $P_1(x_1, y_1)$ to $P_2(x_2, y_2)$.

**Circle Equation:**

$$ (x - p_x)^2 + (y - p_y)^2 = L_d^2 $$

**Parametric Line Equation:**

$$ P(t) = P_1 + t(P_2 - P_1) $$

Substituting the line equation into the circle equation yields a quadratic equation $at^2 + bt + c = 0$, where:

$$ a = dx^2 + dy^2 $$

$$ b = 2(dx(x_1 - p_x) + dy(y_1 - p_y)) $$

$$ c = (x_1 - p_x)^2 + (y_1 - p_y)^2 - L_d^2 $$

With $dx = x_2 - x_1$ and $dy = y_2 - y_1$.

The discriminant is $\Delta = b^2 - 4ac$. If $\Delta \geq 0$, the roots are:

$$ t = \frac{-b \pm \sqrt{\Delta}}{2a} $$

A valid intersection point is chosen where $0 \leq t \leq 1$ (on the segment) and the point lies in front of the robot (dot product with the robot's heading vector $> 0$).

### b. Coordinate Transformation to Robot Frame

The vector from the robot's rear axle $(rear_x, rear_y)$ to the lookahead point $(look_x, look_y)$ is transformed into the robot's local coordinate frame:

$$ \Delta x = look_x - rear_x $$

$$ \Delta y = look_y - rear_y $$

Rotating by the robot's yaw angle $\theta$:

$$ x_f = \Delta x \cos\theta + \Delta y \sin\theta \quad \text{(Forward distance)} $$

$$ y_l = -\Delta x \sin\theta + \Delta y \cos\theta \quad \text{(Lateral distance)} $$

### c. Curvature and Steering Angle Calculation

Using Pure Pursuit geometry, the path curvature $\kappa$ is calculated as:

$$ \kappa = \frac{2y_l}{L_d^2} $$

The required steering angle $\delta$ is then calculated using the bicycle model:

$$ \delta = \arctan(\kappa \cdot L) $$

This angle is then clamped to respect the mechanical limits of the robot:

$$ \delta_{clamped} = \max(-\delta_{max}, \min(\delta_{max}, \delta)) $$

---

## 3. Steering Rate Limiting

To prevent abrupt and mechanical stressful steering movements, the rate of change of the steering angle is limited by $\dot{\delta}_{max}$ over a time step $\Delta t$:

$$ \Delta \delta = \delta_{desired} - \delta_{prev} $$

If $\Delta \delta > \dot{\delta}_{max} \Delta t$:

$$ \delta_{desired} = \delta_{prev} + \dot{\delta}_{max} \Delta t $$

If $\Delta \delta < -\dot{\delta}_{max} \Delta t$:

$$ \delta_{desired} = \delta_{prev} - \dot{\delta}_{max} \Delta t $$

---

## 4. Kinematic Position Update (Dead Reckoning)

The robot's position is updated every time step $\Delta t$ ($DT$) using the kinematic bicycle model.

### a. Simulated Angular Velocity

The simulated angular velocity $\omega_{sim}$ is derived from the linear velocity $v_{sim}$ and the steering angle:

$$ \omega_{sim} = \frac{v_{sim}}{L} \tan(\delta) $$

### b. Position and Orientation Update (Euler Integration)

$$ x_{t+1} = x_t + v_{sim} \cos(\theta_t) \Delta t $$

$$ y_{t+1} = y_t + v_{sim} \sin(\theta_t) \Delta t $$

$$ \theta_{t+1} = \theta_t + \omega_{sim} \Delta t $$

---

## 5. Actuator Mapping (Steering to Servo)

The calculated steering angle is normalized and linearly mapped to the physical limits of the servo motor.

### a. Normalization

$$ \delta_{norm} = \frac{\delta}{\delta_{max}} $$

*(If the steering mechanism is inverted, $\delta_{norm}$ is multiplied by -1).*

### b. Linear Mapping to Servo Limits

$$ \text{Servo Angle} = \text{SERVO\CENTER} + \delta{norm} \times (\text{Limit Offset}) $$

The final value is clamped to ensure it stays within the absolute mechanical bounds:

$$ \text{Servo PWM} = \max(\text{SERVO\LEFT\LIMIT}, \min(\text{SERVO\RIGHT\LIMIT}, \text{Servo Angle})) $$

---

$$ P_{global} = R(\phi) \cdot P_{local} + T $$

Where $T$ is the translation vector representing the robot's global $[x, y]$ position.
