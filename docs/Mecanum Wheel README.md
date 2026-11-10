### Mecanum Information
To provide this range of movement, Icarus is required to calculate individual wheel rotation velocities based on chassis velocity command.  The wheel velocities are calculated using the Mecanum Inverse kinematic formula as shown below.

$$\left(\begin{matrix}
\omega_{fl} &= \frac{1}{r} \left[v_x - v_y - (l_x + l_y)\omega_z \right ] \\ 
\omega_{fr} &= \frac{1}{r} \left[v_x + v_y + (l_x + l_y)\omega_z \right ] \\ 
\omega_{rl} &= \frac{1}{r} \left[v_x + v_y - (l_x + l_y)\omega_z \right ] \\ 
\omega_{rr} &= \frac{1}{r} \left[v_x - v_y + (l_x + l_y)\omega_z \right ] 
\end{matrix}\right)$$


The robot also publishes the Odometry of its movement, so it must calculate its actual speed and position.  Using the motor encoders to find wheel velocity, we use the Mecanum Kinematic Formula shown below to find its chassis velocity.

$$\left(\begin{matrix}
v_x & = (\omega_{fl} + \omega_{fr} + \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4}\\ 
v_y & = (-\omega_{fl} + \omega_{fr} + \omega_{rl} - \omega_{rr}) \cdot \frac{r}{4}\\ 
\omega_z & = (-\omega_{fl} + \omega_{fr} - \omega_{rl} + \omega_{rr}) \cdot \frac{r}{4(l_x + l_y)}
\end{matrix}\right)$$

These equations and diagrams were sourced from the Ecam Eurobot, which can be found under references on the repo main page.
