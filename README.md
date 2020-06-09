# Bitcoin Miner x Brushless Motor Control

## Background

Using a NUCLEO-F303K8 controlling a brushless motor, we can investigate real time control algorithms and their efficiency. A Bitcoin miner was implemented as a proof of work , such that the hashrate of the system can be used to quantitatively evaluate performance. The overall implementation of the code uses a total of three threads for incoming communication, outgoing communication and motor control. There is an ISR for reading from the serial port and and an ISR for the photointerrupters.


## Components and Constraints

Nucleo-F303K8: The microcontroller used. 74MHz clock speed max, 64KB Flash, 16KB StaticRAM

Brushless Motor: Brushless motors use pulses of current through motor windings to control the torque and velocity of the motor. To control the torque. We modulate the motor current using PWM.

## Performance

The brushless motor performed well within the limits of precision expected. Efficient code enabled responsive control when setting commands to carry out a certain number of rotations at a defined velocity, where overshoot was the measured metric of performance. In other words, the less the motor overshoots/undershoots the better it performs.  

![](https://github.com/RajanPatel97/Bitcoin-Miner-Brushless-Motor-Controller/blob/master/assets/brushlessMotor.png)
