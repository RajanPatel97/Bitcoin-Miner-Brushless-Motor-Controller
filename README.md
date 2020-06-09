# Bitcoin Miner x Brushless Motor Control

Using a NUCLEO-F303K8 controlling a brushless motor, we can investigate real time
control algorithms and their efficiency. A Bitcoin miner has been implemented as a
proof of work , such that the hashrate of the system can be used to quantitatively
evaluate performance. The overall implementation of the code uses a total of three
threads for incoming communication, outgoing communication and motor control.
There is an ISR for reading from the serial port and and an ISR for the
photointerrupters
