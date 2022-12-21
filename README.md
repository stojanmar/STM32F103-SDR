# STM32F103-SDR
Stand alone SDR with STM32F103 Bluepill board without audio codec.
This is a proof, what can be achieved with simple and cheap materials to build an SDR based receiver.
The goal was to build radio without using audio bords containing a usual codec chips. Namelly this impacts the assembly cost evidently.
For the SDR front end a standard purchased mixer with I and Q outputs is used and for the audio output a modulated PWM is simply filtered through RC couple of elements
and further connected to audio amplifier.
Special care to avoid unwanted noise comming from supportive circuits was taken. For example a power supply does not include any switching ICs.
I an Q signals are captured using two ADC channels configured to highes possible speed. 
In the digital filtering where the Hilbert coefitients have been used, a special trick to reduce the number of multiplications in the convolution  process has been reduced, so the smaller computational power of the selected microcontroller can bring acceptable result.
For the interface a Nextion display communicates everithing through RS232 port.
The approach was very basic and can deffinitely be improved. 
Currently this work has no further improovement plan at least at my side.
The link to working protoype :
https://www.youtube.com/watch?v=-c7fZPpSIDY&ab_channel=stojanmarkic
