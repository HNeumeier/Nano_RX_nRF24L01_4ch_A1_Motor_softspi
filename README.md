# Nano_RX_nRF24L01_4ch_A1_Motor_softspi

Many thanks to Stanek for the receiver code

https://github.com/stanekTM/RX_nRF24L01_Telemetry_Motor_Servo

I have modified a old Tank Modell and replaced all the Electronics now i'am using a Arduino NANO with NRF24L01
also the Module L298 2x Modules for the left and right Motors each Module parrallel to get 4A for each Motor
1x Module for the Tower to rotate 360 Degree and for the Shooter Motor.

The RF24 library is copied from the Original to My_RF24 that i can define the soft_SPI Parameter without to change the Original library.

This code is used wit a Taranis X7 the mix of the Motors are done in the Transmitter

![Tank](https://user-images.githubusercontent.com/33512064/217525413-fd9777ff-e5b9-4376-b860-d7552625eb4c.jpg)
