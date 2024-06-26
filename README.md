﻿# ESP32 Biosignal Monitor

This project is a physiological signal acquisition system developed using ESP32-­WROOM-­32E, ADS1293, and AD5941. It is designed to collect various physiological signals, such as electrocardiograms (ECG) and electrodermal activity (EDA), and transmit them wirelessly using websockets. It also can calculate the heart rate by extracting the R peaks from the ECG signal. It employed notch filters and wavelet filters for ECG signal denoising. 

## Introduction
### ECG Signal Denoising
![](img/PixPin_2024-03-29_10-38-19.png)
![](img/PixPin_2024-03-29_10-38-29.png)

### Heart Rate Extraction
![](img/PixPin_2024-03-29_10-37-46.png)

### Schematic Diagram
![Schematic Diagram](img/PixPin_2024-03-29_10-26-49.png)

### Circuit Diagram
![Circuit Diagram](img/PixPin_2024-03-29_10-18-46.png)

### Shell Design
![Shell design](img/PixPin_2024-03-29_10-13-16.png)

### Physical Prototype
![Physical Prototype](img/PixPin_2024-03-29_10-14-32.png)



## Usage

![Connection Method](img/PixPin_2024-03-29_10-20-07.png)


## Acknowledgements
This project makes use of the following open-source libraries:

+ [esp32-websocket](https://github.com/Molorius/esp32-websocket) - A WebSocket library for the ESP32, enabling efficient real-time communication for IoT devices. This library forms the backbone of our system's wireless data transmission capabilities.
+ [wavelib](https://github.com/rafat/wavelib) - A Wavelet Transform library that provides various signal processing functions that help denoise the physiological signals collected by our system.

Special thanks to the authors and contributors of these projects for their valuable work, which significantly enhanced the functionality and performance of our physiological signal acquisition system.
