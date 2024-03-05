# MiniSegway

## Notes

### 07.03.2024
- Finding a pin that is working for PPM (TBS CROSSFIRE Nano RX V1) was a bit a pain, but the actual pin layout is working for PPM and should work for all the necessary hardware, see ***config.h***.

### 05.03.2024

- NUCLEO_L432KC does not support ``"target.printf_lib": "std",``
- TBS CROSSFIRE Nano RX V3 does not support PPM anymore $\rightarrow$ SBUS implementation needed

## TODO

Organise:
- Stecker

Implement:
- reset function for SerialStream, this requires some change in the logic part
- implement motor control
- implement imu
- adjust imu internal filters
- implement filter for rc smoothing with reseting
- SBUS protocol (inverted SBUS via ELRS)