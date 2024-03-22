# MiniSegway

## Notes

## 10.03.2024
- Implemented SBus protocol for ELRS
- Currently it looks like the NUCLEO_L432KC can only have one uart, which means i can either use SBus or OpenLager... need to double check with someone who knows more about this. Otherwise i need to switch to a NUCLEO_F446RE.

### 07.03.2024
- Finding a pin that is working for PPM (TBS CROSSFIRE Nano RX V1) was a bit a pain, but the actual pin layout is working for PPM and should work for all the necessary hardware, see ***config.h***.

### 05.03.2024

- NUCLEO_L432KC does not support ``"target.printf_lib": "std",``
- TBS CROSSFIRE Nano RX V3 does not support PPM anymore $\rightarrow$ SBUS implementation needed

## TODO

Coding stuff:

```
/**
 * TODO:
 * - move serialStream and rc to MiniSegway and remove option of SBus as thread
 * 
 * - check for all threads the destructor
 *  _Timeout.detach();
    _Ticker.detach();
    _Thread.terminate();
*
* - check for all const functions possible move to header
*
* - move all unused destructors to header
*
* - check all defines to have the header name first
*
* - check serial stream and usage of serial pipe and 
*   buffered serial again according to sbus
*/
```

Organise:
- ...

Implement:
- implement motor control
- implement imu
- adjust imu internal filters
- implement filter for rc smoothing with reseting

## Links

### SBus
- Ported Libary: https://github.com/george-hawkins/arduino-sbus
- General Infos: https://github.com/bolderflight/sbus
- Stuff: https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/

### Softserial
- https://os.mbed.com/users/Sissors/code/BufferedSoftSerial/
