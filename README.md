# MiniSegway

## Notes

## 29.03.2024
- 2 Encoders on NUCELO_L432KC still not working, this needs further investigation
- Slowly migrating to NUCLEO_F446RE for Nacht der Technik
- IMU LSM6DS3 is working, but has some spikes in the data, ordered MPU6500
- IMU LSM6DS3 has also a big offset in z-axis, it's actually not the gain thats wrong on this sensor unit

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

```cpp
/**
 * TODO:
 * - adjust LSM6DS3 internal filter settings
 * - move serialStream and rc to MiniSegway
 * - check for all threads the destructor
 *      _Timeout.detach();
 *      _Ticker.detach();
 *      _Thread.terminate();
 * - check for all const functions possible move to header
 * - move all unused destructors to header
 * - check all defines to have the header name first
*/
```

Organise:
- Summarice all the stuff i bought

## Links

### SBus
- Ported Libary: https://github.com/george-hawkins/arduino-sbus
- General Infos: https://github.com/bolderflight/sbus
- Stuff: https://os.mbed.com/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/

### Softserial (not used yet)
- https://os.mbed.com/users/Sissors/code/BufferedSoftSerial/
