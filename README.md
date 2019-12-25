# ArduinoXVC

This project implements a bare-bones Xilinx Virtual Cable server on an Arduino Uno / Leonardo, based on the specification posted in [Xilinx/XilinxVirtualCable](https://github.com/Xilinx/XilinxVirtualCable).

## Hardware
Make sure that the device's JTAG output-high voltage does not exceed the Arduino's maximum input voltage!

Connect your Arduino Uno to the JTAG port of the target device:

| Arduino Pin     | JTAG Pin |
|-----------------|----------|
| `2` (OD output) | `TCK`    |
| `3` (OD output) | `TMS`    |
| `4` (OD output) | `TDI`    |
| `5` (input)     | `TDO`    |
| `GND`           | `GND`    |

The Arduino's pins are configured as *open drain* outputs to automatically adjust to the target's JTAG I/O voltage. External pull-up resistors aren't required, as Xilinx PLDs supply them internally.

## Software
So far, this project has been tested successfully using **ISE 14.7** and **iMPACT**.

An additional program is required to translate the serial port to a TCP socket, e.g. [pyserial: tcp_serial_redirect](https://github.com/pyserial/pyserial/blob/master/examples/tcp_serial_redirect.py).

1. Start the tcp-to-serial bridge application to serve the Arduino's Serial VCP to `localhost:2542`. Use baudrate `115200` and `8N1` mode.

```
python tcp_serial_redirect -P 2542 COMx 115200
```

2. Open iMPACT and start the *Boundary Scan* flow
3. Open the *Cable Setup* dialog (either via right-click or the *Output* menu)
4. Mark the checkbox under *Cable Plug-In*
5. Enter the following into the combobox below:

```
xilinx_xvc host=127.0.0.1:2542 disableversioncheck=true
```

6. The PLD should now show up and be identified correctly when you run *Initialize Chain*.
