# Simple GPS Logger

Log GPS position every fifteen minutes.

Wire up an OpenLog to UART5 for SD logging.

Electron | OpenLog
-------- | -------
3v3      | Vcc
GND      | GND
C1       | RX
C0       | TX

I've been uploading using the included build script and putting the Electron in DFU mode by holding both buttons, releasing the Mode button and then waiting for the yellow flashing LED.
