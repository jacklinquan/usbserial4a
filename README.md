# usbserial4a
[![PyPI version](https://badge.fury.io/py/usbserial4a.svg)](https://badge.fury.io/py/usbserial4a) [![Downloads](https://pepy.tech/badge/usbserial4a)](https://pepy.tech/project/usbserial4a)

Python package for Kivy Android USB serial port.

Please try the Android App built with usbserial4a on Google Play: [PyTool USB Serial Free](https://play.google.com/store/apps/details?id=com.quanlin.pytoolusbserialfree), [PyTool Modbus Free](https://play.google.com/store/apps/details?id=com.quanlin.pytoolmodbusfree).

Implemented drivers are listed below:
* FTDI serial driver - done and tested with FT230X.
* CDC ACM serial driver - done and tested with MCP2200.
* CP210x serial driver - done and tested with CP2102.
* CH34x serial driver - done and tested with CH340.
* PL2303 serial driver - done and tested with PL2303.

Please consider [![Paypal Donate](https://github.com/jacklinquan/images/blob/master/paypal_donate_button_200x80.png)](https://www.paypal.me/jacklinquan) to support me.

## How to use it:
**To make quick prototype or to test and debug the script before building an App:**

It works on Android 6.0+.

If there is any usb serial device, connect it to the Android phone/tablet through USB OTG cable (It's needed for Android USB host function). 

Get Pydroid Apps from [here](https://github.com/jacklinquan/Pydroid_Apks), or get the latest versions on [Google Play](https://play.google.com/store/apps).

In Pydroid, go to `Menu->Pip` and install `usbserial4a`.

Or go to `Menu->Terminal` and enter `pip install usbserial4a`.

Open `example.py` and run it. When it runs for the first time, it might prompt you for permission to access the USB device. Accept the permission and run this script again, then it should send the data `b'Hello world!'` as expected.

Go to `Menu->Graphical program output`.

Scroll to the last line, it should list all the USB devices connected to the Android phone/tablet with vendor id, vendor name, product id and product name.

**To build dedicated Apps with buildozer:**

It works on Android 4.0+.

In `buildozer.spec` add `termios.so` to the whitelist.

Include `pyserial`, `usb4a` and `usbserial4a` in requirements.

Add `intent-filter.xml`.

```
# (list) python-for-android whitelist
android.p4a_whitelist = lib-dynload/termios.so

# (list) Application requirements
# comma seperated e.g. requirements = sqlite3,kivy
requirements = kivy, pyjnius, pyserial, usb4a, usbserial4a

# (str) XML file to include as an intent filters in <activity> tag
android.manifest.intent_filters = intent-filter.xml 
```

Build the project for the first time and it will fail with an error as expected.

`buildozer android debug`

In the generated  `.buildozer` folder find a `res` folder like this one:

`.buildozer/android/platform/build/dists/YOU_PROJECT_NAME/src/main/res`

Create a `xml` folder in `res` folder.

Add `device_filter.xml` to this `res/xml/` folder.

Find a manifest template file like this one:

`.buildozer/android/platform/build/dists/YOUR_PROJECT_NAME/templates/AndroidManifest.tmpl.xml`

Add  `<uses-feature android:name="android.hardware.usb.host" />`  to this `AndroidManifest.tmpl.xml` at a good position.

Build the project again and it should pass.

The serial port class extends pyserial's SerialBase class. So it can be used in a similar way to serial.Serial from pyserial.
