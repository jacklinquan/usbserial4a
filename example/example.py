# When this script is run for the first time, it might prompt you for 
# permission. Accept the permission and run this script again, then it should 
# send the data as expected.

# Kivy is needed for pyjnius behind the scene.
import kivy
from usb4a import usb
from usbserial4a import serial4a
from pprint import pprint

usb_device_list = usb.get_usb_device_list()
usb_device_name_list = [device.getDeviceName() for device in usb_device_list]
usb_device_dict = {
    device.getDeviceName():[
        device.getVendorId(), 
        device.getManufacturerName(),
        device.getProductId(),
        device.getProductName()
        ] for device in usb_device_list
    }
pprint(usb_device_dict)

if usb_device_list:
    serial_port = serial4a.get_serial_port(
        usb_device_list[0].getDeviceName(), 
        9600, 
        8, 
        'N', 
        1)
    if serial_port and serial_port.is_open:
        serial_port.write(b'Hello world!')
        serial_port.close()
