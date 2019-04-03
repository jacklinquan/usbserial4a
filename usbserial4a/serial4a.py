'''Android USB host serial port.

Functions:
get_serial_port
'''

from usb4a import usb
from .ftdiserial4a import FtdiSerial
from .cdcacmserial4a import CdcAcmSerial
from .cp210xserial4a import Cp210xSerial

FTDI_VENDOR_ID = 0x0403
SILABS_VENDOR_ID = 0x10C4
VENDOR_IDS = {
    'ftdi': FTDI_VENDOR_ID,
    'silabs': SILABS_VENDOR_ID
}
PRODUCT_IDS = {
    FTDI_VENDOR_ID: {
        'ft232': 0x6001,
        'ft232r': 0x6001,
        'ft232h': 0x6014,
        'ft2232': 0x6010,
        'ft2232d': 0x6010,
        'ft2232h': 0x6010,
        'ft4232': 0x6011,
        'ft4232h': 0x6011,
        'ft230x': 0x6015
    },
    SILABS_VENDOR_ID: {
        'cp2102': 0xEA60
    }
}

def get_serial_port(device_name, *args, **kwargs):
    '''Get a USB serial port from the system.
    
    The parameters are compatible with serial.Serial from pyserial.
    The class of the returned object extends SerialBase from pyserial.
    
    Parameters:
        device_name (str): the name of the USB device.
    
    Returns:
        USB serial port: an object representing the USB serial port.  
    '''
    device = usb.get_usb_device(device_name)
    if device:
        device_vid = device.getVendorId()
        if device_vid == VENDOR_IDS['ftdi']:
            return FtdiSerial(device_name, *args, **kwargs)
        elif device_vid == VENDOR_IDS['silabs']:
            return Cp210xSerial(device_name, *args, **kwargs)
        else:
            return CdcAcmSerial(device_name, *args, **kwargs)
    else:
        raise usb.USBError('Device does not exist!')
    
    