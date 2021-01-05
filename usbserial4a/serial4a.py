"""Android USB host serial port.

Functions:
get_serial_port
"""

from usb4a import usb
from .vidpid4a import (
    FTDI_VENDOR_ID,
    SILABS_VENDOR_ID,
    QINHENG_VENDOR_ID,
    PROLIFIC_VENDOR_ID,
    FTDI_VID_PID_GROUP,
    SILABS_VID_PID_GROUP,
    QINHENG_VID_PID_GROUP,
    PROLIFIC_VID_PID_GROUP,
)
from .ftdiserial4a import FtdiSerial
from .cdcacmserial4a import CdcAcmSerial
from .cp210xserial4a import Cp210xSerial
from .ch34xserial4a import Ch34xSerial
from .pl2303serial4a import Pl2303Serial


def get_serial_port(device_name, *args, **kwargs):
    """Get a USB serial port from the system.

    Parameters:
        device_name (str): the name of the USB device.

    Returns:
        USB serial port: an object representing the USB serial port.
    """
    device = usb.get_usb_device(device_name)
    if device:
        device_vid = device.getVendorId()
        device_pid = device.getProductId()
        if (
            device_vid == FTDI_VENDOR_ID
            or (device_vid, device_pid) in FTDI_VID_PID_GROUP
        ):
            return FtdiSerial(device_name, *args, **kwargs)
        elif (
            device_vid == SILABS_VENDOR_ID
            or (device_vid, device_pid) in SILABS_VID_PID_GROUP
        ):
            return Cp210xSerial(device_name, *args, **kwargs)
        elif (
            device_vid == QINHENG_VENDOR_ID
            or (device_vid, device_pid) in QINHENG_VID_PID_GROUP
        ):
            return Ch34xSerial(device_name, *args, **kwargs)
        elif (
            device_vid == PROLIFIC_VENDOR_ID
            or (device_vid, device_pid) in PROLIFIC_VID_PID_GROUP
        ):
            return Pl2303Serial(device_name, *args, **kwargs)
        else:
            return CdcAcmSerial(device_name, *args, **kwargs)
    else:
        raise usb.USBError("Device does not exist!")
