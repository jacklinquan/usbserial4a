"""Android USB serial PL2303 driver.

Classes:
Pl2303Serial
"""

from struct import pack, unpack
import time
from .utilserial4a import (
    SerialBase,
    SerialException,
    to_bytes,
    PortNotOpenError,
    Timeout,
)
from usb4a import usb


class Pl2303Serial(SerialBase):
    """PL2303 serial port class."""

    # Default baudrate
    DEFAULT_BAUDRATE = 9600

    USB_RECIP_INTERFACE = 0x01

    PROLIFIC_VENDOR_READ_REQUEST = 0x01
    PROLIFIC_VENDOR_WRITE_REQUEST = 0x01

    # 0x21
    PROLIFIC_CTRL_OUT_REQTYPE = (
        usb.UsbConstants.USB_DIR_OUT
        | usb.UsbConstants.USB_TYPE_CLASS
        | USB_RECIP_INTERFACE
    )

    # 0xa1
    PROLIFIC_CTRL_IN_REQTYPE = (
        usb.UsbConstants.USB_DIR_IN
        | usb.UsbConstants.USB_TYPE_CLASS
        | USB_RECIP_INTERFACE
    )

    # 0x40
    PROLIFIC_VENDOR_OUT_REQTYPE = (
        usb.UsbConstants.USB_DIR_OUT | usb.UsbConstants.USB_TYPE_VENDOR
    )

    # 0xC0
    PROLIFIC_VENDOR_IN_REQTYPE = (
        usb.UsbConstants.USB_DIR_IN | usb.UsbConstants.USB_TYPE_VENDOR
    )

    WRITE_ENDPOINT = 0x02
    READ_ENDPOINT = 0x83
    INTERRUPT_ENDPOINT = 0x81

    FLUSH_RX_REQUEST = 0x08
    FLUSH_TX_REQUEST = 0x09

    SET_LINE_REQUEST = 0x20
    SET_CONTROL_REQUEST = 0x22

    CONTROL_DTR = 0x01
    CONTROL_RTS = 0x02

    BREAK_REQUEST = 0x23
    BREAK_ON = 0xFFFF
    BREAK_OFF = 0x0000

    GET_LINE_REQUEST = 0x21

    STATUS_FLAG_CD = 0x01
    STATUS_FLAG_DSR = 0x02
    STATUS_FLAG_RI = 0x08
    STATUS_FLAG_CTS = 0x80

    STATUS_BUFFER_SIZE = 10
    STATUS_BYTE_IDX = 8

    DEVICE_TYPE_HX = 0
    DEVICE_TYPE_0 = 1
    DEVICE_TYPE_1 = 2

    # Buffer
    DEFAULT_READ_BUFFER_SIZE = 16 * 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024

    # Timeout
    USB_READ_TIMEOUT_MILLIS = 5000
    USB_WRITE_TIMEOUT_MILLIS = 5000

    def __init__(self, *args, **kwargs):
        self._device = None
        self._connection = None
        self._interface = None
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._read_buffer = bytearray()
        self._device_type = self.DEVICE_TYPE_HX
        super(Pl2303Serial, self).__init__(*args, **kwargs)

    def open(self):
        """Open the serial port.

        When the serial port is instantiated, it will try to open automatically.
        """
        self.close()

        device = usb.get_usb_device(self.portstr)
        if not device:
            raise SerialException("Device not present {}".format(self.portstr))

        if not usb.has_usb_permission(device):
            usb.request_usb_permission(device)
            return

        connection = usb.get_usb_manager().openDevice(device)
        if not connection:
            raise SerialException("Failed to open device!")

        self._device = device
        self._connection = connection

        for i in range(self._device.getInterfaceCount()):
            if not self._connection.claimInterface(self._device.getInterface(i), True):
                raise SerialException("Could not claim interface {}.".format(i))

        self._interface = self._device.getInterface(
            self._device.getInterfaceCount() - 1
        )

        for i in range(self._interface.getEndpointCount()):
            ep = self._interface.getEndpoint(i)
            if (ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_INT
            ):
                self._control_endpoint = ep
            elif (ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                self._read_endpoint = ep
            elif (ep.getDirection() == usb.UsbConstants.USB_DIR_OUT) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                self._write_endpoint = ep

        # Check that all endpoints are good
        if None in [self._control_endpoint, self._read_endpoint, self._write_endpoint]:
            raise SerialException("Could not establish all endpoints!")

        if self._device.getDeviceClass() == usb.UsbConstants.USB_CLASS_COMM:
            self._device_type = self.DEVICE_TYPE_0
        else:
            raw_descriptors = self._connection.getRawDescriptors()
            max_packet_size_0 = raw_descriptors[7]
            if max_packet_size_0 == 64:
                self._device_type = self.DEVICE_TYPE_HX
            elif (
                self._device.getDeviceClass()
                == usb.UsbConstants.USB_CLASS_PER_INTERFACE
                or self._device.getDeviceClass()
                == usb.UsbConstants.USB_CLASS_VENDOR_SPEC
            ):
                self._device_type = self.DEVICE_TYPE_1
            else:
                # Unknown device sub type, assume DEVICE_TYPE_HX.
                self._device_type = self.DEVICE_TYPE_HX

        self._init_device()
        self.is_open = True
        self._set_dtr_rts(self._dtr_state, self._rts_state)
        self._reconfigure_port()

    def _reconfigure_port(self):
        """Reconfigure serial port parameters."""
        self._set_parameters(self.baudrate, self.bytesize, self.parity, self.stopbits)

    def close(self):
        """Close the serial port."""
        if self._connection:
            self._connection.close()
        self._connection = None
        self.is_open = False

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    @property
    def in_waiting(self):
        """Return the number of bytes currently in the input buffer.

        Returns:
            Length (int): number of data bytes in the input buffer.
        """
        # Read from serial port hardware and put the data into read buffer.
        self._read_buffer.extend(self._read())
        return len(self._read_buffer)

    @property
    def out_waiting(self):
        """Return the number of bytes currently in the output buffer.

        Always return 0.
        """
        return 0

    def read(self, size=1):
        """Read data from the serial port.

        Parameters:
            size (int): the number of data bytes to read.

        Returns:
            read (bytes): data bytes read from the serial port.
        """
        read = bytearray()
        timeout = Timeout(self.timeout)

        # If there is enough data in the buffer, do not bother to read.
        if len(self._read_buffer) < size:
            # Keep reading until there is enough data or timeout.
            while self.in_waiting < size:
                if timeout.expired():
                    break

        # Get data from read buffer.
        read = self._read_buffer[:size]
        self._read_buffer = self._read_buffer[size:]

        return bytes(read)

    def write(self, data):
        """Write data to the serial port.

        Parameters:
            data (bytearray): data written to the serial port.

        Returns:
            wrote (int): the number of data bytes written.
        """
        if not self.is_open:
            return None
        offset = 0
        timeout = int(
            self._write_timeout * 1000
            if self._write_timeout
            else self.USB_WRITE_TIMEOUT_MILLIS
        )
        wrote = 0
        while offset < len(data):
            data_length = min(len(data) - offset, self.DEFAULT_WRITE_BUFFER_SIZE)
            buf = data[offset : offset + data_length]
            i = self._connection.bulkTransfer(
                self._write_endpoint, buf, data_length, timeout
            )
            if i <= 0:
                raise SerialException("Failed to write {}: {}".format(buf, i))
            offset += data_length
            wrote += i
        return wrote

    def flush(self):
        """Simply wait some time to allow all data to be written."""
        pass

    def reset_input_buffer(self):
        """Clear input buffer."""
        if not self.is_open:
            raise PortNotOpenError()
        self._purgeHwBuffers(True, False)

    def reset_output_buffer(self):
        """Clear output buffer."""
        if not self.is_open:
            raise PortNotOpenError()
        self._purgeHwBuffers(False, True)

    def send_break(self, duration=0.25):
        """Send break condition.

        Parameters:
            duration (float): break time in seconds.
        """
        if not self.is_open:
            raise PortNotOpenError()
        self._set_break(True)
        time.sleep(duration)
        self._set_break(False)

    def _update_break_state(self):
        """Send break condition."""
        self._set_break(self._break_state)

    def _update_rts_state(self):
        """Set terminal status line: Request To Send."""
        self._set_rts(self._rts_state)

    def _update_dtr_state(self):
        """Set terminal status line: Data Terminal Ready."""
        self._set_dtr(self._dtr_state)

    @property
    def cts(self):
        """Read terminal status line: Clear To Send."""
        status = self._poll_modem_status()
        return bool(status & self.STATUS_FLAG_CTS)

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready."""
        status = self._poll_modem_status()
        return bool(status & self.STATUS_FLAG_DSR)

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator."""
        status = self._poll_modem_status()
        return bool(status & self.STATUS_FLAG_RI)

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect."""
        status = self._poll_modem_status()
        return bool(status & self.STATUS_FLAG_CD)

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def _ctrl_out(self, request, value, index, buf=None):
        """USB control transfer out.

        This function does the USB configuration job.
        """
        result = self._connection.controlTransfer(
            self.PROLIFIC_CTRL_OUT_REQTYPE,
            request,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

        return result

    def _ctrl_in(self, request, value, index, buf=None):
        """USB control transfer in.

        Request for a control message from the device.
        """
        result = self._connection.controlTransfer(
            self.PROLIFIC_CTRL_IN_REQTYPE,
            request,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_READ_TIMEOUT_MILLIS,
        )

        return result

    def _vendor_out(self, value, index, buf=None):
        """USB vendor transfer out.

        This function does the USB configuration job.
        """
        result = self._connection.controlTransfer(
            self.PROLIFIC_VENDOR_OUT_REQTYPE,
            self.PROLIFIC_VENDOR_WRITE_REQUEST,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

        return result

    def _vendor_in(self, value, index, buf=None):
        """USB vendor transfer in.

        Request for a control message from the device.
        """
        result = self._connection.controlTransfer(
            self.PROLIFIC_VENDOR_IN_REQTYPE,
            self.PROLIFIC_VENDOR_READ_REQUEST,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_READ_TIMEOUT_MILLIS,
        )

        return result

    def _init_device(self):
        buf = bytearray(1)

        self._vendor_in(0x8484, 0, buf)
        self._vendor_out(0x0404, 0)
        self._vendor_in(0x8484, 0, buf)
        self._vendor_in(0x8383, 0, buf)
        self._vendor_in(0x8484, 0, buf)
        self._vendor_out(0x0404, 1)
        self._vendor_in(0x8484, 0, buf)
        self._vendor_in(0x8383, 0, buf)
        self._vendor_out(0, 1)
        self._vendor_out(1, 0)
        self._vendor_out(
            2, (0x44 if self._device_type == self.DEVICE_TYPE_HX else 0x24)
        )

    def _set_break(self, break_):
        """Start or stop a break exception event on the serial line.

        Parameters:
            break_ (bool): either start or stop break event.
        """
        result = self._ctrl_out(
            self.BREAK_REQUEST, self.BREAK_ON if break_ else self.BREAK_OFF, 0
        )
        if result != 0:
            raise SerialException("Unable to set break state!")

    def _set_dtr(self, state):
        """Set dtr line.

        Parameters:
            state (bool): new DTR logical level.
        """
        self._set_dtr_rts(state, self._rts_state)

    def _set_rts(self, state):
        """Set rts line.

        Parameters:
            state (bool): new RTS logical level.
        """
        self._set_dtr_rts(self._dtr_state, state)

    def _set_dtr_rts(self, dtr, rts):
        """Set dtr and rts lines at once.

        Parameters:
            dtr (bool): new DTR logical level.
            rts (bool): new RTS logical level.
        """
        value = 0
        if dtr:
            value |= self.CONTROL_DTR
        if rts:
            value |= self.CONTROL_RTS

        result = self._ctrl_out(self.SET_CONTROL_REQUEST, value, 0)
        if result != 0:
            raise SerialException("Unable to set DTR/RTS lines!")

    def _purgeHwBuffers(self, purgeReadBuffers, purgeWriteBuffers):
        """Set serial port parameters.

        Parameters:
            purgeReadBuffers (bool): need to purge read buffer or not.
            purgeWriteBuffers (bool): need to purge write buffer or not.
        Returns:
            result (bool): successful or not.
        """
        if purgeReadBuffers:
            self._vendor_out(self.FLUSH_RX_REQUEST, 0)

        if purgeWriteBuffers:
            self._vendor_out(self.FLUSH_TX_REQUEST, 0)

        return True

    def _read(self):
        """Hardware dependent read function.

        Returns:
            read (bytes): data bytes read from the serial port.
        """
        if not self.is_open:
            raise PortNotOpenError()
        if not self._read_endpoint:
            raise SerialException("Read endpoint does not exist!")

        # Get raw data from hardware.
        buf = bytearray(self.DEFAULT_READ_BUFFER_SIZE)
        totalBytesRead = self._connection.bulkTransfer(
            self._read_endpoint,
            buf,
            self.DEFAULT_READ_BUFFER_SIZE,
            self.USB_READ_TIMEOUT_MILLIS,
        )
        if totalBytesRead < 0:
            # Read timeout. Set totalBytesRead to 0.
            totalBytesRead = 0

        read = buf[:totalBytesRead]
        return bytes(read)

    def _set_parameters(self, baudrate, databits, parity, stopbits):
        """Set serial port parameters.

        Parameters:
            baudrate (int): the new baudrate for the UART(eg 9600).
            databits (int): number of bits in data(5, 6, 7 or 8).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
            stopbits (float): number of stop bits(1, 1.5, 2).
        """
        buf = bytearray(7)

        buf[0] = baudrate & 0xFF
        buf[1] = (baudrate >> 8) & 0xFF
        buf[2] = (baudrate >> 16) & 0xFF
        buf[3] = (baudrate >> 24) & 0xFF

        if stopbits == 1:
            buf[4] = 0
        elif stopbits == 1.5:
            buf[4] = 1
        elif stopbits == 2:
            buf[4] = 2
        else:
            buf[4] = 0

        if parity == "N":
            buf[5] = 0
        elif parity == "O":
            buf[5] = 1
        elif parity == "E":
            buf[5] = 2
        elif parity == "M":
            buf[5] = 3
        elif parity == "S":
            buf[5] = 4
        else:
            buf[5] = 0

        buf[6] = databits & 0xFF

        self._ctrl_out(self.SET_LINE_REQUEST, 0, 0, buf)

    def _poll_modem_status(self):
        """Poll modem status information.

        This function allows the retrieve the one status byte of the
        device, useful in UART mode.

        Returns:
            status (int): modem status, as a proprietary bitfield
        """
        buf = bytearray(self.STATUS_BUFFER_SIZE)
        read_bytes_count = self._connection.bulkTransfer(
            self._control_endpoint, buf, len(buf), self.USB_READ_TIMEOUT_MILLIS
        )

        if read_bytes_count != self.STATUS_BUFFER_SIZE:
            raise SerialException("Unable to get modem status!")

        status = buf[self.STATUS_BYTE_IDX] & 0xFF
        return status
