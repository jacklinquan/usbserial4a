"""Android USB serial CDC ACM driver.

Classes:
CdcAcmSerial
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


class CdcAcmSerial(SerialBase):
    """CDC ACM serial port class."""

    REQTYPE_HOST2DEVICE = 0x21
    REQTYPE_DEVICE2HOST = 0xA1

    SET_LINE_CODING = 0x20
    GET_LINE_CODING = 0x21
    SET_CONTROL_LINE_STATE = 0x22
    SEND_BREAK = 0x23

    SET_CONTROL_LINE_STATE_DTR = 0x1
    SET_CONTROL_LINE_STATE_RTS = 0x2

    STOPBIT_MAP = {1: 0, 1.5: 1, 2: 2}
    PARITY_MAP = {"N": 0, "O": 1, "E": 2, "M": 3, "S": 4}

    DEFAULT_READ_BUFFER_SIZE = 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024

    USB_READ_TIMEOUT_MILLIS = 50
    USB_WRITE_TIMEOUT_MILLIS = 5000

    def __init__(self, *args, **kwargs):
        self._device = None
        self._connection = None
        self._control_index = None
        self._control_interface = None
        self._data_interface = None
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._read_buffer = bytearray()
        super(CdcAcmSerial, self).__init__(*args, **kwargs)

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

        if self._device.getInterfaceCount() == 1:
            # Device might be castrated ACM device. Try single interface logic.
            self._open_single_interface()
        else:
            # Try default interface logic.
            self._open_interface()

        # Check that all endpoints are good
        if None in [self._control_endpoint, self._write_endpoint, self._read_endpoint]:
            raise SerialException("Could not establish all endpoints!")

        self.is_open = True
        self._set_dtr_rts(self._dtr_state, self._rts_state)
        self._reconfigure_port()

    def _open_single_interface(self):
        """Open single interface device."""
        device = self._device

        # Claiming control/data interface.
        self._control_index = 0
        self._control_interface = device.getInterface(0)
        self._data_interface = device.getInterface(0)
        if not self._connection.claimInterface(self._control_interface, True):
            raise SerialException("Could not claim shared control/data interface.")

        num_endpoints = self._control_interface.getEndpointCount()
        if num_endpoints < 3:
            msg = "Not enough endpoints - need 3, got {}".format(num_endpoints)
            raise SerialException(msg)

        for i in range(num_endpoints):
            ep = self._control_interface.getEndpoint(i)
            if (ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_INT
            ):
                # Found controlling endpoint.
                self._control_endpoint = ep
            elif (ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                # Found reading endpoint.
                self._read_endpoint = ep
            elif (ep.getDirection() == usb.UsbConstants.USB_DIR_OUT) and (
                ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                # Found writing endpoint
                self._write_endpoint = ep

            if None not in [
                self._control_endpoint,
                self._write_endpoint,
                self._read_endpoint,
            ]:
                # Found all endpoints.
                break

    def _open_interface(self):
        """Open default interface device."""
        device = self._device

        for i in range(device.getInterfaceCount()):
            interface = device.getInterface(i)
            if interface.getInterfaceClass() == usb.UsbConstants.USB_CLASS_COMM:
                self._control_index = i
                self._control_interface = interface
            if interface.getInterfaceClass() == usb.UsbConstants.USB_CLASS_CDC_DATA:
                self._data_interface = interface

        if self._control_interface is None:
            raise SerialException("Could not find control interface.")
        if not self._connection.claimInterface(self._control_interface, True):
            raise SerialException("Could not claim control interface.")

        self._control_endpoint = self._control_interface.getEndpoint(0)

        if self._data_interface is None:
            raise SerialException("Could not find data interface.")
        if not self._connection.claimInterface(self._data_interface, True):
            raise SerialException("Could not claim data interface.")

        for i in range(self._data_interface.getEndpointCount()):
            ep = self._data_interface.getEndpoint(i)
            if (
                ep.getDirection() == usb.UsbConstants.USB_DIR_IN
                and ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                self._read_endpoint = ep
            if (
                ep.getDirection() == usb.UsbConstants.USB_DIR_OUT
                and ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK
            ):
                self._write_endpoint = ep

        if None in (self._read_endpoint, self._write_endpoint):
            raise SerialException("Could not find read/write endpoint.")

    def _reconfigure_port(self):
        """Reconfigure serial port parameters."""
        msg = bytearray(
            [
                self.baudrate & 0xFF,
                self.baudrate >> 8 & 0xFF,
                self.baudrate >> 16 & 0xFF,
                self.baudrate >> 24 & 0xFF,
                self.STOPBIT_MAP[self.stopbits],
                self.PARITY_MAP[self.parity],
                self.bytesize,
            ]
        )
        # Set line coding.
        self._ctrl_transfer_out(self.SET_LINE_CODING, 0, msg)

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
        self._ctrl_transfer_out(self.SEND_BREAK, int(duration * 1000))

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
        return bool(status)

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready."""
        status = self._poll_modem_status()
        return bool(status)

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator."""
        status = self._poll_modem_status()
        return bool(status)

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect."""
        status = self._poll_modem_status()
        return bool(status)

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def _ctrl_transfer_out(self, request, value, buf=None):
        """USB control transfer out.

        This function does the USB configuration job.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_HOST2DEVICE,
            request,
            value,
            self._control_index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

        return result

    def _ctrl_transfer_in(self, request, value, buf):
        """USB control transfer in.

        Request for a control message from the device.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_DEVICE2HOST,
            request,
            value,
            self._control_index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_READ_TIMEOUT_MILLIS,
        )

        return result

    def _set_break(self, break_):
        """Start or stop a break exception event on the serial line.

        Parameters:
            break_ (bool): either start or stop break event.
        """
        pass

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
            value |= self.SET_CONTROL_LINE_STATE_DTR
        if rts:
            value |= self.SET_CONTROL_LINE_STATE_RTS
        self._ctrl_transfer_out(self.SET_CONTROL_LINE_STATE, value)

    def _purgeHwBuffers(self, purgeReadBuffers, purgeWriteBuffers):
        """Set serial port parameters.

        Parameters:
            purgeReadBuffers (bool): need to purge read buffer or not.
            purgeWriteBuffers (bool): need to purge write buffer or not.
        Returns:
            result (bool): successful or not.
        """
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

    def _poll_modem_status(self):
        """Poll modem status information.

        This function allows the retrieve the one status byte of the
        device, useful in UART mode.

        Returns:
            status (int): modem status, as a proprietary bitfield
        """
        return 0
