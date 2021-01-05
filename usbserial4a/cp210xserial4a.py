"""Android USB serial CP210x driver.

Classes:
Cp210xSerial
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


class Cp210xSerial(SerialBase):
    """CP210x serial port class."""

    # Default baudrate
    DEFAULT_BAUDRATE = 9600

    # Config request types
    REQTYPE_HOST_TO_INTERFACE = 0x41
    REQTYPE_INTERFACE_TO_HOST = 0xC1
    REQTYPE_HOST_TO_DEVICE = 0x40
    REQTYPE_DEVICE_TO_HOST = 0xC0

    # Config request codes
    CP210X_IFC_ENABLE = 0x00
    CP210X_SET_BAUDDIV = 0x01
    CP210X_GET_BAUDDIV = 0x02
    CP210X_SET_LINE_CTL = 0x03
    CP210X_GET_LINE_CTL = 0x04
    CP210X_SET_BREAK = 0x05
    CP210X_IMM_CHAR = 0x06
    CP210X_SET_MHS = 0x07
    CP210X_GET_MDMSTS = 0x08
    CP210X_SET_XON = 0x09
    CP210X_SET_XOFF = 0x0A
    CP210X_SET_EVENTMASK = 0x0B
    CP210X_GET_EVENTMASK = 0x0C
    CP210X_SET_CHAR = 0x0D
    CP210X_GET_CHARS = 0x0E
    CP210X_GET_PROPS = 0x0F
    CP210X_GET_COMM_STATUS = 0x10
    CP210X_RESET = 0x11
    CP210X_PURGE = 0x12
    CP210X_SET_FLOW = 0x13
    CP210X_GET_FLOW = 0x14
    CP210X_EMBED_EVENTS = 0x15
    CP210X_GET_EVENTSTATE = 0x16
    CP210X_SET_CHARS = 0x19
    CP210X_GET_BAUDRATE = 0x1D
    CP210X_SET_BAUDRATE = 0x1E

    # CP210X_IFC_ENABLE
    UART_ENABLE = 0x0001
    UART_DISABLE = 0x0000

    # CP210X_(SET|GET)_BAUDDIV
    BAUD_RATE_GEN_FREQ = 0x384000

    # CP210X_(SET|GET)_LINE_CTL
    BITS_DATA_MASK = 0x0F00
    BITS_DATA_5 = 0x0500
    BITS_DATA_6 = 0x0600
    BITS_DATA_7 = 0x0700
    BITS_DATA_8 = 0x0800
    BITS_DATA_9 = 0x0900

    BITS_PARITY_MASK = 0x00F0
    BITS_PARITY_NONE = 0x0000
    BITS_PARITY_ODD = 0x0010
    BITS_PARITY_EVEN = 0x0020
    BITS_PARITY_MARK = 0x0030
    BITS_PARITY_SPACE = 0x0040

    BITS_STOP_MASK = 0x000F
    BITS_STOP_1 = 0x0000
    BITS_STOP_1_5 = 0x0001
    BITS_STOP_2 = 0x0002

    # CP210X_SET_BREAK
    BREAK_ON = 0x0001
    BREAK_OFF = 0x0000

    # CP210X_(SET_MHS|GET_MDMSTS)
    CONTROL_DTR = 0x0001
    CONTROL_RTS = 0x0002
    CONTROL_CTS = 0x0010
    CONTROL_DSR = 0x0020
    CONTROL_RING = 0x0040
    CONTROL_DCD = 0x0080
    CONTROL_WRITE_DTR = 0x0100
    CONTROL_WRITE_RTS = 0x0200

    # Purge
    FLUSH_WRITE_CODE = 0x05
    FLUSH_READ_CODE = 0x0A

    # Buffer
    DEFAULT_READ_BUFFER_SIZE = 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024

    # Timeout
    USB_READ_TIMEOUT_MILLIS = 5000
    USB_WRITE_TIMEOUT_MILLIS = 5000

    def __init__(self, *args, **kwargs):
        self._device = None
        self._connection = None
        self._interface = None
        self._index = 0
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._lineprop = 0
        self._read_buffer = bytearray()
        super(Cp210xSerial, self).__init__(*args, **kwargs)

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
        if None in [self._write_endpoint, self._read_endpoint]:
            raise SerialException("Could not establish all endpoints!")

        # Enable UART
        self._ctrl_transfer_out(self.CP210X_IFC_ENABLE, self.UART_ENABLE)
        # Set Baud Div
        self._ctrl_transfer_out(
            self.CP210X_SET_BAUDDIV, self.BAUD_RATE_GEN_FREQ / self.DEFAULT_BAUDRATE
        )

        self.is_open = True

        # Set DTR and RTS
        self.dtr = True
        self.rts = True

        self._reconfigure_port()

    def _reconfigure_port(self):
        """Reconfigure serial port parameters."""
        self._setParameters(self.baudrate, self.bytesize, self.parity, self.stopbits)

        if self._rtscts:
            self._set_flowctrl("hw_rtscts")
        elif self._dsrdtr:
            self._set_flowctrl("hw_dsrdtr")
        elif self._xonxoff:
            self._set_flowctrl("sw")
        else:
            self._set_flowctrl("")

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
        return bool(status & self.CONTROL_CTS)

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready."""
        status = self._poll_modem_status()
        return bool(status & self.CONTROL_DSR)

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator."""
        status = self._poll_modem_status()
        return bool(status & self.CONTROL_RING)

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect."""
        status = self._poll_modem_status()
        return bool(status & self.CONTROL_DCD)

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def _ctrl_transfer_out(self, request, value, buf=None):
        """USB control transfer out.

        This function does the USB configuration job.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_HOST_TO_INTERFACE,
            request,
            value,
            self._index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

        return result

    def _ctrl_transfer_in(self, request, buf):
        """USB control transfer in.

        Request for a control message from the device.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_INTERFACE_TO_HOST,
            request,
            0,
            self._index,
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
        if break_:
            if self._ctrl_transfer_out(self.CP210X_SET_BREAK, self.BREAK_ON) < 0:
                raise SerialException("Unable to start break sequence")
        else:
            if self._ctrl_transfer_out(self.CP210X_SET_BREAK, self.BREAK_OFF) < 0:
                raise SerialException("Unable to stop break sequence")

    def _set_dtr(self, state):
        """Set dtr line.

        Parameters:
            state (bool): new DTR logical level.
        """
        value = 0
        if state:
            value |= self.CONTROL_DTR
        if (
            self._ctrl_transfer_out(self.CP210X_SET_MHS, value | self.CONTROL_WRITE_DTR)
            < 0
        ):
            raise SerialException("Unable to set DTR line")

    def _set_rts(self, state):
        """Set rts line.

        Parameters:
            state (bool): new RTS logical level.
        """
        value = 0
        if state:
            value |= self.CONTROL_RTS
        if (
            self._ctrl_transfer_out(self.CP210X_SET_MHS, value | self.CONTROL_WRITE_RTS)
            < 0
        ):
            raise SerialException("Unable to set RTS line")

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
        if (
            self._ctrl_transfer_out(
                self.CP210X_SET_MHS,
                value | self.CONTROL_WRITE_DTR | self.CONTROL_WRITE_RTS,
            )
            < 0
        ):
            raise SerialException("Unable to set DTR/RTS lines")

    def _purgeHwBuffers(self, purgeReadBuffers, purgeWriteBuffers):
        """Set serial port parameters.

        Parameters:
            purgeReadBuffers (bool): need to purge read buffer or not.
            purgeWriteBuffers (bool): need to purge write buffer or not.
        Returns:
            result (bool): successful or not.
        """
        if purgeReadBuffers:
            result = self._ctrl_transfer_out(self.CP210X_PURGE, self.FLUSH_READ_CODE)
            if result < 0:
                raise SerialException("Flushing RX failed: result={}".format(result))

        if purgeWriteBuffers:
            result = self._ctrl_transfer_out(self.CP210X_PURGE, self.FLUSH_WRITE_CODE)
            if result < 0:
                raise SerialException("Flushing TX failed: result={}".format(result))

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

    def _set_baudrate(self, baudrate):
        """Change the current UART baudrate.

        Parameters:
            baudrate (int): the new baudrate for the UART.

        Raises:
            SerialException: if not able to set baudrate.
        """
        baud_bytes = pack("<L", baudrate)
        result = self._ctrl_transfer_out(self.CP210X_SET_BAUDRATE, 0, baud_bytes)
        if result < 0:
            raise SerialException("Unable to set baudrate.")

    def _set_line_property(self, bits, stopbits, parity):
        """Set serial port line property.

        Parameters:
            bits (int): number of bits in data(5, 6, 7 or 8).
            stopbits (float): number of stop bits(1, 1.5, 2).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
        """
        value = (bits << 8) & self.BITS_DATA_MASK

        if parity == "N":
            value |= self.BITS_PARITY_NONE
        elif parity == "O":
            value |= self.BITS_PARITY_ODD
        elif parity == "E":
            value |= self.BITS_PARITY_EVEN
        elif parity == "M":
            value |= self.BITS_PARITY_MARK
        elif parity == "S":
            value |= self.BITS_PARITY_SPACE
        else:
            raise ValueError("Unknown parity value: {}".format(parity))

        if stopbits == 1:
            value |= self.BITS_STOP_1
        elif stopbits == 1.5:
            value |= self.BITS_STOP_1_5
        elif stopbits == 2:
            value |= self.BITS_STOP_2
        else:
            raise ValueError("Unknown stopbits value: {}".format(stopbits))

        result = self._ctrl_transfer_out(self.CP210X_SET_LINE_CTL, value)
        if result < 0:
            raise SerialException(
                "Setting line property failed: result={}".format(result)
            )

        self._lineprop = value

    def _setParameters(self, baudrate, databits, parity, stopbits):
        """Set serial port parameters.

        Parameters:
            baudrate (int): the new baudrate for the UART(eg 9600).
            databits (int): number of bits in data(5, 6, 7 or 8).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
            stopbits (float): number of stop bits(1, 1.5, 2).
        """

        self._set_baudrate(baudrate)
        self._set_line_property(databits, stopbits, parity)

    def _poll_modem_status(self):
        """Poll modem status information.

        This function allows the retrieve the one status byte of the
        device, useful in UART mode.

        Returns:
            status (int): modem status, as a proprietary bitfield
        """
        buf = bytearray(1)
        result = self._ctrl_transfer_in(self.CP210X_GET_MDMSTS, buf)
        if result != 1:
            raise SerialException("Unable to get modem status.")
        buf = buf[:1]
        (status,) = unpack("<B", bytes(buf))
        return status

    def _set_flowctrl(self, flowctrl):
        """Select flowcontrol in UART mode.

        Either hardware flow control through RTS/CTS or DSR/DTR,
        software flow control or no flow control.
        Parameters:
            flowctrl (str): 'hw_rtscts', 'hw_dsrdtr', 'sw' or ''
        """
        if flowctrl == "hw_rtscts":
            data_hw_rtscts = bytearray(
                [
                    0x09,
                    0x00,
                    0x00,
                    0x00,
                    0x40,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x20,
                    0x00,
                    0x00,
                ]
            )
            result1 = self._ctrl_transfer_out(self.CP210X_SET_FLOW, 0, data_hw_rtscts)
            result2 = 0
            self.rts = True
        elif flowctrl == "hw_dsrdtr":
            data_hw_dsrdtr = bytearray(
                [
                    0x11,
                    0x00,
                    0x00,
                    0x00,
                    0x40,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x20,
                    0x00,
                    0x00,
                ]
            )
            result1 = self._ctrl_transfer_out(self.CP210X_SET_FLOW, 0, data_hw_dsrdtr)
            result2 = 0
            self.dtr = True
        elif flowctrl == "sw":
            data_sw = bytearray(
                [
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    0x43,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x20,
                    0x00,
                    0x00,
                ]
            )
            data_chars = bytearray([0x00, 0x00, 0x00, 0x00, 0x11, 0x13])
            result2 = self._ctrl_transfer_out(self.CP210X_SET_CHARS, 0, data_chars)
            result1 = self._ctrl_transfer_out(self.CP210X_SET_FLOW, 0, data_sw)
        else:
            data_off = bytearray(
                [
                    0x01,
                    0x00,
                    0x00,
                    0x00,
                    0x40,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x80,
                    0x00,
                    0x00,
                    0x00,
                    0x20,
                    0x00,
                    0x00,
                ]
            )
            result1 = self._ctrl_transfer_out(self.CP210X_SET_FLOW, 0, data_off)
            result2 = 0

        if result1 < 0 or result2 < 0:
            raise SerialException(
                "Setting flow control failed: result1={}, result2={}".format(
                    result1, result2
                )
            )
