"""Android USB serial CH34x driver.

Classes:
Ch34xSerial
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


class Ch34xSerial(SerialBase):
    """CH34x serial port class."""

    # Default baudrate
    DEFAULT_BAUDRATE = 9600

    # Config request types
    REQTYPE_DEVICE_TO_HOST = (
        usb.UsbConstants.USB_TYPE_VENDOR | usb.UsbConstants.USB_DIR_IN
    )
    REQTYPE_HOST_TO_DEVICE = (
        usb.UsbConstants.USB_TYPE_VENDOR | usb.UsbConstants.USB_DIR_OUT
    )

    # Config request codes
    CH34X_REQ_READ_VERSION = 0x5F
    CH34X_REQ_WRITE_REG = 0x9A
    CH34X_REQ_READ_REG = 0x95
    CH34X_REQ_SERIAL_INIT = 0xA1
    CH34X_REQ_MODEM_CTRL = 0xA4

    CH34X_REG_BREAK = 0x05
    CH34X_REG_LCR = 0x18
    CH34X_NBREAK_BITS = 0x01

    # LCR values
    CH34X_LCR_ENABLE_RX = 0x80
    CH34X_LCR_ENABLE_TX = 0x40
    CH34X_LCR_MARK_SPACE = 0x20
    CH34X_LCR_PAR_EVEN = 0x10
    CH34X_LCR_ENABLE_PAR = 0x08
    CH34X_LCR_STOP_BITS_2 = 0x04
    CH34X_LCR_CS8 = 0x03
    CH34X_LCR_CS7 = 0x02
    CH34X_LCR_CS6 = 0x01
    CH34X_LCR_CS5 = 0x00

    # Baud rates values
    CH34X_300_1312 = 0xD980
    CH34X_300_0f2c = 0xEB

    CH34X_600_1312 = 0x6481
    CH34X_600_0f2c = 0x76

    CH34X_1200_1312 = 0xB281
    CH34X_1200_0f2c = 0x3B

    CH34X_2400_1312 = 0xD981
    CH34X_2400_0f2c = 0x1E

    CH34X_4800_1312 = 0x6482
    CH34X_4800_0f2c = 0x0F

    CH34X_9600_1312 = 0xB282
    CH34X_9600_0f2c = 0x08

    CH34X_19200_1312 = 0xD982
    CH34X_19200_0f2c_rest = 0x07

    CH34X_38400_1312 = 0x6483

    CH34X_57600_1312 = 0x9883

    CH34X_115200_1312 = 0xCC83

    CH34X_230400_1312 = 0xE683

    CH34X_460800_1312 = 0xF383

    CH34X_921600_1312 = 0xF387

    # Parity values
    CH34X_PARITY_NONE = 0x00
    CH34X_PARITY_ODD = 0x08
    CH34X_PARITY_EVEN = 0x18
    CH34X_PARITY_MARK = 0x28
    CH34X_PARITY_SPACE = 0x38

    # Flow control values
    CH34X_FLOW_CONTROL_NONE = 0x0000
    CH34X_FLOW_CONTROL_RTS_CTS = 0x0101
    CH34X_FLOW_CONTROL_DSR_DTR = 0x0202
    CH34X_CONTROL_DTR = 0x20
    CH34X_CONTROL_RTS = 0x40
    CH34X_CONTROL_CTS = 0x01
    CH34X_CONTROL_DSR = 0x02
    CH34X_CONTROL_RI = 0x04
    CH34X_CONTROL_DCD = 0x08

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
        self._index = 0
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._lineprop = 0
        self._read_buffer = bytearray()
        super(Ch34xSerial, self).__init__(*args, **kwargs)

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

        self._init_device()

        self.is_open = True

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
        return bool(status & self.CH34X_CONTROL_CTS)

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready."""
        status = self._poll_modem_status()
        return bool(status & self.CH34X_CONTROL_DSR)

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator."""
        status = self._poll_modem_status()
        return bool(status & self.CH34X_CONTROL_RI)

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect."""
        status = self._poll_modem_status()
        return bool(status & self.CH34X_CONTROL_DCD)

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def _ctrl_transfer_out(self, request, value, index, buf=None):
        """USB control transfer out.

        This function does the USB configuration job.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_HOST_TO_DEVICE,
            request,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

        return result

    def _ctrl_transfer_in(self, request, value, index, buf=None):
        """USB control transfer in.

        Request for a control message from the device.
        """
        result = self._connection.controlTransfer(
            self.REQTYPE_DEVICE_TO_HOST,
            request,
            value,
            index,
            buf,
            (0 if buf is None else len(buf)),
            self.USB_READ_TIMEOUT_MILLIS,
        )

        return result

    def _check_state(self, msg, request, value, expected):
        buf = bytearray(len(expected))
        ret = self._ctrl_transfer_in(request, value, 0, buf)
        if ret != len(expected):
            raise SerialException(
                "Expected {} bytes, got {}, [{}]".format(len(expected), ret, msg)
            )

    def _init_device(self):
        if self._ctrl_transfer_out(0xA1, 0xC29C, 0xB2B9) < 0:
            raise SerialException("Init failed! #1")
        if self._ctrl_transfer_out(0xA4, 0xDF, 0) < 0:
            raise SerialException("Init failed! #2")
        if self._ctrl_transfer_out(0xA4, 0x9F, 0) < 0:
            raise SerialException("Init failed! #3")

        self._check_state("Init #4", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE])

        if self._ctrl_transfer_out(0x9A, 0x2727, 0x0000) < 0:
            raise SerialException("Init failed! #5")
        if self._ctrl_transfer_out(0x9A, 0x1312, 0xB282) < 0:
            raise SerialException("Init failed! #6")
        if self._ctrl_transfer_out(0x9A, 0x0F2C, 0x0008) < 0:
            raise SerialException("Init failed! #7")
        if self._ctrl_transfer_out(0x9A, 0x2518, 0x00C3) < 0:
            raise SerialException("Init failed! #8")

        self._check_state("Init #9", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE])

        if self._ctrl_transfer_out(0x9A, 0x2727, 0x0000) < 0:
            raise SerialException("Init failed! #10")

    def _set_break(self, break_):
        """Start or stop a break exception event on the serial line.

        Parameters:
            break_ (bool): either start or stop break event.
        """
        ch34x_break_reg = (self.CH34X_REG_LCR << 8) | self.CH34X_REG_BREAK
        break_reg = bytearray(2)
        if (
            self._ctrl_transfer_in(
                self.CH34X_REQ_READ_REG, ch34x_break_reg, 0, break_reg
            )
            < 0
        ):
            raise SerialException("Unable to read break state!")

        if break_:
            break_reg[0] &= ~self.CH34X_NBREAK_BITS
            break_reg[1] &= ~self.CH34X_LCR_ENABLE_TX
        else:
            break_reg[0] |= self.CH34X_NBREAK_BITS
            break_reg[1] |= self.CH34X_LCR_ENABLE_TX

        # Unpack break_reg into int
        reg_contents = break_reg[1] * 256 + break_reg[0]

        if (
            self._ctrl_transfer_out(
                self.CH34X_REQ_WRITE_REG, ch34x_break_reg, reg_contents
            )
            < 0
        ):
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
            value |= self.CH34X_CONTROL_DTR
        if rts:
            value |= self.CH34X_CONTROL_RTS
        if self._ctrl_transfer_out(self.CH34X_REQ_MODEM_CTRL, ~value, 0) < 0:
            raise SerialException("Unable to set DTR/RTS lines!")

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

    def _set_baudrate(self, baudrate):
        """Change the current UART baudrate.

        Parameters:
            baudrate (int): the new baudrate for the UART.

        Raises:
            SerialException: if not able to set baudrate.
        """
        if baudrate <= 300:
            index1312 = self.CH34X_300_1312
            index0f2c = self.CH34X_300_0f2c
        elif 300 < baudrate <= 600:
            index1312 = self.CH34X_600_1312
            index0f2c = self.CH34X_600_0f2c
        elif 600 < baudrate <= 1200:
            index1312 = self.CH34X_1200_1312
            index0f2c = self.CH34X_1200_0f2c
        elif 1200 < baudrate <= 2400:
            index1312 = self.CH34X_2400_1312
            index0f2c = self.CH34X_2400_0f2c
        elif 2400 < baudrate <= 4800:
            index1312 = self.CH34X_4800_1312
            index0f2c = self.CH34X_4800_0f2c
        elif 4800 < baudrate <= 9600:
            index1312 = self.CH34X_9600_1312
            index0f2c = self.CH34X_9600_0f2c
        elif 9600 < baudrate <= 19200:
            index1312 = self.CH34X_19200_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 19200 < baudrate <= 38400:
            index1312 = self.CH34X_38400_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 38400 < baudrate <= 57600:
            index1312 = self.CH34X_57600_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 57600 < baudrate <= 115200:
            index1312 = self.CH34X_115200_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 115200 < baudrate <= 230400:
            index1312 = self.CH34X_230400_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 230400 < baudrate <= 460800:
            index1312 = self.CH34X_460800_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        elif 460800 < baudrate <= 921600:
            index1312 = self.CH34X_921600_1312
            index0f2c = self.CH34X_19200_0f2c_rest
        else:
            raise SerialException("Baudrate out of range!")

        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x1312, index1312) < 0:
            raise SerialException("Fail to set baudrate index1312!")

        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x0F2C, index0f2c) < 0:
            raise SerialException("Fail to set baudrate index0f2c!")

        self._check_state("Set baudrate", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE])

        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x2727, 0) < 0:
            raise SerialException("Fail to set baudrate!")

    def _set_line_property(self, bits, stopbits, parity):
        """Set serial port line property.

        Parameters:
            bits (int): number of bits in data(5, 6, 7 or 8).
            stopbits (float): number of stop bits(1, 1.5, 2).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
        """
        lcr = self.CH34X_LCR_ENABLE_RX | self.CH34X_LCR_ENABLE_TX

        if bits == 5:
            lcr |= self.CH34X_LCR_CS5
        elif bits == 6:
            lcr |= self.CH34X_LCR_CS6
        elif bits == 7:
            lcr |= self.CH34X_LCR_CS7
        elif bits == 8:
            lcr |= self.CH34X_LCR_CS8
        else:
            raise ValueError("Unknown bits value: {}".format(bits))

        if parity == "N":
            pass
        elif parity == "O":
            lcr |= self.CH34X_LCR_ENABLE_PAR
        elif parity == "E":
            lcr |= self.CH34X_LCR_ENABLE_PAR | self.CH34X_LCR_PAR_EVEN
        elif parity == "M":
            lcr |= self.CH34X_LCR_ENABLE_PAR | self.CH34X_LCR_MARK_SPACE
        elif parity == "S":
            lcr |= (
                self.CH34X_LCR_ENABLE_PAR
                | self.CH34X_LCR_MARK_SPACE
                | self.CH34X_LCR_PAR_EVEN
            )
        else:
            raise ValueError("Unknown parity value: {}".format(parity))

        if stopbits == 1:
            pass
        elif stopbits == 1.5:
            pass
        elif stopbits == 2:
            lcr |= self.CH34X_LCR_STOP_BITS_2
        else:
            raise ValueError("Unknown stopbits value: {}".format(stopbits))

        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x2518, lcr) < 0:
            raise SerialException("Setting line property failed!")

        self._check_state("Set parity", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE])

        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x2727, 0) < 0:
            raise SerialException("Fail to set line property!")

        self._lineprop = lcr

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
        buf = bytearray(2)
        result = self._ctrl_transfer_in(self.CH34X_REQ_READ_REG, 0x0706, 0, buf)
        if result != 2:
            raise SerialException("Unable to get modem status!")
        buf = buf[:2]
        (status,) = unpack("<H", bytes(buf))
        return status

    def _set_flowctrl(self, flowctrl):
        """Select flowcontrol in UART mode.

        Either hardware flow control through RTS/CTS or DSR/DTR,
        software flow control or no flow control.
        Parameters:
            flowctrl (str): 'hw_rtscts', 'hw_dsrdtr', 'sw' or ''
        """
        if flowctrl == "hw_rtscts":
            ch34x_flowctrl = self.CH34X_FLOW_CONTROL_RTS_CTS
            self.rts = True
        elif flowctrl == "hw_dsrdtr":
            ch34x_flowctrl = self.CH34X_FLOW_CONTROL_DSR_DTR
            self.dtr = True
        # elif flowctrl == 'sw':
        else:
            ch34x_flowctrl = self.CH34X_FLOW_CONTROL_NONE

        self._check_state(
            "Set flow control", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE]
        )

        if (
            self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x2727, ch34x_flowctrl)
            < 0
        ):
            raise SerialException("Setting flow control failed!")
