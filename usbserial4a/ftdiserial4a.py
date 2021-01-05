"""Android USB serial FTDI driver.

Classes:
FtdiSerial
"""

from struct import unpack
import time
from .utilserial4a import (
    SerialBase,
    SerialException,
    to_bytes,
    PortNotOpenError,
    Timeout,
)
from usb4a import usb


class FtdiSerial(SerialBase):
    """FTDI serial port class."""

    # Modem status
    MODEM_CTS = 1 << 4  # Clear to send
    MODEM_DSR = 1 << 5  # Data set ready
    MODEM_RI = 1 << 6  # Ring indicator
    MODEM_RLSD = 1 << 7  # Carrier detect
    MODEM_DR = 1 << 8  # Data ready
    MODEM_OE = 1 << 9  # Overrun error
    MODEM_PE = 1 << 10  # Parity error
    MODEM_FE = 1 << 11  # Framing error
    MODEM_BI = 1 << 12  # Break interrupt
    MODEM_THRE = 1 << 13  # Transmitter holding register
    MODEM_TEMT = 1 << 14  # Transmitter empty
    MODEM_RCVE = 1 << 15  # Error in RCVR FIFO

    # Requests
    SIO_RESET = 0  # Reset the port
    SIO_SET_MODEM_CTRL = 1  # Set the modem control register
    SIO_SET_FLOW_CTRL = 2  # Set flow control register
    SIO_SET_BAUDRATE = 3  # Set baud rate
    SIO_SET_DATA = 4  # Set the data characteristics of the port
    SIO_POLL_MODEM_STATUS = 5  # Get line status
    SIO_SET_EVENT_CHAR = 6  # Change event character
    SIO_SET_ERROR_CHAR = 7  # Change error character
    SIO_SET_LATENCY_TIMER = 9  # Change latency timer
    SIO_GET_LATENCY_TIMER = 10  # Get latency timer
    SIO_SET_BITMODE = 11  # Change bit mode
    SIO_READ_PINS = 12  # Read GPIO pin value

    # Eeprom requests
    SIO_EEPROM = 0x90
    SIO_READ_EEPROM = SIO_EEPROM + 0  # Read EEPROM content
    SIO_WRITE_EEPROM = SIO_EEPROM + 1  # Write EEPROM content
    SIO_ERASE_EEPROM = SIO_EEPROM + 2  # Erase EEPROM content

    # Reset commands
    SIO_RESET_SIO = 0  # Reset device
    SIO_RESET_PURGE_RX = 1  # Drain RX buffer
    SIO_RESET_PURGE_TX = 2  # Drain TX buffer

    # Flow control
    SIO_DISABLE_FLOW_CTRL = 0x0
    SIO_RTS_CTS_HS = 0x1 << 8
    SIO_DTR_DSR_HS = 0x2 << 8
    SIO_XON_XOFF_HS = 0x4 << 8
    SIO_SET_DTR_MASK = 0x1
    SIO_SET_DTR_HIGH = SIO_SET_DTR_MASK | (SIO_SET_DTR_MASK << 8)
    SIO_SET_DTR_LOW = 0x0 | (SIO_SET_DTR_MASK << 8)
    SIO_SET_RTS_MASK = 0x2
    SIO_SET_RTS_HIGH = SIO_SET_RTS_MASK | (SIO_SET_RTS_MASK << 8)
    SIO_SET_RTS_LOW = 0x0 | (SIO_SET_RTS_MASK << 8)

    # Break type
    BREAK_OFF, BREAK_ON = range(2)

    # cts:  Clear to send
    # dsr:  Data set ready
    # ri:   Ring indicator
    # dcd:  Data carrier detect
    # dr:   Data ready
    # oe:   Overrun error
    # pe:   Parity error
    # fe:   Framing error
    # bi:   Break interrupt
    # thre: Transmitter holding register
    # temt: Transmitter empty
    # err:  Error in RCVR FIFO
    MODEM_STATUS = [
        ("", "", "", "", "cts", "dsr", "ri", "dcd"),
        ("dr", "overrun", "parity", "framing", "break", "thre", "txe", "rcvr"),
    ]

    ERROR_BITS = (0x00, 0x8E)

    # Clocks and baudrates
    BUS_CLOCK_BASE = 6.0e6  # 6 MHz
    BUS_CLOCK_HIGH = 30.0e6  # 30 MHz
    BAUDRATE_REF_BASE = int(3.0e6)  # 3 MHz
    BAUDRATE_REF_HIGH = int(12.0e6)  # 12 MHz
    BAUDRATE_REF_SPECIAL = int(2.0e6)  # 3 MHz
    BAUDRATE_TOLERANCE = 3.0  # acceptable clock drift, in %
    BITBANG_CLOCK_MULTIPLIER = 4

    FTDI_DEVICE_OUT_REQTYPE = usb.build_usb_control_request_type(
        usb.UsbConstants.USB_DIR_OUT,
        usb.UsbConstants.USB_TYPE_VENDOR,
        usb.USB_RECIPIENT_DEVICE,
    )
    FTDI_DEVICE_IN_REQTYPE = usb.build_usb_control_request_type(
        usb.UsbConstants.USB_DIR_IN,
        usb.UsbConstants.USB_TYPE_VENDOR,
        usb.USB_RECIPIENT_DEVICE,
    )

    DEFAULT_READ_BUFFER_SIZE = 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024

    USB_READ_TIMEOUT_MILLIS = 5000
    USB_WRITE_TIMEOUT_MILLIS = 5000

    # Length of the modem status header, transmitted with every read.
    MODEM_STATUS_HEADER_LENGTH = 2

    def __init__(self, *args, **kwargs):
        self._device = None
        self._connection = None
        self._bcd_device = None
        self._interface = None
        self._index = None
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._lineprop = 0
        self._read_buffer = bytearray()
        super(FtdiSerial, self).__init__(*args, **kwargs)

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

        raw_descriptors = self._connection.getRawDescriptors()
        self._bcd_device = raw_descriptors[12] + raw_descriptors[13] * 256

        for i in range(self._device.getInterfaceCount()):
            if i == 0:
                self._interface = self._device.getInterface(i)
            if not self._connection.claimInterface(self._device.getInterface(i), True):
                raise SerialException("Could not claim interface {}.".format(i))

        self._index = self._interface.getId() + 1

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

        self.is_open = True
        self._reconfigure_port()

    def _reconfigure_port(self):
        """Reconfigure serial port parameters."""
        self._setParameters(self.baudrate, self.bytesize, self.parity, self.stopbits)

        if self._rtscts:
            self._set_flowctrl("hw")
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

    def reset(self):
        """Reset the serial port."""
        if self._connection:
            result = self._ctrl_transfer_out(
                self.SIO_RESET, self.SIO_RESET_SIO, self._index
            )
            if result != 0:
                raise SerialException("Reset failed: result={}".format(result))

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
        return bool(status & self.MODEM_CTS)

    @property
    def dsr(self):
        """Read terminal status line: Data Set Ready."""
        status = self._poll_modem_status()
        return bool(status & self.MODEM_DSR)

    @property
    def ri(self):
        """Read terminal status line: Ring Indicator."""
        status = self._poll_modem_status()
        return bool(status & self.MODEM_RI)

    @property
    def cd(self):
        """Read terminal status line: Carrier Detect."""
        status = self._poll_modem_status()
        return bool(status & self.MODEM_RLSD)

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def _ctrl_transfer_out(self, request, value, index):
        """USB control transfer out.

        This function does the USB configuration job.
        """
        return self._connection.controlTransfer(
            self.FTDI_DEVICE_OUT_REQTYPE,
            request,
            value,
            index,
            None,
            0,
            self.USB_WRITE_TIMEOUT_MILLIS,
        )

    def _ctrl_transfer_in(self, request, buf, length):
        """USB control transfer in.

        Request for a control message from the device.
        """
        return self._connection.controlTransfer(
            self.FTDI_DEVICE_IN_REQTYPE,
            request,
            0,
            self._index,
            buf,
            length,
            self.USB_READ_TIMEOUT_MILLIS,
        )

    def _set_break(self, break_):
        """Start or stop a break exception event on the serial line.

        Parameters:
            break_ (bool): either start or stop break event.
        """
        if break_:
            value = self._lineprop | (0x01 << 14)
            if self._ctrl_transfer_out(self.SIO_SET_DATA, value, self._index):
                raise SerialException("Unable to start break sequence")
        else:
            value = self._lineprop & ~(0x01 << 14)
            if self._ctrl_transfer_out(self.SIO_SET_DATA, value, self._index):
                raise SerialException("Unable to stop break sequence")
        self._lineprop = value

    def _set_dtr(self, state):
        """Set dtr line.

        Parameters:
            state (bool): new DTR logical level.
        """
        value = state and self.SIO_SET_DTR_HIGH or self.SIO_SET_DTR_LOW
        if self._ctrl_transfer_out(self.SIO_SET_MODEM_CTRL, value, self._index):
            raise SerialException("Unable to set DTR line")

    def _set_rts(self, state):
        """Set rts line.

        Parameters:
            state (bool): new RTS logical level.
        """
        value = state and self.SIO_SET_RTS_HIGH or self.SIO_SET_RTS_LOW
        if self._ctrl_transfer_out(self.SIO_SET_MODEM_CTRL, value, self._index):
            raise SerialException("Unable to set RTS line")

    def _set_dtr_rts(self, dtr, rts):
        """Set dtr and rts lines at once.

        Parameters:
            dtr (bool): new DTR logical level.
            rts (bool): new RTS logical level.
        """
        value = 0
        value |= dtr and self.SIO_SET_DTR_HIGH or self.SIO_SET_DTR_LOW
        value |= rts and self.SIO_SET_RTS_HIGH or self.SIO_SET_RTS_LOW
        if self._ctrl_transfer_out(self.SIO_SET_FLOW_CTRL, value, self._index):
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
            result = self._ctrl_transfer_out(
                self.SIO_RESET, self.SIO_RESET_PURGE_RX, self._index
            )
            if result != 0:
                raise SerialException("Flushing RX failed: result={}".format(result))

        if purgeWriteBuffers:
            result = self._ctrl_transfer_out(
                self.SIO_RESET, self.SIO_RESET_PURGE_TX, self._index
            )
            if result != 0:
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
        if totalBytesRead < self.MODEM_STATUS_HEADER_LENGTH:
            raise SerialException(
                "Expected at least {} bytes".format(self.MODEM_STATUS_HEADER_LENGTH)
            )

        # Translate raw data into serial port data.
        read = bytearray()
        self._filterStatusBytes(
            buf, read, totalBytesRead, self._read_endpoint.getMaxPacketSize()
        )

        return bytes(read)

    def _set_baudrate(self, baudrate):
        """Change the current UART baudrate.

        The FTDI device is not able to use an arbitrary baudrate. Its
        internal dividors are only able to achieve some baudrates.
        It attemps to find the closest configurable baudrate and if
        the deviation from the requested baudrate is too high, it rejects
        the configuration.

        Parameters:
            baudrate (int): the new baudrate for the UART.

        Raises:
            ValueError: if deviation from selected baudrate is too large.
            SerialException: if not able to set baudrate.
        """
        actual, value, index = self._convert_baudrate(baudrate)
        delta = 100 * abs(float(actual - baudrate)) / baudrate
        if delta > self.BAUDRATE_TOLERANCE:
            raise ValueError(
                "Baudrate tolerance exceeded: %.02f%% "
                "(wanted %d, achievable %d)" % (delta, baudrate, actual)
            )
        result = self._ctrl_transfer_out(self.SIO_SET_BAUDRATE, value, index)
        if result != 0:
            raise SerialException("Unable to set baudrate.")

    def _set_line_property(self, bits, stopbits, parity, break_=0):
        """Set serial port line property.

        Parameters:
            bits (int): number of bits in data(5, 6, 7 or 8).
            stopbits (float): number of stop bits(1, 1.5, 2).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
            break_ (int): 0 for break off or 1 for break on.
        """
        value = bits & 0x0F

        if parity == "N":
            value |= 0x00 << 8
        elif parity == "O":
            value |= 0x01 << 8
        elif parity == "E":
            value |= 0x02 << 8
        elif parity == "M":
            value |= 0x03 << 8
        elif parity == "S":
            value |= 0x04 << 8
        else:
            raise ValueError("Unknown parity value: {}".format(parity))

        if stopbits == 1:
            value |= 0x00 << 11
        elif stopbits == 1.5:
            value |= 0x01 << 11
        elif stopbits == 2:
            value |= 0x02 << 11
        else:
            raise ValueError("Unknown stopbits value: {}".format(stopbits))

        if break_ == self.BREAK_ON:
            value |= 0x01 << 14

        result = self._ctrl_transfer_out(self.SIO_SET_DATA, value, self._index)
        if result != 0:
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

        This function allows the retrieve the two status bytes of the
        device, useful in UART mode.
        FTDI device does not have a so-called USB "interrupt" end-point,
        event polling on the UART interface is done through the regular
        control endpoint.
        Returns:
            status (int): modem status, as a proprietary bitfield
        """
        buf = bytearray(self.DEFAULT_READ_BUFFER_SIZE)
        result = self._ctrl_transfer_in(self.SIO_POLL_MODEM_STATUS, buf, 2)
        if result != 2:
            raise SerialException("Unable to get modem status.")
        buf = buf[:2]
        (status,) = unpack("<H", bytes(buf))
        return status

    def _set_flowctrl(self, flowctrl):
        """Select flowcontrol in UART mode.

        Either hardware flow control through RTS/CTS UART lines,
        software or no flow control.
        Parameters:
            flowctrl (str): 'hw', 'sw' or ''
        """
        ctrl = {
            "hw": self.SIO_RTS_CTS_HS,
            "sw": self.SIO_XON_XOFF_HS,
            "": self.SIO_DISABLE_FLOW_CTRL,
        }
        try:
            value = ctrl[flowctrl] | self._index
        except KeyError:
            raise ValueError("Unknown flow control: {}".format(flowctrl))

        result = self._ctrl_transfer_out(self.SIO_SET_FLOW_CTRL, 0, value)
        if result != 0:
            raise SerialException(
                "Setting flow control failed: result={}".format(result)
            )

    def _has_mpsse(self):
        """Tell whether the device supports MPSSE (I2C, SPI, JTAG, ...).

        Returns:
            result (bool): True if the FTDI device supports MPSSE.
        Raise:
            SerialException: if no FTDI port is open.
        """
        if not self._bcd_device:
            raise SerialException("Device characteristics not yet known!")
        return self._bcd_device in (0x0500, 0x0700, 0x0800, 0x0900)

    def _is_legacy(self):
        """Tell whether the device is a low-end FTDI.

        Returns:
            result (bool): True if the FTDI device can only be used as a slow
            USB-UART bridge.
        Raises:
            SerialException: if no FTDI port is open.
        """
        if not self._bcd_device:
            raise SerialException("Device characteristics not yet known!")
        return self._bcd_device <= 0x0200

    def _is_H_series(self):
        """Tell whether the device is a high-end FTDI.

        Returns:
            result (bool): True if the FTDI device is a high-end USB-UART
            bridge.
        Raises:
            SerialException: if no FTDI port is open.
        """
        if not self._bcd_device:
            raise SerialException("Device characteristics not yet known!")
        return self._bcd_device in (0x0700, 0x0800, 0x0900)

    def _convert_baudrate(self, baudrate):
        """Convert a requested baudrate into the closest possible one.

        Convert a requested baudrate into the closest possible baudrate that
        can be assigned to the FTDI device.

        Parameters:
            baudrate (int): requested baudrate.
        Returns:
            result (tuple): (best_baud, value, index) for baudrate
                configuration.
        """
        if baudrate < ((2 * self.BAUDRATE_REF_BASE) // (2 * 16384 + 1)):
            raise ValueError("Invalid baudrate (too low)")
        if baudrate > self.BAUDRATE_REF_BASE:
            if not self._is_H_series or baudrate > self.BAUDRATE_REF_HIGH:
                raise ValueError("Invalid baudrate (too high)")
            refclock = self.BAUDRATE_REF_HIGH
            hispeed = True
        else:
            refclock = self.BAUDRATE_REF_BASE
            hispeed = False
        # AM legacy device only supports 3 sub-integer dividers, where the
        # other devices supports 8 sub-integer dividers
        am_adjust_up = [0, 0, 0, 1, 0, 3, 2, 1]
        am_adjust_dn = [0, 0, 0, 1, 0, 1, 2, 3]
        # Sub-divider code are not ordered in the natural order
        frac_code = [0, 3, 2, 4, 1, 5, 6, 7]
        divisor = (refclock * 8) // baudrate
        if self._is_legacy:
            # Round down to supported fraction (AM only)
            divisor -= am_adjust_dn[divisor & 7]
        # Try this divisor and the one above it (because division rounds down)
        best_divisor = 0
        best_baud = 0
        best_baud_diff = 0
        for i in range(2):
            try_divisor = divisor + i
            if not hispeed:
                # Round up to supported divisor value
                if try_divisor <= 8:
                    # Round up to minimum supported divisor
                    try_divisor = 8
                elif self._is_legacy and try_divisor < 12:
                    # BM doesn't support divisors 9 through 11 inclusive
                    try_divisor = 12
                elif divisor < 16:
                    # AM doesn't support divisors 9 through 15 inclusive
                    try_divisor = 16
                else:
                    if self._is_legacy:
                        # Round up to supported fraction (AM only)
                        try_divisor += am_adjust_up[try_divisor & 7]
                        if try_divisor > 0x1FFF8:
                            # Round down to maximum supported div value (AM)
                            try_divisor = 0x1FFF8
                    else:
                        if try_divisor > 0x1FFFF:
                            # Round down to maximum supported div value (BM)
                            try_divisor = 0x1FFFF
            # Get estimated baud rate (to nearest integer)
            baud_estimate = ((refclock * 8) + (try_divisor // 2)) // try_divisor
            # Get absolute difference from requested baud rate
            if baud_estimate < baudrate:
                baud_diff = baudrate - baud_estimate
            else:
                baud_diff = baud_estimate - baudrate
            if (i == 0) or (baud_diff < best_baud_diff):
                # Closest to requested baud rate so far
                best_divisor = try_divisor
                best_baud = baud_estimate
                best_baud_diff = baud_diff
                if baud_diff == 0:
                    break
        # Encode the best divisor value
        encoded_divisor = (best_divisor >> 3) | (frac_code[best_divisor & 7] << 14)
        # Deal with special cases for encoded value
        if encoded_divisor == 1:
            encoded_divisor = 0  # 3000000 baud
        elif encoded_divisor == 0x4001:
            encoded_divisor = 1  # 2000000 baud (BM only)
        # Split into "value" and "index" values
        value = encoded_divisor & 0xFFFF
        if self._has_mpsse:
            index = (encoded_divisor >> 8) & 0xFFFF
            index &= 0xFF00
            index |= self._index
        else:
            index = (encoded_divisor >> 16) & 0xFFFF
        if hispeed:
            index |= 1 << 9  # use hispeed mode
        return (best_baud, value, index)

    def _filterStatusBytes(self, src, dest, totalBytesRead, maxPacketSize):
        """Filter FTDI status bytes from buffer.

        Parameters:
            src (bytearray): the source buffer(which contains status bytes).
            dest (bytearray): the destination buffer to write the status bytes
                into(can be src).
            totalBytesRead (int): number of bytes read to src.
            maxPacketSize (int): the USB endpoint max packet size.
        Returns:
            result (int): the number of payload bytes.
        """
        packetsCount = totalBytesRead // maxPacketSize + (
            0 if totalBytesRead % maxPacketSize == 0 else 1
        )
        for packetIdx in range(packetsCount):
            count = (
                ((totalBytesRead % maxPacketSize) - self.MODEM_STATUS_HEADER_LENGTH)
                if (packetIdx == (packetsCount - 1))
                else (maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH)
            )
            if count > 0:
                usb.arraycopy(
                    src,
                    packetIdx * maxPacketSize + self.MODEM_STATUS_HEADER_LENGTH,
                    dest,
                    packetIdx * (maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH),
                    count,
                )

        return totalBytesRead - (packetsCount * 2)
