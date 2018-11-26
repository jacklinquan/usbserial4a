'''Android USB serial FTDI driver.

Classes:
FtdiSerial(serial.serialutil.SerialBase)
'''

from serial.serialutil import SerialBase, SerialException
from usb4a import usb

class FtdiSerial(SerialBase):
    '''FTDI serial port class.
    
    FtdiSerial extends serial.serialutil.SerialBase.
    It can be used in a similar way to serial.Serial from pyserial.
    '''
    
    # Requests
    SIO_RESET = 0               # Reset the port
    SIO_SET_MODEM_CTRL = 1      # Set the modem control register
    SIO_SET_FLOW_CTRL = 2       # Set flow control register
    SIO_SET_BAUDRATE = 3        # Set baud rate
    SIO_SET_DATA = 4            # Set the data characteristics of the port
    SIO_POLL_MODEM_STATUS = 5   # Get line status
    SIO_SET_EVENT_CHAR = 6      # Change event character
    SIO_SET_ERROR_CHAR = 7      # Change error character
    SIO_SET_LATENCY_TIMER = 9   # Change latency timer
    SIO_GET_LATENCY_TIMER = 10  # Get latency timer
    SIO_SET_BITMODE = 11        # Change bit mode
    SIO_READ_PINS = 12          # Read GPIO pin value

    # Eeprom requests
    SIO_EEPROM = 0x90
    SIO_READ_EEPROM = SIO_EEPROM + 0   # Read EEPROM content
    SIO_WRITE_EEPROM = SIO_EEPROM + 1  # Write EEPROM content
    SIO_ERASE_EEPROM = SIO_EEPROM + 2  # Erase EEPROM content

    # Reset commands
    SIO_RESET_SIO = 0          # Reset device
    SIO_RESET_PURGE_RX = 1     # Drain RX buffer
    SIO_RESET_PURGE_TX = 2     # Drain TX buffer

    # Flow control
    SIO_DISABLE_FLOW_CTRL = 0x0
    SIO_RTS_CTS_HS = (0x1 << 8)
    SIO_DTR_DSR_HS = (0x2 << 8)
    SIO_XON_XOFF_HS = (0x4 << 8)
    SIO_SET_DTR_MASK = 0x1
    SIO_SET_DTR_HIGH = (SIO_SET_DTR_MASK | (SIO_SET_DTR_MASK << 8))
    SIO_SET_DTR_LOW = (0x0 | (SIO_SET_DTR_MASK << 8))
    SIO_SET_RTS_MASK = 0x2
    SIO_SET_RTS_HIGH = (SIO_SET_RTS_MASK | (SIO_SET_RTS_MASK << 8))
    SIO_SET_RTS_LOW = (0x0 | (SIO_SET_RTS_MASK << 8))
    
    # Clocks and baudrates
    BUS_CLOCK_BASE = 6.0E6  # 6 MHz
    BUS_CLOCK_HIGH = 30.0E6  # 30 MHz
    BAUDRATE_REF_BASE = int(3.0E6)  # 3 MHz
    BAUDRATE_REF_HIGH = int(12.0E6)  # 12 MHz
    BAUDRATE_REF_SPECIAL = int(2.0E6)  # 3 MHz
    BAUDRATE_TOLERANCE = 3.0  # acceptable clock drift, in %
    BITBANG_CLOCK_MULTIPLIER = 4
    
    DEFAULT_READ_BUFFER_SIZE = 16 * 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024
    
    FTDI_DEVICE_OUT_REQTYPE = usb.build_usb_control_request_type(
        usb.UsbConstants.USB_DIR_OUT, 
        usb.UsbConstants.USB_TYPE_VENDOR, 
        usb.USB_RECIPIENT_DEVICE)
    FTDI_DEVICE_IN_REQTYPE = usb.build_usb_control_request_type(
        usb.UsbConstants.USB_DIR_IN, 
        usb.UsbConstants.USB_TYPE_VENDOR, 
        usb.USB_RECIPIENT_DEVICE)
    
    USB_WRITE_TIMEOUT_MILLIS = 5000
    USB_READ_TIMEOUT_MILLIS = 5000
    
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
        super(FtdiSerial, self).__init__(*args, **kwargs)
    
    def open(self):
        '''Open the serial port.
        
        When the serial port is instantiated, it will try to open automatically.
        '''
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
            if not self._connection.claimInterface(
                self._device.getInterface(i), 
                True):
                raise SerialException("Could not claim interface {}.".format(i))
        
        self._index = self._interface.getId() + 1
        
        for i in range(self._interface.getEndpointCount()):
            ep = self._interface.getEndpoint(i)
            if ((ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_INT)):
                self._control_endpoint = ep
            elif ((ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK)):
                self._read_endpoint = ep
            elif ((ep.getDirection() == usb.UsbConstants.USB_DIR_OUT) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK)):
                self._write_endpoint = ep  
                
        #: Check that all endpoints are good
        if None in [self._write_endpoint, self._read_endpoint]:
            raise SerialException("Could not establish all endpoints!")
        
        self.is_open = True
        self._reconfigure_port()
        
    def close(self):
        '''Close the serial port.'''
        if self._connection:
            self._connection.close()
        self._connection = None
        self.is_open = False
    
    def reset(self):
        '''Reset the serial port.'''
        if self._connection:
            result = self._ctrl_transfer_out(
                self.SIO_RESET, 
                self.SIO_RESET_SIO, 
                0)
            if result != 0:
                raise SerialException("Reset failed: result={}".format(result))
    
    def read(self, data_length=1):
        '''Read data from the serial port.
        
        This function should be used in another thread, as it blocks.
        
        Parameters:
            data_length (int): the length of read buffer(better <= 1024).
    
        Returns:
            dest (bytearray): data read from the serial port. 
        '''
        if not self.is_open:
            return None
        if not self._read_endpoint:
            raise SerialException("Read endpoint does not exist!")
        
        buf = bytearray(data_length)
        timeout = int(
            self._timeout * 1000 if self._timeout \
                else self.USB_READ_TIMEOUT_MILLIS)
        totalBytesRead = self._connection.bulkTransfer(
            self._read_endpoint, 
            buf, 
            data_length, 
            timeout)
        if totalBytesRead < self.MODEM_STATUS_HEADER_LENGTH:
            raise SerialException(
                "Expected at least {} bytes".format(
                    self.MODEM_STATUS_HEADER_LENGTH))
    
        dest = bytearray()
        self._filterStatusBytes(
            buf, 
            dest, 
            totalBytesRead, 
            self._read_endpoint.getMaxPacketSize())
        return dest
    
    def write(self, data):
        '''Write data to the serial port.
        
        Parameters:
            data (bytearray): data written to the serial port.
    
        Returns:
            wrote (int): the number of data bytes written. 
        '''
        if not self.is_open:
            return None
        offset = 0
        timeout = int(
            self._write_timeout * 1000 if self._write_timeout \
                else self.USB_WRITE_TIMEOUT_MILLIS)
        wrote = 0
        while offset < len(data):
            data_length = min(
                len(data) - offset, 
                self.DEFAULT_WRITE_BUFFER_SIZE)
            buf = data[offset:offset + data_length]
            i = self._connection.bulkTransfer(
                self._write_endpoint,
                buf,
                data_length,
                timeout)
            if i <= 0:
                raise SerialException("Failed to write {}: {}".format(buf, i))
            offset += data_length
            wrote += i
        return wrote
    
    def purgeHwBuffers(self, purgeReadBuffers, purgeWriteBuffers):
        '''Set serial port parameters.
        
        Parameters:
            purgeReadBuffers (bool): need to purge read buffer or not.
            purgeWriteBuffers (bool): need to purge write buffer or not.
        Returns:
            result (bool): successful or not.
        '''
        if purgeReadBuffers:
            result = self._ctrl_transfer_out(
                self.SIO_RESET, 
                self.SIO_RESET_PURGE_RX, 
                0)
            if result != 0:
                raise SerialException(
                    "Flushing RX failed: result={}".format(result))

        if purgeWriteBuffers:
            result = self._ctrl_transfer_out(
                self.SIO_RESET, 
                self.SIO_RESET_PURGE_TX, 
                0)
            if result != 0:
                raise SerialException(
                    "Flushing TX failed: result={}".format(result))
        
        return True
    
    def _set_baudrate(self, baudrate):
        '''Change the current UART baudrate.
        
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
        '''
        actual, value, index = self._convert_baudrate(baudrate)
        delta = 100*abs(float(actual-baudrate))/baudrate
        if delta > self.BAUDRATE_TOLERANCE:
            raise ValueError('Baudrate tolerance exceeded: %.02f%% '
                             '(wanted %d, achievable %d)' %
                             (delta, baudrate, actual))
        result = self._ctrl_transfer_out(self.SIO_SET_BAUDRATE, value, index)
        if result != 0:
            raise SerialException('Unable to set baudrate')
    
    def _setParameters(self, baudrate, databits, parity, stopbits):
        '''Set serial port parameters.
        
        Parameters:
            baudrate (int): the new baudrate for the UART(eg 9600).
            databits (int): number of bits in data(5, 6, 7 or 8).
            parity (str): 'N', 'E', 'O', 'M' or 'S'.
            stopbits (float): number of stop bits(1, 1.5, 2).
        '''
    
        self._set_baudrate(baudrate)
        
        config = databits
        
        if parity == 'N':
            config |= (0x00 << 8)
        elif parity == 'O':
            config |= (0x01 << 8)
        elif parity == 'E':
            config |= (0x02 << 8)
        elif parity == 'M':
            config |= (0x03 << 8)
        elif parity == 'S':
            config |= (0x04 << 8)
        else:
            raise ValueError("Unknown parity value: {}".format(parity))
        
        if stopbits == 1:
            config |= (0x00 << 11)
        elif stopbits == 1.5:
            config |= (0x01 << 11)
        elif stopbits == 2:
            config |= (0x02 << 11)
        else:
            raise ValueError("Unknown stopbits value: {}".format(stopbits))
        
        result = self._ctrl_transfer_out(self.SIO_SET_DATA, config, 0)
        if result != 0:
            raise SerialException(
                "Setting parameters failed: result={}".format(result))
    
    def _reconfigure_port(self):
        '''Reconfigure serial port parameters.'''
        self._setParameters(
            self.baudrate, 
            self.bytesize, 
            self.parity, 
            self.stopbits)
    
    def _ctrl_transfer_out(self, request, value, index):
        '''USB control transfer.
        
        This function does the USB configuration job.
        '''
        return self._connection.controlTransfer(
            self.FTDI_DEVICE_OUT_REQTYPE, 
            request, 
            value, 
            index, 
            None, 
            0, 
            self.USB_WRITE_TIMEOUT_MILLIS)
    
    def _has_mpsse(self):
        '''Tell whether the device supports MPSSE (I2C, SPI, JTAG, ...).
        
        Returns:
            result (bool): True if the FTDI device supports MPSSE.
        Raise:
            SerialException: if no FTDI port is open.
        '''
        if not self._bcd_device:
            raise SerialException('Device characteristics not yet known!')
        return self._bcd_device in (0x0500, 0x0700, 0x0800, 0x0900)
    
    def _is_legacy(self):
        '''Tell whether the device is a low-end FTDI.
        
        Returns:
            result (bool): True if the FTDI device can only be used as a slow
            USB-UART bridge.
        Raises:
            SerialException: if no FTDI port is open.
        '''
        if not self._bcd_device:
            raise SerialException('Device characteristics not yet known!')
        return self._bcd_device <= 0x0200

    def _is_H_series(self):
        '''Tell whether the device is a high-end FTDI.
        
        Returns:
            result (bool): True if the FTDI device is a high-end USB-UART 
            bridge.
        Raises:
            SerialException: if no FTDI port is open.
        '''
        if not self._bcd_device:
            raise SerialException('Device characteristics not yet known!')
        return self._bcd_device in (0x0700, 0x0800, 0x0900)
    
    def _convert_baudrate(self, baudrate):
        '''Convert a requested baudrate into the closest possible one.
        
        Convert a requested baudrate into the closest possible baudrate that
        can be assigned to the FTDI device.
        
        Parameters:
            baudrate (int): requested baudrate.
        Returns:
            result (tuple): (best_baud, value, index) for baudrate 
                configuration.
        '''
        if baudrate < ((2*self.BAUDRATE_REF_BASE)//(2*16384+1)):
            raise ValueError('Invalid baudrate (too low)')
        if baudrate > self.BAUDRATE_REF_BASE:
            if not self._is_H_series or \
                    baudrate > self.BAUDRATE_REF_HIGH:
                raise ValueError('Invalid baudrate (too high)')
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
        divisor = (refclock*8) // baudrate
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
                elif self._is_legacy and \
                        try_divisor < 12:
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
            baud_estimate = ((refclock*8) + (try_divisor//2))//try_divisor
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
        encoded_divisor = (best_divisor >> 3) | \
                          (frac_code[best_divisor & 7] << 14)
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
        '''Filter FTDI status bytes from buffer.
        
        Parameters:
            src (bytearray): the source buffer(which contains status bytes).
            dest (bytearray): the destination buffer to write the status bytes
                into(can be src).
            totalBytesRead (int): number of bytes read to src.
            maxPacketSize (int): the USB endpoint max packet size.
        Returns:
            result (int): the number of payload bytes.
        '''
        packetsCount = totalBytesRead // maxPacketSize + (
            0 if totalBytesRead % maxPacketSize == 0 else 1)
        for packetIdx in range(packetsCount):
            count = (totalBytesRead % maxPacketSize) - \
                self.MODEM_STATUS_HEADER_LENGTH if (
                    packetIdx == (packetsCount - 1)) else \
                    maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH
            if count > 0:
                usb.arraycopy(
                    src,
                    packetIdx * maxPacketSize + self.MODEM_STATUS_HEADER_LENGTH,
                    dest,
                    packetIdx * (
                        maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH),
                    count)
                
        return totalBytesRead - (packetsCount * 2)
        