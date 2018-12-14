'''Android USB serial CDC ACM driver.

Classes:
CdcAcmSerial(serial.serialutil.SerialBase)
'''

import time
from serial.serialutil import SerialBase, SerialException
from usb4a import usb

class CdcAcmSerial(SerialBase):
    '''CDC ACM serial port class.
    
    CdcAcmSerial extends serial.serialutil.SerialBase.
    It can be used in a similar way to serial.Serial from pyserial.
    '''
    
    USB_RECIP_INTERFACE = 0x01
    USB_RT_ACM = usb.UsbConstants.USB_TYPE_CLASS | USB_RECIP_INTERFACE

    SET_LINE_CODING = 0x20  # USB CDC 1.1 section 6.2
    GET_LINE_CODING = 0x21
    SET_CONTROL_LINE_STATE = 0x22
    SEND_BREAK = 0x23
    
    STOPBIT_MAP = {
        1:0,
        1.5:1,
        2:2}
    PARITY_MAP = {
        'N':0,
        'O':1,
        'E':2,
        'M':3,
        'S':4}
    
    DEFAULT_READ_BUFFER_SIZE = 16 * 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024
    
    USB_WRITE_TIMEOUT_MILLIS = 5000
    USB_READ_TIMEOUT_MILLIS = 5000
    
    def __init__(self, *args, **kwargs):
        self._device = None
        self._connection = None
        self._interface = None
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        self._read_buffer = bytearray()
        super(CdcAcmSerial, self).__init__(*args, **kwargs)
    
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
        
        if self._device.getInterfaceCount()==1:
            # Device might be castrated ACM device. Try single interface logic.
            self._open_single_interface()
        else:
            # Try default interface logic. 
            self._open_interface()

        # Check that all endpoints are good
        if None in [
            self._control_endpoint, 
            self._write_endpoint, 
            self._read_endpoint]:
            raise SerialException("Could not establish all endpoints!")
        
        self.is_open = True
        self._reconfigure_port()
        
    def _open_single_interface(self):
        '''Open single interface device.'''
        device = self._device
        
        # Claiming control/data interface.
        self._control_interface = device.getInterface(0)
        self._data_interface = device.getInterface(0)
        if not self._connection.claimInterface(self._control_interface, True):
            raise SerialException(
                "Could not claim shared control/data interface.")
        
        num_endpoints = self._control_interface.getEndpointCount()
        if num_endpoints < 3:
            msg = "Not enough endpoints - need 3, got {}".format(num_endpoints)
            raise SerialException(msg)
        
        for i in range(num_endpoints):
            ep = self._control_interface.getEndpoint(i)
            if ((ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_INT)):
                # Found controlling endpoint.
                self._control_endpoint = ep
            elif ((ep.getDirection() == usb.UsbConstants.USB_DIR_IN) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK)):
                # Found reading endpoint.
                self._read_endpoint = ep
            elif ((ep.getDirection() == usb.UsbConstants.USB_DIR_OUT) and
                (ep.getType() == usb.UsbConstants.USB_ENDPOINT_XFER_BULK)):
                # Found writing endpoint
                self._write_endpoint = ep  
                
            if None not in [
                self._control_endpoint,
                self._write_endpoint,
                self._read_endpoint]:
                # Found all endpoints.
                break
    
    def _open_interface(self):
        '''Open default interface device.'''
        device = self._device
        
        # Claiming control interface.
        self._control_interface = device.getInterface(0)
        if not self._connection.claimInterface(self._control_interface, True):
            raise SerialException("Could not claim control interface.")
            
        self._control_endpoint = self._control_interface.getEndpoint(0)
        
        # Claiming data interface.
        self._data_interface = device.getInterface(1)
        if not self._connection.claimInterface(self._data_interface, True):
            raise SerialException("Could not claim data interface.")
        
        self._read_endpoint = self._data_interface.getEndpoint(1)
        self._write_endpoint = self._data_interface.getEndpoint(0)
        
    def close(self):
        '''Close the serial port.'''
        if self._connection:
            self._connection.close()
        self._connection = None
        self.is_open = False
    
    def read(self, size=1):
        '''Read data from the serial port.
        
        Parameters:
            size (int): the number of data bytes to read.
    
        Returns:
            read (bytes): data bytes read from the serial port. 
        '''
        if not self.is_open:
            return None
        if not self._read_endpoint:
            raise SerialException("Read endpoint does not exist!")
        
        read = bytearray()
        
        if (size <= len(self._read_buffer)):
            # There is enough data in read buffer.
            # Get data from read buffer.
            read = self._read_buffer[:size]
            self._read_buffer = self._read_buffer[size:]
        else:
            timeout = int(
                self._timeout * 1000 if self._timeout \
                    else self.USB_READ_TIMEOUT_MILLIS)
            
            # Get raw data from hardware.
            buf = bytearray(1024)
            totalBytesRead = self._connection.bulkTransfer(
                self._read_endpoint, 
                buf, 
                1024, 
                timeout)
            if totalBytesRead < 0:
                # Read timeout. Set totalBytesRead to 0.
                totalBytesRead = 0
        
            # Put serial port data into read buffer.
            self._read_buffer.extend(buf[:totalBytesRead])
            # Get data from read buffer.
            read = self._read_buffer[:size]
            self._read_buffer = self._read_buffer[size:]
            
        return bytes(read)
    
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
    
    def flush(self, flush_time=0.2):
        '''Simply wait some time to allow all data to be written.
        
        Parameters:
            flush_time (float): time in seconds to wait.
        '''
        time.sleep(flush_time)
    
    def _send_acm_control_message(self, request, value, buf=None):
        '''USB control transfer.
        
        This function does the USB configuration job.
        '''
        return self._connection.controlTransfer(
            self.USB_RT_ACM, 
            request, 
            value, 
            0, 
            buf,
            0 if buf is None else len(buf),
            self.USB_WRITE_TIMEOUT_MILLIS)
        
    def _reconfigure_port(self):
        '''Reconfigure serial port parameters.'''
        msg = bytearray([
            self.baudrate & 0xff,
            self.baudrate >> 8 & 0xff,
            self.baudrate >> 16 & 0xff,
            self.baudrate >> 24 & 0xff,
            self.STOPBIT_MAP[self.stopbits],
            self.PARITY_MAP[self.parity],
            self.bytesize])
        # Set line coding.
        self._send_acm_control_message(self.SET_LINE_CODING, 0, msg)
        # Set line state.
        value = (0x2 if self._rts_state else 0) \
            or (0x1 if self._dtr_state else 0)
        self._send_acm_control_message(self.SET_CONTROL_LINE_STATE, value)
    
    