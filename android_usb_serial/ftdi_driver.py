'''FTDI serial port driver for Kivy Android.

Created on 16 Aug, 2018
License MIT
Author: jacklinquan@gmail.com
'''

from jnius import autoclass, cast
from serial.serialutil import SerialBase, SerialException, SerialTimeoutException

UsbConstants = autoclass('android.hardware.usb.UsbConstants')
Context = autoclass('org.kivy.android.PythonActivity')
ByteBuffer = autoclass('java.nio.ByteBuffer')
UsbRequest = autoclass('android.hardware.usb.UsbRequest')
Intent = autoclass('android.content.Intent')
PendingIntent = autoclass('android.app.PendingIntent')

import logging
log = logging.getLogger("kivy")


def arraycopy(source, sourcepos, dest, destpos, numelem):
    '''Python version of System.arraycopy() in Java
    '''
    dest[destpos:destpos+numelem] = source[sourcepos:sourcepos+numelem]

    
class FtdiSerial(SerialBase):
    """FTDI serial port.
    """
    
    DEFAULT_READ_BUFFER_SIZE = 16 * 1024
    DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024
    
    # Device type
    TYPE_BM, TYPE_AM, TYPE_2232C, TYPE_R, TYPE_2232H, TYPE_4232H = (0, 1, 2, 3, 4, 5)
    
    USB_TYPE_STANDARD = 0x00 << 5
    USB_TYPE_CLASS = 0x00 << 5
    USB_TYPE_VENDOR = 0x00 << 5
    USB_TYPE_RESERVED = 0x00 << 5

    USB_RECIP_DEVICE = 0x00
    USB_RECIP_INTERFACE = 0x01
    USB_RECIP_ENDPOINT = 0x02
    USB_RECIP_OTHER = 0x03

    USB_ENDPOINT_IN = 0x80
    USB_ENDPOINT_OUT = 0x00

    USB_WRITE_TIMEOUT_MILLIS = 5000
    USB_READ_TIMEOUT_MILLIS = 5000
    
    # From ftdi.h
    # Reset the port.
    SIO_RESET_REQUEST = 0
    # Set the modem control register.
    SIO_MODEM_CTRL_REQUEST = 1
    # Set flow control register.
    SIO_SET_FLOW_CTRL_REQUEST = 2
    # Set baud rate.
    SIO_SET_BAUD_RATE_REQUEST = 3
    # Set the data characteristics of the port.
    SIO_SET_DATA_REQUEST = 4
    
    SIO_RESET_SIO = 0
    SIO_RESET_PURGE_RX = 1
    SIO_RESET_PURGE_TX = 2
    
    FTDI_DEVICE_OUT_REQTYPE = UsbConstants.USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT
    FTDI_DEVICE_IN_REQTYPE = UsbConstants.USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN
    
    # Length of the modem status header, transmitted with every read.
    MODEM_STATUS_HEADER_LENGTH = 2
    
    mMaxPacketSize = 64
    
    ENABLE_ASYNC_READS = False
    
    
    def __init__(self, *args, **kwargs):
        self._device = None
        self._device_type = self.TYPE_R
        self._connection = None
        self._interface = None
        self._control_endpoint = None
        self._read_endpoint = None
        self._write_endpoint = None
        super(FtdiSerial, self).__init__(*args, **kwargs)
        
    def filterStatusBytes(self, src, dest, totalBytesRead, maxPacketSize):
        '''Filter FTDI status bytes from buffer
        
        @param bytearray src The source buffer (which contains status bytes)
        @param bytearray dest The destination buffer to write the status bytes into (can be src)
        @param int totalBytesRead Number of bytes read to src
        @param int maxPacketSize The USB endpoint max packet size
        @return int The number of payload bytes
        '''
        
        packetsCount = totalBytesRead // maxPacketSize + (0 if totalBytesRead % maxPacketSize == 0 else 1)
        for packetIdx in range(packetsCount):
            count = (totalBytesRead % maxPacketSize) - self.MODEM_STATUS_HEADER_LENGTH if (packetIdx == (packetsCount - 1)) else maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH
            if count > 0:
                arraycopy(src,
                    packetIdx * maxPacketSize + self.MODEM_STATUS_HEADER_LENGTH,
                    dest,
                    packetIdx * (maxPacketSize - self.MODEM_STATUS_HEADER_LENGTH),
                    count
                )
                
        return totalBytesRead - (packetsCount * 2)
    
    def reset(self):
        result = self._connection.controlTransfer(self.FTDI_DEVICE_OUT_REQTYPE, self.SIO_RESET_REQUEST, self.SIO_RESET_SIO, 0, None, 0, self.USB_WRITE_TIMEOUT_MILLIS)
        if result != 0:
            raise SerialException("Reset failed: result={}".format(result))
    
        self._device_type = self.TYPE_R
        
    def open(self):
        self.close()
        
        activity = cast('android.content.Context', Context.mActivity)
        manager = activity.getSystemService("usb")
        usb_device_list = manager.getDeviceList().values().toArray()
        
        device = None
        log.info("UsbDevices: {}".format(usb_device_list))
        for device in usb_device_list:
            if device and device.getDeviceName()==self.portstr:
                log.info("Found device {}".format(device.getDeviceName()))
                break
        if not device:
            raise SerialException("Device not present {}".format(self.portstr))
        
        ACTION_USB_PERMISSION = "com.access.device.USB_PERMISSION"
        intent = Intent(ACTION_USB_PERMISSION)
        pintent = PendingIntent.getBroadcast(activity, 0, intent, 0)
        if not manager.hasPermission(device):
            manager.requestPermission(device, pintent)
            return
            
        connection = manager.openDevice(device)
        if not connection:
            raise SerialException("Failed to open device!")
                
        log.info("UsbDevice connection made {}!".format(connection))
            
        self._device = device
        self._connection = connection
        
        for i in range(self._device.getInterfaceCount()):
            if i == 0:
                self._interface = self._device.getInterface(i)
            if self._connection.claimInterface(self._device.getInterface(i), True):
                log.info("Claim interface {} successful!".format(i))
            else:
                raise SerialException("Could not claim interface {}.".format(i))
                
        for i in range(self._interface.getEndpointCount()):
            ep = self._interface.getEndpoint(i)
            if ((ep.getDirection() == UsbConstants.USB_DIR_IN) and
                (ep.getType() == UsbConstants.USB_ENDPOINT_XFER_INT)):
                log.debug("Found controlling endpoint({})".format(i))
                self._control_endpoint = ep
            elif ((ep.getDirection() == UsbConstants.USB_DIR_IN) and
                (ep.getType() == UsbConstants.USB_ENDPOINT_XFER_BULK)):
                log.debug("Found reading endpoint({})".format(i))
                self._read_endpoint = ep
            elif ((ep.getDirection() == UsbConstants.USB_DIR_OUT) and
                (ep.getType() == UsbConstants.USB_ENDPOINT_XFER_BULK)):
                log.debug("Found writing endpoint({})".format(i))
                self._write_endpoint = ep  
                
        #: Check that all endpoints are good
        if None in [self._write_endpoint,
                    self._read_endpoint]:
            msg = "Could not establish all endpoints"
            log.debug(msg)
            raise SerialException(msg)
        
        self.is_open = True
        self._reconfigure_port()
        
    def close(self):
        if self._connection:
            self._connection.close()
        self._connection = None
        self.is_open = False
    
    def read(self, data_length):
        if not self.is_open:
            return None
        if not self._read_endpoint:
            raise SerialException("Read endpoint does not exist!")
        
        if self.ENABLE_ASYNC_READS:
            # Not implemented yet.
            return None
        else:
            buf = bytearray(data_length)
            timeout = int(self._timeout * 1000 if self._timeout else self.USB_READ_TIMEOUT_MILLIS)
            totalBytesRead = self._connection.bulkTransfer(self._read_endpoint, buf, data_length, timeout)
            if totalBytesRead < self.MODEM_STATUS_HEADER_LENGTH:
                raise SerialException("Expected at least {} bytes".format(self.MODEM_STATUS_HEADER_LENGTH))
        
            dest = bytearray()
            self.filterStatusBytes(buf, dest, totalBytesRead, self._read_endpoint.getMaxPacketSize())
            return dest
        
    def write(self, data):
        if not self.is_open:
            return None
        offset = 0
        timeout = int(self._write_timeout * 1000 if self._write_timeout else self.USB_WRITE_TIMEOUT_MILLIS)
        wrote = 0
        log.info("AndroidSerial: write data={}, timeout={}".format(data, timeout))
        while offset < len(data):
            data_length = min(len(data) - offset, self.DEFAULT_WRITE_BUFFER_SIZE)
            buf = data[offset:offset + data_length]
            i = self._connection.bulkTransfer(self._write_endpoint,
                                              buf,
                                              data_length,
                                              timeout)
            if i <= 0:
                raise IOError("Failed to write {}: {}".format(buf, i))
            offset += data_length
            wrote += i
        log.info("AndroidSerial: wrote {}".format(wrote))
        return wrote
    
    def convertBaudrate(self, baudrate):
        divisor = 24000000 // baudrate
        bestDivisor = 0;
        bestBaud = 0;
        bestBaudDiff = 0;
        fracCode = [0, 3, 2, 4, 1, 5, 6, 7]
        
        for i in range(2):
            tryDivisor = divisor + i
            if tryDivisor <= 8:
                tryDivisor = 8
            elif self._device_type != self.TYPE_AM and tryDivisor < 12:
                tryDivisor = 12
            elif divisor < 16:
                tryDivisor = 16
            else:
                if self._device_type == self.TYPE_AM:
                    pass
                else:
                    if tryDivisor > 0x1FFFF:
                        tryDivisor = 0x1FFFF
            
            baudEstimate = (24000000 + (tryDivisor // 2)) // tryDivisor
            if baudEstimate < baudrate:
                baudDiff = baudrate - baudEstimate
            else:
                baudDiff = baudEstimate - baudrate
            
            if i == 0 or baudDiff < bestBaudDiff:
                bestDivisor = tryDivisor
                bestBaud = baudEstimate
                bestBaudDiff = baudDiff
                if baudDiff == 0:
                    break
        
        encodedDivisor = (bestDivisor >> 3) | (fracCode[bestDivisor & 7] << 14)
        if encodedDivisor == 1:
            encodedDivisor = 0      # 3000000 baud
        elif encodedDivisor == 0x4001:
            encodedDivisor = 1      # 2000000 baud (BM only)
        
        value = encodedDivisor & 0xFFFF
        if self._device_type in [self.TYPE_2232C, self.TYPE_2232H, self.TYPE_4232H]:
            index = (encodedDivisor >> 8) & 0xFFFF
            index &= 0xFF00
            index |= 0
        else:
            index = (encodedDivisor >> 16) & 0xFFFF
        
        return (bestBaud, index, value)
    
    def setBaudrate(self, baudrate):
        vals = self.convertBaudrate(baudrate)
        actualBaudrate = vals[0]
        index = vals[1]
        value = vals[2]
        
        result = self._connection.controlTransfer(self.FTDI_DEVICE_OUT_REQTYPE, self.SIO_SET_BAUD_RATE_REQUEST, value, index, None, 0, self.USB_WRITE_TIMEOUT_MILLIS)
        if result != 0:
            raise IOError("Setting baudrate failed: result={}".format(result))
        
        return actualBaudrate
        
    def setParameters(self, baudrate, databits, stopbits, parity):
        self.setBaudrate(baudrate)
        
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
        
        result = self._connection.controlTransfer(self.FTDI_DEVICE_OUT_REQTYPE, self.SIO_SET_DATA_REQUEST, config, 0, None, 0, self.USB_WRITE_TIMEOUT_MILLIS)
        if result != 0:
            raise IOError("Setting parameters failed: result={}".format(result))
    
    def _reconfigure_port(self):
        self.setParameters(self.baudrate, self.bytesize , self.stopbits, self.parity)

    
    def purgeHwBuffers(self, purgeReadBuffers, purgeWriteBuffers):
        if purgeReadBuffers:
            result = self._connection.controlTransfer(self.FTDI_DEVICE_OUT_REQTYPE, self.SIO_RESET_REQUEST, self.SIO_RESET_PURGE_RX, 0, None, 0, self.USB_WRITE_TIMEOUT_MILLIS)
            if result != 0:
                raise IOError("Flushing RX failed: result={}".format(result))

        if purgeWriteBuffers:
            result = self._connection.controlTransfer(self.FTDI_DEVICE_OUT_REQTYPE, self.SIO_RESET_REQUEST, self.SIO_RESET_PURGE_TX, 0, None, 0, self.USB_WRITE_TIMEOUT_MILLIS)
            if result != 0:
                raise IOError("Flushing TX failed: result={}".format(result))
        
        return True
        
        
        
    
    
    
    
    
    
    
    
    
    
    
    