# Copyright (C) 2006, 2007  Douglas P Lau
# Copyright (C) 2009  Marcus Wanner
# Copyright (C) 2011  Paul Hollensen, Marcus Wanner
#
# Modified by @paulaksm in June, 2018
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import usb, os
import struct 

ID_VENDOR_LEGO = 0x0694
ID_PRODUCT_NXT = 0x0002

IN_ENDPOINT  = 0x82
OUT_ENDPOINT = 0x01

NXT_CONFIGURATION = 1
NXT_INTERFACE     = 0

class USBInterface(object):
    """
    Class to communicate with a USB device

    :param device: USB device connected
    :type device: usb.core.Device
    :param debug: flag to turn verbose mode on (default=False)
    :type debug: bool
    """

    id2type = {'f': float, 'i': int, '?': bool, 's': str}
    type2id = { id2type[val_id]:val_id for val_id in id2type.keys() }

    bsize = 60  

    type = 'usb'

    def __init__(self, 
                 device, 
                 debug=False):
        self.device = device
        self.debug = debug

    def __str__(self):
        return "USB {}".format(self.device)

    def connect(self):
        """
        Connect to device
        """
        if self.debug:
            print ('Connecting via USB...')
        try:
            if self.device.is_kernel_driver_active(NXT_INTERFACE):
                self.device.detach_kernel_driver(NXT_INTERFACE)
            self.device.set_configuration(NXT_CONFIGURATION)
        except Exception as err:
            if self.debug:
                print ('ERROR:usbsock:connect', err)
            raise
        if self.debug:
            print ('Connected.')

    def close(self):
        """
        Close connection
        """
        if self.debug:
            print ('Closing USB connection...')
        self.device = None
        if self.debug:
            print ('USB connection closed.')

    def send(self, 
             data, 
             dtype=None):
        """
        Send data to device, packed using big-endian convention by default.
        Float data can be packed as float or double; if double the argument dtype must be dtype='d'
        If dtype is not passed (dtype=None), the class type is infered from the data itself.
        
        dtype='?' - bool
        dtype='i' - int
        dtype='f' - float
        dtype='d' - double

        :param data: data to be sent
        :type data: int, float or bool
        :param dtype: type of the data to be sent (default=None)
        :type dtype: str  
        """ 
        assert isinstance(data, str) or dtype == 's', "Can't send string data"
        if self.debug:
            print ('Send:',end=" ")
            print (': {}'.format(ord(data)))
        if dtype is None:
            if isinstance(data, float):
                dtype = 'f'
            elif isinstance(data, bool):
                dtype = '?'
            elif isinstance(data, int):
                dtype = 'i'
            else:
                print("Data type not recognized as int, bool or float...")
                return -1
        elif dtype != 'd':
            assert isinstance(data, id2type[dtype]), "Value for dtype doesn't correspond to the type of data parameter"
        else:
            assert isinstance(data, float), "Expected 'd' indicating a double and data of type float"
        encode = struct.pack('>{}'.format(dtype), data)
        self.device.write(OUT_ENDPOINT, encode, NXT_INTERFACE) 

    def recv(self, 
             dtype='i'):
        """
        Recieve data from device of a specific dtype using big-endian convention. 
        
        dtype='?' - bool
        dtype='i' - int
        dtype='f' - float
        dtype='d' - double
        dtype='s' - str

        :param dtype: type of the data to be sent (default='i')
        :type dtype: str  
        """ 
        data = self.device.read(IN_ENDPOINT, 64, 1000)
        if self.debug:
            print ('Recv:', end=" ")
            print (':'.join('{}'.format(c & 0xFF) for c in data))
        if dtype == 's':
            return ''.join(chr(d & 0xFF) for d in data)
        decode = struct.unpack('>{}'.format(dtype), data)
        return decode[0]

def find_bricks(debug=False):
    """
    Returns a generator with the located USB devices that corresponds to Lego NXT brick specification.  

    :param debug: flag to turn verbose mode on (default=False)
    :type debug: bool 
    """  
    for device in usb.core.find(find_all=True, idVendor=ID_VENDOR_LEGO, idProduct=ID_PRODUCT_NXT):
        yield USBInterface(device, debug)
