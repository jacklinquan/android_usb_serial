from jnius import autoclass, cast

Context = autoclass('org.kivy.android.PythonActivity')

def get_usb_device_list():
    '''Get USB device list.
    
    Usage:
    usb_device_list[0].getDeviceName()
    usb_device_list[0].getVendorId()
    usb_device_list[0].getManufacturerName()
    usb_device_list[0].getProductId()
    usb_device_list[0].getProductName()
    '''
    activity = cast('android.content.Context', Context.mActivity)
    manager = activity.getSystemService("usb")
    usb_device_list = manager.getDeviceList().values().toArray()
    return usb_device_list
    