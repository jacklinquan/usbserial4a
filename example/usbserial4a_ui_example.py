'''usbserial4a example with UI.

This example directly works on Android 6.0+ with Pydroid App.
And it also works on main stream desktop OS like Windows, Linux and OSX.
To make it work on Android 4.0+, please follow the readme file on
https://github.com/jacklinquan/usbserial4a
'''

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.button import Button
from kivy.clock import mainthread
from kivy.utils import platform
import threading

if platform == 'android':
    from usb4a import usb
    from usbserial4a import serial4a
else:
    from serial.tools import list_ports
    from serial import Serial

kv = '''
BoxLayout:
    id: box_root
    orientation: 'vertical'
    
    Label:
        size_hint_y: None
        height: '50dp'
        text: 'usbserial4a example'
    
    ScreenManager:
        id: sm
        on_parent: app.uiDict['sm'] = self
        
        Screen:
            name: 'screen_scan'
            BoxLayout:
                orientation: 'vertical'
                ScrollView:
                    BoxLayout:
                        id: box_list
                        orientation: 'vertical'
                        on_parent: app.uiDict['box_list'] = self
                        size_hint_y: None
                        height: max(self.minimum_height, self.parent.height)
                Button:
                    id: btn_scan
                    on_parent: app.uiDict['btn_scan'] = self
                    size_hint_y: None
                    height: '50dp'
                    text: 'Scan USB Device'
                    on_release: app.on_btn_scan_release()
    
        Screen:
            name: 'screen_test'
            BoxLayout:
                orientation: 'vertical'
                ScrollView:
                    size_hint_y: None
                    height: '50dp'
                    TextInput:
                        id: txtInput_write
                        on_parent: app.uiDict['txtInput_write'] = self
                        size_hint_y: None
                        height: max(self.minimum_height, self.parent.height)
                        text: ''
                Button:
                    id: btn_write
                    on_parent: app.uiDict['btn_write'] = self
                    size_hint_y: None
                    height: '50dp'
                    text: 'Write'
                    on_release: app.on_btn_write_release()
                ScrollView:
                    TextInput:
                        id: txtInput_read
                        on_parent: app.uiDict['txtInput_read'] = self
                        size_hint_y: None
                        height: max(self.minimum_height, self.parent.height)
                        readonly: True
                        text: ''
'''

class MainApp(App):
    def __init__(self, *args, **kwargs):
        self.uiDict = {}
        self.device_name_list = []
        self.serial_port = None
        self.read_thread = None
        super(MainApp, self).__init__(*args, **kwargs)

    def build(self):
        return Builder.load_string(kv)

    def on_stop(self):
        if self.serial_port:
            self.serial_port.close()
        
    def on_btn_scan_release(self):
        self.uiDict['box_list'].clear_widgets()
        self.device_name_list = []
        
        if platform == 'android':
            usb_device_list = usb.get_usb_device_list()
            self.device_name_list = [
                device.getDeviceName() for device in usb_device_list]
        else:
            usb_device_list = list_ports.comports()
            self.device_name_list = [port.device for port in usb_device_list]
        
        for device_name in self.device_name_list:
            btnText = device_name
            button = Button(text=btnText, size_hint_y=None, height='100dp')
            button.bind(on_release=self.on_btn_device_release)
            self.uiDict['box_list'].add_widget(button)
        
    def on_btn_device_release(self, btn):
        device_name = btn.text
        
        if platform == 'android':
            device = usb.get_usb_device(device_name)
            if not device:
                raise SerialException(
                    "Device {} not present!".format(device_name))
            if not usb.has_usb_permission(device):
                usb.request_usb_permission(device)
                return
            self.serial_port = serial4a.get_serial_port(
                device_name,
                9600,
                8,
                'N',
                1,
                timeout=0.5)
        else:
            self.serial_port = Serial(
                device_name,
                9600,
                8,
                'N',
                1,
                timeout=0.5)
        
        if self.serial_port.is_open and not self.read_thread:
            self.read_thread = threading.Thread(target = self.read_msg_thread)
            self.read_thread.start()
        
        self.uiDict['sm'].current = 'screen_test'

    def on_btn_write_release(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(
                bytes(self.uiDict['txtInput_write'].text + '\n'))
            self.uiDict['txtInput_read'].text += '[Sent]{}\n'.format(
                self.uiDict['txtInput_write'].text)
            self.uiDict['txtInput_write'].text = ''
    
    def read_msg_thread(self):
        while True:
            try:
                if not self.serial_port.is_open:
                    break
                received_msg = self.serial_port.read()
                if received_msg:
                    self.display_received_msg(str(received_msg))
            except:
                break
        
    @mainthread
    def display_received_msg(self, msg):
        self.uiDict['txtInput_read'].text += msg

if __name__ == '__main__':
    MainApp().run()

