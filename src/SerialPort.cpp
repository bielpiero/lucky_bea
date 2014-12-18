#include "SerialPort.h"

const uint16_t SerialPort::vendorId = 0x1ffb;
const size_t SerialPort::nProductIds = 4;
const uint16_t SerialPort::productIds[] = { 0x0089, 0x008a, 0x008b, 0x008c };

SerialPort::SerialPort()
{
    bool found = false;
	libusb_init(&ctx);
	libusb_device** list;
	int count = libusb_get_device_list(ctx, &list);
	
	for (int i = 0; i < count; i++) {
		libusb_device* device = list[i];
                
		struct libusb_device_descriptor desc;
		libusb_get_device_descriptor(device, &desc);
		if (isMaestroDevice(desc)) 
		{
                    //printdev(device);
                    std::cout << "Pololu Maestro device detected..." << std::endl;
                    found = true;
                    libusb_device_handle* current_device;
                    int ret = libusb_open(device, &current_device);
                    device_handle.push_back(current_device);                        
		}
	}
        libusb_free_device_list(list, 1);
        if(!found)
        {
            std::cout << "No Pololu Maestro Device Detected..." << std::endl;
            libusb_exit(ctx);
        }
}

SerialPort::~SerialPort()
{
    for(int i = 0; i< device_handle.size(); i++)
    {
        libusb_close(device_handle[i]);
    }
    libusb_exit(ctx);
}
void SerialPort::printdev(libusb_device *dev) {
    libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
        cout<<"failed to get device descriptor"<<endl;
        return;
    }
    cout<<"Number of possible configurations: "<<(int)desc.bNumConfigurations<< endl;
    cout<<"Device Class: "<<(int)desc.bDeviceClass<< endl;
    cout<<"VendorID: "<<desc.idVendor<< endl;
    cout<<"ProductID: "<<desc.idProduct<<endl;
    libusb_config_descriptor *config;
    libusb_get_config_descriptor(dev, 0, &config);
    cout<<"Interfaces: "<<(int)config->bNumInterfaces<< endl;
    const libusb_interface *inter;
    const libusb_interface_descriptor *interdesc;
    const libusb_endpoint_descriptor *epdesc;
    for(int i=0; i<(int)config->bNumInterfaces; i++) {
        inter = &config->interface[i];
        cout<<"Number of alternate settings: "<<inter->num_altsetting<<endl;
        for(int j=0; j<inter->num_altsetting; j++) {
            interdesc = &inter->altsetting[j];
            cout<<"Interface Number: "<<(int)interdesc->bInterfaceNumber<<" | ";
            cout<<"Number of endpoints: "<<(int)interdesc->bNumEndpoints<<" | ";
            for(int k=0; k<(int)interdesc->bNumEndpoints; k++) {
                epdesc = &interdesc->endpoint[k];
                cout<<"Descriptor Type: "<<(int)epdesc->bDescriptorType<<" | ";
                cout<<"EP Address: "<<(int)epdesc->bEndpointAddress<< endl;
            }
        }
    }
    cout<<endl<<endl<<endl;
    libusb_free_config_descriptor(config);
}

bool SerialPort::isMaestroDevice(libusb_device_descriptor& desc) {
  if (desc.idVendor == vendorId) {
	  for (int i = 0; i < nProductIds; i++) {
		if (desc.idProduct == productIds[i]) {
		  // vendor and product both matched
		  return true;
		}
	  }
  }
  // no product id match
  return false;
}

int SerialPort::controlTransfer(uint8_t cardId, uint8_t request, uint16_t value, uint16_t index) {
  int ret = libusb_control_transfer(
    device_handle[cardId], 0x40, request,
    value, index,
    (unsigned char*) 0, 0, 5000);
  return 0;
}

void SerialPort::setTarget(uint8_t cardId, uint8_t servo, uint16_t value) {
    controlTransfer(cardId, REQUEST_SET_TARGET, value, servo);
}

void SerialPort::setSpeed(uint8_t cardId, uint8_t servo, uint16_t value) {
    controlTransfer(cardId, REQUEST_SET_SERVO_VARIABLE, value, servo);
}

void SerialPort::setAcceleration(uint8_t cardId, uint8_t servo, uint16_t value) {
// setting high bit on servo number indicates acceleration
    controlTransfer(cardId, REQUEST_SET_SERVO_VARIABLE, value, servo | 0x80);
}