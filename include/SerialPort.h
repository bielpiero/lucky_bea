#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#define REQUEST_SET_TARGET 0x85
#define REQUEST_SET_SERVO_VARIABLE 0x84

#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>


using namespace std;

class SerialPort
{
	private:
		static const uint16_t vendorId;
		static const size_t nProductIds;
		static const uint16_t productIds[];
		static bool isMaestroDevice(libusb_device_descriptor& desc);
		
		uint16_t productId;

		libusb_context* ctx;
		std::vector<libusb_device_handle*> device_handle;
		void printdev(libusb_device *dev);
		int controlTransfer(uint8_t cardId, uint8_t request, uint16_t value, uint16_t index);
	protected:
		int serialPort;
	public:
		SerialPort();
		~SerialPort();
        void setTarget(uint8_t cardId, uint8_t servo, uint16_t value);
        void setSpeed(uint8_t cardId, uint8_t servo, uint16_t value);
        void setAcceleration(uint8_t cardId, uint8_t servo, uint16_t value);
};
#endif