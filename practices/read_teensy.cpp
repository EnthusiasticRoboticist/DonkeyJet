#include <iostream>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID 0x16C0   // Vendor ID of Teensy
#define PRODUCT_ID 0x0483  // Product ID of Teensy


void printEndpoint(libusb_device_handle* handle, uint8_t endpoint) {
    libusb_endpoint_descriptor* endpointDesc;
    int result = libusb_get_descriptor(handle, LIBUSB_DT_ENDPOINT, endpoint, reinterpret_cast<unsigned char*>(&endpointDesc), sizeof(libusb_endpoint_descriptor));
    if (result >= 0) {
        std::cout << "Endpoint Descriptor:" << std::endl;
        std::cout << "  Endpoint Address: 0x" << std::hex << static_cast<int>(endpointDesc->bEndpointAddress) << std::endl;
        std::cout << "  Transfer Type: " << static_cast<int>(endpointDesc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) << std::endl;
        std::cout << "  Max Packet Size: " << endpointDesc->wMaxPacketSize << std::endl;
        std::cout << "  Polling Interval: " << static_cast<int>(endpointDesc->bInterval) << std::endl;

        uint8_t stringIndex = endpointDesc->iInterface;
        if (stringIndex > 0) {
            char endpointString[256];
            result = libusb_get_string_descriptor_ascii(handle, stringIndex, reinterpret_cast<unsigned char*>(&endpointString), sizeof(endpointString));
            if (result > 0) {
                std::cout << "  Endpoint String: " << endpointString << std::endl;
            } else {
                std::cerr << "Error getting endpoint string: " << libusb_error_name(result) << std::endl;
            }
        }
    } else {
        std::cerr << "Error getting endpoint descriptor: " << libusb_error_name(result) << std::endl;
    }
}

int main() {
    libusb_device **devices;
    libusb_context *ctx = NULL;
    int result;

    result = libusb_init(&ctx);
    if (result < 0) {
        std::cerr << "Error initializing libusb: " << libusb_error_name(result) << std::endl;
        return 1;
    }

    ssize_t count = libusb_get_device_list(ctx, &devices);
    if (count < 0) {
        std::cerr << "Error getting device list: " << libusb_error_name(count) << std::endl;
        libusb_exit(ctx);
        return 1;
    }

    libusb_device_handle *handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (handle == NULL) {
        std::cerr << "Error opening device." << std::endl;
        libusb_free_device_list(devices, 1);
        libusb_exit(ctx);
        return 1;
    }

    libusb_device* device = libusb_get_device(handle);
    libusb_config_descriptor* configDesc;
    result = libusb_get_active_config_descriptor(device, &configDesc);
    if (result == 0) {
        for (int i = 0; i < configDesc->bNumInterfaces; ++i) {
            const libusb_interface* interface = &(configDesc->interface[i]);
            for (int j = 0; j < interface->num_altsetting; ++j) {
                const libusb_interface_descriptor* interfaceDesc = &(interface->altsetting[j]);
                for (int k = 0; k < interfaceDesc->bNumEndpoints; ++k) {
                    const libusb_endpoint_descriptor* endpointDesc = &(interfaceDesc->endpoint[k]);
                    printEndpoint(handle, endpointDesc->bEndpointAddress);
                }
            }
        }
        libusb_free_config_descriptor(configDesc);
    } else {
        std::cerr << "Error getting config descriptor: " << libusb_error_name(result) << std::endl;
    }

    libusb_close(handle);
    libusb_free_device_list(devices, 1);
    libusb_exit(ctx);

    return 0;
}
