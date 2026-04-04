#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x1d50  // HackRF One vendor
#define PRODUCT_ID 0x6089  // HackRF One product
#define PACKET_SIZE 128      // Must match RX_BIT_PACKET_SIZE in firmware
#define TIMEOUT_MS 100      // USB timeout in ms

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *dev_handle = NULL;

    // Initialize libusb
    if (libusb_init(&ctx) < 0) {
        fprintf(stderr, "Failed to initialize libusb\n");
        return 1;
    }

    // Open the HackRF One device
    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle) {
        fprintf(stderr, "HackRF One not found\n");
        libusb_exit(ctx);
        return 1;
    }

    // Claim interface 0
    if (libusb_claim_interface(dev_handle, 0) != 0) {
        fprintf(stderr, "Cannot claim interface 0\n");
        libusb_close(dev_handle);
        libusb_exit(ctx);
        return 1;
    }

    unsigned char buffer[PACKET_SIZE];
    int transferred;

    printf("=== HackRF Bits Viewer ===\n");

    while (1) {
        // Attempt to read a USB bulk packet
        int r = libusb_bulk_transfer(dev_handle, 0x81, buffer, PACKET_SIZE, &transferred, TIMEOUT_MS);

        if (r == 0 && transferred == PACKET_SIZE) {
            // Successfully received a full packet
            for (int i = 0; i < PACKET_SIZE; i++) {
                printf("%d", buffer[i]);
            }
            printf("\n");
            fflush(stdout);
        } 
        else {
            // Other errors
            fprintf(stderr, "USB read error: %d\n", r);
        }
    }

    // Cleanup
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    libusb_exit(ctx);

    return 0;
}