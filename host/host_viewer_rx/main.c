#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

#define VENDOR_ID  0x1d50
#define PRODUCT_ID 0x6089
#define PACKET_SIZE 128
#define TIMEOUT_MS 1000

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *dev_handle = NULL;

    if (libusb_init(&ctx) < 0) return 1;

    dev_handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!dev_handle) {
        fprintf(stderr, "HackRF One not found\n");
        libusb_exit(ctx);
        return 1;
    }

    if (libusb_claim_interface(dev_handle, 0) != 0) {
        fprintf(stderr, "Cannot claim interface 0\n");
        libusb_close(dev_handle);
        libusb_exit(ctx);
        return 1;
    }

    // --- OPEN AS TEXT FILE ---
    FILE *fp = fopen("detected_bits2.txt", "w");
    if (!fp) {
        perror("Failed to open file");
        return 1;
    }

    unsigned char buffer[PACKET_SIZE];
    int transferred;

    printf("=== Recording Bits to detected_bits.txt ===\n");

    while (1) {
        int r = libusb_bulk_transfer(dev_handle, 0x81, buffer, PACKET_SIZE, &transferred, TIMEOUT_MS);

        if (r == 0 && transferred > 0) {
            // Loop through each byte and write its digit character
            for (int i = 0; i < transferred; i++) {
                // This assumes your firmware sends 0x00 or 0x01
                // We add '0' to convert the value 0 to the ASCII character '0'
                fprintf(fp, "%d", buffer[i]);
            }
            
            // Optional: Add a newline after every packet to make it readable
            //fprintf(fp, "\n"); 
            fflush(fp); 
        } 
        else if (r == LIBUSB_ERROR_TIMEOUT) {
            // Just wait for the next attempt
            continue;
        }
        else {
            fprintf(stderr, "\nUSB read error: %d\n", r);
            break; 
        }
    }

    fclose(fp);
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    libusb_exit(ctx);

    return 0;
}