/**
 * @file i2cdev.c
 * @author Max Chan
 * 
 * I2C Driver API
 */

#include "i2cdev.h"
#include "nrf_dev_twi.h"
#include <string.h>

#define MIN(x, y) (((x) > (y)) ? (y) : (x))

int i2cdev_read(i2cdev *device, uint16_t address, char *buffer, size_t length) {
    char buf[2] = {(address >> 8) & 0xff, address & 0xff};
    switch (device->addrsize) {
        case I2CDEV_ADDRSIZE_16BIT:
            nrf_drv_twi_tx(device->controller, device->devaddr, buf, 2, true);
            break;
        case I2CDEV_ADDRSIZE_8BIT:
            nrf_drv_twi_tx(device->controller, device->devaddr, buf + 1, 1, true);
            break;
    }

    return nrf_drv_twi_rx(device->controller, device->devaddr, buffer, length);
}

int i2cdev_write(i2cdev *device, uint16_t address, const char *buffer, size_t length) {
    char buf[64];
    char *cp = buf;
    size_t len = MIN(64 - device->addrsize, length);

    switch (device->addrsize) {
        case I2CDEV_ADDRSIZE_16BIT:
            *cp = (address >> 8) & 0xff;
            cp++;
        case I2CDEV_ADDRSIZE_8BIT:
            *cp = address & 0xff;
            cp++;
        case I2CDEV_ADDRSIZE_NONE:
            break;
    }

    memcpy(cp, buffer, len);

    return nrf_drv_twi_tx(device->controller, device->devaddr, buffer, len + 1, false);
}