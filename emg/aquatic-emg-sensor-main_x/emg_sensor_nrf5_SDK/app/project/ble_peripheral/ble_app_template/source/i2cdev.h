/**
 * @file i2cdev.h
 * @author Max Chan
 * 
 * I2C Driver API
 */

#ifndef I2CDEV_H
#define I2CDEV_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/cdefs.h>

typedef struct {
    void *controller;
    uint16_t devaddr;
    size_t addrsize;
} i2cdev;

enum {
    I2CDEV_ADDRSIZE_NONE = 0,
    I2CDEV_ADDRSIZE_8BIT = 1,
    I2CDEV_ADDRSIZE_16BIT = 2
}

#define I2CDEV_OPEN(cont, addr, adsz) { .controller = (cont), .devaddr = (addr), .addrsize = (adsz) }

__BEGIN_DECLS

int i2cdev_read(i2cdev *device, uint16_t address, char *buffer, size_t length);
int i2cdev_write(i2cdev *device, uint16_t address, const char *buffer, size_t length);

__END_DECLS

#endif // !defined(I2CDEV_H)