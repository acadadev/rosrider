#ifndef __I2C_HAL_h
#define __I2C_HAL_h

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_RETRIES 0x0701
#define I2C_TIMEOUT 0x0702  
#define I2C_SMBUS 0x0720

#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

#define I2C_SMBUS_QUICK             0
#define I2C_SMBUS_BYTE              1
#define I2C_SMBUS_BYTE_DATA         2 
#define I2C_SMBUS_WORD_DATA         3
#define I2C_SMBUS_PROC_CALL         4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7
#define I2C_SMBUS_I2C_BLOCK_DATA    8

#define I2C_SMBUS_BLOCK_MAX         32      
#define I2C_SMBUS_I2C_BLOCK_MAX     32

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <errno.h>

extern uint8_t I2C_RW_Block(int fd, uint8_t reg, uint8_t read_write, uint8_t length, unsigned char* buffer);

#ifdef __cplusplus
}
#endif

#endif