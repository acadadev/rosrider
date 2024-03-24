#include "i2c_hal.h"


uint8_t I2C_RW_Block(int fd, uint8_t reg, uint8_t read_write, uint8_t length, unsigned char* buffer) {

    struct i2c_smbus_ioctl_data ioctl_data;
    union i2c_smbus_data smbus_data;

    // https://github.com/torvalds/linux/blob/master/drivers/i2c/i2c-core-smbus.c
    // (See i2c_smbus_xfer_emulated CASE:I2C_SMBUS_I2C_BLOCK_DATA)

    int rv;

    // notice: commented out on purpose
    // if(length > I2C_SMBUS_BLOCK_MAX) { return -1; }

    // first byte is always the size to write and to receive
    smbus_data.block[0] = length;
    if(read_write != I2C_SMBUS_READ) {
        for(uint8_t i = 0; i < length; i++) {
            smbus_data.block[i + 1] = buffer[i];
        }
    }

    ioctl_data.read_write = read_write;
    ioctl_data.command = reg;
    ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
    ioctl_data.data = &smbus_data;

    rv = ioctl(fd, I2C_SMBUS, &ioctl_data);
    if(rv != 0) { return errno; }               // return ioctl errno

    if(read_write == I2C_SMBUS_READ) {
        for(uint8_t i = 0; i < length; i++) {
            buffer[i] = smbus_data.block[i+1];  // skip first byte
        }
    }

    return rv;

}