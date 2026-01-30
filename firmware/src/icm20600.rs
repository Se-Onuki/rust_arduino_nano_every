use embedded_hal::blocking::i2c::{Write, WriteRead};

pub const ICM20600_ADDR: u8 = 0x69; // Or 0x68, need to check schematic, usually 0x69 for Grove

// Registers
const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_GYRO_XOUT_H: u8 = 0x43;

pub struct Icm20600 {
    addr: u8,
}

impl Icm20600 {
    pub fn new() -> Self {
        Icm20600 {
            addr: ICM20600_ADDR,
        }
    }

    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        // Reset
        self.write_reg(i2c, REG_PWR_MGMT_1, 0x80)?;
        // Wait implemented in main
        
        // Wake up
        self.write_reg(i2c, REG_PWR_MGMT_1, 0x01)?; // Auto select clock
        Ok(())
    }

    fn write_reg<I2C, E>(&mut self, i2c: &mut I2C, reg: u8, value: u8) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        i2c.write(self.addr, &[reg, value])
    }
    
    pub fn read_accel_gyro<I2C, E>(&mut self, i2c: &mut I2C) -> Result<([i16; 3], [i16; 3]), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let mut buf = [0u8; 14];
        i2c.write_read(self.addr, &[REG_ACCEL_XOUT_H], &mut buf)?;
        
        let ax = (buf[0] as i16) << 8 | (buf[1] as i16);
        let ay = (buf[2] as i16) << 8 | (buf[3] as i16);
        let az = (buf[4] as i16) << 8 | (buf[5] as i16);
        
        let gx = (buf[8] as i16) << 8 | (buf[9] as i16);
        let gy = (buf[10] as i16) << 8 | (buf[11] as i16);
        let gz = (buf[12] as i16) << 8 | (buf[13] as i16);
        
        Ok(([ax, ay, az], [gx, gy, gz]))
    }
}
