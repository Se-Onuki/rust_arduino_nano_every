use embedded_hal::blocking::i2c::{Write, WriteRead};

pub const AK09918_ADDR: u8 = 0x0C;

// Registers
const REG_WIA1: u8 = 0x00;
const REG_WIA2: u8 = 0x01;
const REG_HXL: u8 = 0x11;
const REG_CNTL2: u8 = 0x31;
const REG_CNTL3: u8 = 0x32; // Reset

const MODE_CONTINUOUS_100HZ: u8 = 0x08;

pub struct Ak09918 {
    addr: u8,
}

impl Ak09918 {
    pub fn new() -> Self {
        Ak09918 {
            addr: AK09918_ADDR,
        }
    }

    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        // Reset
        self.write_reg(i2c, REG_CNTL3, 0x01)?;
        // Wait implemented in main
        
        // Continuous mode 100Hz
        self.write_reg(i2c, REG_CNTL2, MODE_CONTINUOUS_100HZ)?;
        Ok(())
    }

    fn write_reg<I2C, E>(&mut self, i2c: &mut I2C, reg: u8, value: u8) -> Result<(), E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        i2c.write(self.addr, &[reg, value])
    }
    
    pub fn read_mag<I2C, E>(&mut self, i2c: &mut I2C) -> Result<[i16; 3], E>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let mut buf = [0u8; 8];
        
        i2c.write_read(self.addr, &[REG_HXL], &mut buf)?;
        
        let mx = (buf[1] as i16) << 8 | (buf[0] as i16);
        let my = (buf[3] as i16) << 8 | (buf[2] as i16);
        let mz = (buf[5] as i16) << 8 | (buf[4] as i16);
        
        Ok([mx, my, mz])
    }
}
