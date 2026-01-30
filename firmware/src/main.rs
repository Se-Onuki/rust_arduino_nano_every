#![no_std]
#![no_main]

use panic_halt as _;
use arduino_hal::prelude::*;
use embedded_hal::blocking::delay::DelayMs;

mod icm20600;
mod ak09918;

use icm20600::Icm20600;
use ak09918::Ak09918;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    
    // Serial setup (115200 baud)
    // For Nano Every (ATmega4809), we might need to use generic serial if board support is tricky, 
    // but arduino-hal-board usually maps default serial.
    // However, Nano Every uses Serial for USB (CDC) via Mux? No, it has dedicated USB chip but 4809 talks UART to it.
    // Usually Serial1 on pins 0/1, but "Serial" (USB) depends on board wiring.
    // On Nano Every, the USB serial is usually USART3 connected to the SAMD11 (programmer).
    // Let's rely on default_serial!() if available, or finding the right peripheral.
    // Checking rahix/avr-hal repo for nano-every, it seems it exposes `usart0` as `serial` (pins 0,1) 
    // and `usart3` connected to the bridge?
    // Let's try standard `arduino_hal::default_serial!`.
    
    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);

    // I2C setup
    // Nano Every I2C is on SDA/SCL pins. 
    // `arduino_hal::I2c::new` requires specific pins.
    // PC2 (SDA), PC3 (SCL) for ATmega4809 usually.
    let mut i2c = arduino_hal::I2c::new(
        dp.TWI0, // TWI0 is default
        pins.a4, // SDA (check pin map, typically A4/A5 on Uno form factor)
        pins.a5, // SCL
        50000,   // 50kHz (conservative) or 100000 
    );

    ufmt::uwriteln!(&mut serial, "Init Sensors...").void_unwrap();

    let mut icm = Icm20600::new();
    let mut ak = Ak09918::new();
    
    // Init ICM
    if let Err(_) = icm.init(&mut i2c) {
        ufmt::uwriteln!(&mut serial, "ICM Init Failed").void_unwrap();
    }
    arduino_hal::delay_ms(100);

    // Init AK
    if let Err(_) = ak.init(&mut i2c) {
        ufmt::uwriteln!(&mut serial, "AK Init Failed").void_unwrap();
    }
    arduino_hal::delay_ms(100);

    ufmt::uwriteln!(&mut serial, "Start Loop").void_unwrap();

    loop {
        let mut success = true;
        
        let (accel, gyro) = match icm.read_accel_gyro(&mut i2c) {
            Ok(val) => val,
            Err(_) => {
                success = false;
                ([0,0,0], [0,0,0])
            }
        };

        let mag = match ak.read_mag(&mut i2c) {
            Ok(val) => val,
            Err(_) => {
                success = false;
                [0,0,0]
            }
        };

        if success {
            // Format: A,ax,ay,az,G,gx,gy,gz,M,mx,my,mz
            ufmt::uwriteln!(
                &mut serial,
                "A,{},{},{},G,{},{},{},M,{},{},{}",
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                mag[0], mag[1], mag[2]
            ).void_unwrap();
        } else {
             ufmt::uwriteln!(&mut serial, "E").void_unwrap();
        }

        arduino_hal::delay_ms(10); // 100Hz approx
    }
}
