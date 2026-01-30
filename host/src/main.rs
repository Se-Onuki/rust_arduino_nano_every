use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use serialport::SerialPort;
use std::io::{BufRead, BufReader};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// Sensor calibration constants (Example values, user needs to calibrate)
const ACCEL_SCALE: f64 = 1.0; // Raw to G? No, raw to m/s^2?
                              // Madgwick expects rad/s for gyro and m/s^2 or normalized for accel.
                              // If valid MAG, use AHRS.

struct SensorData {
    ax: f64,
    ay: f64,
    az: f64,
    gx: f64,
    gy: f64,
    gz: f64,
    mx: f64,
    my: f64,
    mz: f64,
}

fn main() -> anyhow::Result<()> {
    // Shared rotation quaternion
    let rotation = Arc::new(Mutex::new(UnitQuaternion::identity()));
    let rotation_clone = rotation.clone();

    // Serial Thread
    thread::spawn(move || {
        let ports = serialport::available_ports().expect("No ports found!");
        // Auto-detect or asking user is better, but here we pick the last one (often the plugged device)
        // or filtering by USB VID/PID if known.
        // For now, print ports and pick first that looks like Arduino (COM loop maybe?)
        // Hardcoded or simple logic:
        let port_name = if let Some(port) = ports.first() {
            port.port_name.clone()
        } else {
            eprintln!("No serial ports found.");
            return;
        };

        println!("Opening port: {}", port_name);

        // Wait loop for connection
        let mut port = match serialport::new(&port_name, 115200)
            .timeout(Duration::from_millis(1000))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to open port: {}", e);
                return;
            }
        };

        let mut reader = BufReader::new(port);
        let mut line_buf = String::new();

        let mut madgwick = ahrs::Madgwick::new(0.01, 0.1); // sample_period, beta
                                                           // sample_period should match firmware loop (10ms = 0.01s)

        loop {
            line_buf.clear();
            if let Ok(bytes) = reader.read_line(&mut line_buf) {
                if bytes > 0 {
                    let line = line_buf.trim();
                    if let Some(data) = parse_line(line) {
                        // Update AHRS
                        // Convert raw to units.
                        // ICM20600 default: ±16G (2048 LSB/g), ±2000dps (16.4 LSB/dps)
                        // But I didn't set scale in firmware, defaults are usually ±2g and ±250dps on reset?
                        // I set PWR_MGMT_1 reset = 0x80, then 0x01.
                        // Need to check default scales. Usually:
                        // Accel: ±2g -> 16384 LSB/g
                        // Gyro: ±250dps -> 131 LSB/dps

                        // Let's assume defaults for now.
                        let ascale = 16384.0;
                        let gscale = 131.0;

                        let ax = data.ax / ascale;
                        let ay = data.ay / ascale;
                        let az = data.az / ascale;

                        let gx = (data.gx / gscale).to_radians();
                        let gy = (data.gy / gscale).to_radians();
                        let gz = (data.gz / gscale).to_radians();

                        // Mag: AK09918 is 0.15 uT/LSB ?
                        let mx = data.mx;
                        let my = data.my;
                        let mz = data.mz;

                        // Madgwick update
                        // Input: Gyro (rad/s), Accel (any unit as long as consistent), Mag (any unit)
                        match madgwick.update(
                            &Vector3::new(gx, gy, gz),
                            &Vector3::new(ax, ay, az),
                            &Vector3::new(mx, my, mz),
                        ) {
                            Ok(quat) => {
                                let mut rot = rotation_clone.lock().unwrap();
                                *rot = *quat;
                            }
                            Err(e) => eprintln!("AHRS Error: {:?}", e),
                        }
                    }
                }
            }
        }
    });

    // Kiss3d Window
    let mut window = Window::new("Arduino Nano Every - 9DOF");
    let mut c = window.add_cube(1.0, 0.2, 1.5); // Phone-like shape
    c.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);

    while window.render() {
        let rot = {
            let r = rotation.lock().unwrap();
            *r
        };
        // nalgebra quat to unit quat
        c.set_local_rotation(rot);
    }

    Ok(())
}

fn parse_line(line: &str) -> Option<SensorData> {
    // Format: "A,ax,ay,az,G,gx,gy,gz,M,mx,my,mz"
    let parts: Vec<&str> = line.split(',').collect();
    if parts.len() < 12 {
        return None;
    }

    // Safety check basic tokens
    if parts[0] != "A" || parts[4] != "G" || parts[8] != "M" {
        return None;
    }

    let ax = parts[1].parse::<f64>().ok()?;
    let ay = parts[2].parse::<f64>().ok()?;
    let az = parts[3].parse::<f64>().ok()?;

    let gx = parts[5].parse::<f64>().ok()?;
    let gy = parts[6].parse::<f64>().ok()?;
    let gz = parts[7].parse::<f64>().ok()?;

    let mx = parts[9].parse::<f64>().ok()?;
    let my = parts[10].parse::<f64>().ok()?;
    let mz = parts[11].parse::<f64>().ok()?;

    Some(SensorData {
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz,
    })
}
