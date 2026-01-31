use ahrs::{Ahrs, Madgwick};
use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::Vector3; // 0.32 for ahrs
use std::io::{BufRead, BufReader};
use std::sync::mpsc::{channel, TryRecvError};
use std::thread;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // Find a serial port
    let ports = serialport::available_ports().expect("No ports found!");
    if ports.is_empty() {
        eprintln!("No serial ports found.");
        return Ok(());
    }

    // Use the first available port
    let port_name = &ports[0].port_name;
    println!("Using serial port: {}", port_name);

    let port = serialport::new(port_name, 115200)
        .timeout(Duration::from_millis(1000))
        .open()?;

    // Use f32 for compatibility with kiss3d
    let (tx, rx) = channel::<[f32; 6]>();

    // Serial reading thread
    thread::spawn(move || {
        let mut reader = BufReader::new(port);
        let mut line = String::new();

        loop {
            line.clear();
            if let Ok(bytes) = reader.read_line(&mut line) {
                if bytes == 0 {
                    continue;
                }
                let line = line.trim();
                let parts: Vec<&str> = line.split(',').collect();
                if parts.len() == 6 {
                    if let (Ok(ax), Ok(ay), Ok(az), Ok(gx), Ok(gy), Ok(gz)) = (
                        parts[0].parse::<f32>(),
                        parts[1].parse::<f32>(),
                        parts[2].parse::<f32>(),
                        parts[3].parse::<f32>(),
                        parts[4].parse::<f32>(),
                        parts[5].parse::<f32>(),
                    ) {
                        let _ = tx.send([ax, ay, az, gx, gy, gz]);
                    }
                }
            }
        }
    });

    // Visualization setup
    let mut window = Window::new("IMU Visualization");
    window.set_light(Light::StickToCamera);

    let mut cube = window.add_cube(1.0, 0.2, 1.5); // Box shape
    cube.set_color(0.0, 1.0, 1.0); // Cyan

    // Madgwick filter setup
    // Sample period: 20ms delay on Arduino -> ~50Hz -> 0.02s
    let sample_period = 0.02f32;
    let beta = 0.1f32;
    let mut madgwick = Madgwick::new(sample_period, beta);

    while window.render() {
        // Process all pending messages
        let mut latest_data = None;
        loop {
            match rx.try_recv() {
                Ok(data) => latest_data = Some(data),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }

        if let Some(data) = latest_data {
            let ax = data[0];
            let ay = data[1];
            let az = data[2];
            // Convert Gyro from Deg/s to Rad/s
            let gx = data[3].to_radians();
            let gy = data[4].to_radians();
            let gz = data[5].to_radians();

            // Madgwick/ahrs uses nalgebra 0.32
            let gyro_vec = Vector3::new(gx, gy, gz);
            let accel_vec = Vector3::new(ax, ay, az);

            // update_imu returns Result<&Quaternion<f32>, ...>
            if let Ok(quat) = madgwick.update_imu(&gyro_vec, &accel_vec) {
                // quat is nalgebra::Quaternion<f32> (0.32)

                // convert to kiss3d::nalgebra::UnitQuaternion (0.30)
                // Bridge manually via components
                let k_quat = kiss3d::nalgebra::Quaternion::new(quat.w, quat.i, quat.j, quat.k);

                // kiss3d's UnitQuaternion::from_quaternion takes just the quaternion
                let k_unit_quat = kiss3d::nalgebra::UnitQuaternion::from_quaternion(k_quat);

                cube.set_local_rotation(k_unit_quat);
            }
        }
    }

    Ok(())
}
