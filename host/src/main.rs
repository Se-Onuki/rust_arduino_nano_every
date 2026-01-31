use ahrs::{Ahrs, Madgwick};
use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::Vector3; // 0.32 for ahrs
use std::io::{BufReader, Read};
use std::sync::mpsc::{channel, TryRecvError};
use std::thread;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // シリアルポートを検索
    let ports = serialport::available_ports().expect("No ports found!");
    if ports.is_empty() {
        eprintln!("No serial ports found.");
        return Ok(());
    }

    // USBポートを優先的に検索
    let port_info = ports
        .iter()
        .find(|p| matches!(p.port_type, serialport::SerialPortType::UsbPort(_)))
        .unwrap_or(&ports[0]);

    // 最初の利用可能なポートを使用
    let port_name = &port_info.port_name;
    println!("Using serial port: {}", port_name);

    let port = serialport::new(port_name, 115200)
        .timeout(Duration::from_millis(1000))
        .open()?;
    // kiss3dとの互換性のためにf32を使用
    let (tx, rx) = channel::<[f32; 9]>();

    // シリアル読み取りスレッド
    thread::spawn(move || {
        let mut reader = BufReader::new(port);
        // float(4バイト) * 9 = 36バイト
        let mut buffer = [0u8; 36];

        loop {
            // 正確に36バイト読み取ることを試行
            if let Ok(()) = reader.read_exact(&mut buffer) {
                // float変換 (リトルエンディアン)
                let ax = f32::from_le_bytes(buffer[0..4].try_into().unwrap());
                let ay = f32::from_le_bytes(buffer[4..8].try_into().unwrap());
                let az = f32::from_le_bytes(buffer[8..12].try_into().unwrap());
                let gx = f32::from_le_bytes(buffer[12..16].try_into().unwrap());
                let gy = f32::from_le_bytes(buffer[16..20].try_into().unwrap());
                let gz = f32::from_le_bytes(buffer[20..24].try_into().unwrap());
                let mx = f32::from_le_bytes(buffer[24..28].try_into().unwrap());
                let my = f32::from_le_bytes(buffer[28..32].try_into().unwrap());
                let mz = f32::from_le_bytes(buffer[32..36].try_into().unwrap());

                // 要求通り、生の数値をコンソールに出力
                println!(
                    "A[{:>+7.2}, {:>+7.2}, {:>+7.2}] G[{:>+7.2}, {:>+7.2}, {:>+7.2}] M[{:>+7.2}, {:>+7.2}, {:>+7.2}]",
                    ax, ay, az, gx, gy, gz, mx, my, mz
                );

                let _ = tx.send([ax, ay, az, gx, gy, gz, mx, my, mz]);
            }
        }
    });

    // 可視化のセットアップ
    let mut window = Window::new("IMU Visualization");
    window.set_light(Light::StickToCamera);

    let mut cube = window.add_cube(1.0, 0.2, 1.5); // 箱型
    cube.set_color(0.0, 1.0, 1.0); // シアン

    // カメラを少し離れた位置に作成
    // ArcBallカメラ: eye(視点), at(注視点)
    let mut camera = kiss3d::camera::ArcBall::new(
        kiss3d::nalgebra::Point3::new(0.0, 0.0, 4.0), // 視点位置 z=4.0
        kiss3d::nalgebra::Point3::origin(),           // 原点を見る
    );

    // Madgwickフィルタのセットアップ
    // サンプリング周期: Arduinoの遅延20ms -> 約50Hz -> 0.02s
    let sample_period = 0.02f32;
    let beta = 0.1f32;
    let mut madgwick = Madgwick::new(sample_period, beta);

    while window.render_with_camera(&mut camera) {
        // Process all pending messages
        let mut latest_data = None;
        loop {
            match rx.try_recv() {
                Ok(data) => latest_data = Some(data),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }

        // シリアル読み取りスレッド
        // ... (This part is handled by previous edit, but channel type definition is above it)
        // Wait, the previous edit didn't change the channel definition line 32. I need to change that line too.
        // I will encompass the channel definition in this call? No, it's far apart.
        // I will use multi_replace.

        // ...
        // Using multi_replace is better. But I am in replace_file_content.
        // I will abort this and use multi_replace for clearer dispersed edits.
        // Wait, I already did the loop edit. I just need to fix channel def and the render loop.
        // I will do the channel def and render loop update in this single replace if possible? No, they are far apart.
        // I will do render loop first.

        // ...
        if let Some(data) = latest_data {
            let ax = data[0];
            let ay = data[1];
            let az = data[2];
            // ジャイロを Deg/s から Rad/s に変換
            let gx = data[3].to_radians();
            let gy = data[4].to_radians();
            let gz = data[5].to_radians();
            let mx = data[6];
            let my = data[7];
            let mz = data[8];

            // Madgwick/ahrs は nalgebra 0.32 を使用
            let gyro_vec = Vector3::new(gx, gy, gz);
            let accel_vec = Vector3::new(ax, ay, az);
            let mag_vec = Vector3::new(mx, my, mz);

            // 9軸フュージョン (update) を使用
            // update returns Result<&Quaternion<f32>, ...>
            if let Ok(quat) = madgwick.update(&gyro_vec, &accel_vec, &mag_vec) {
                // quat は nalgebra::Quaternion<f32> (0.32)

                // kiss3d::nalgebra::UnitQuaternion (0.30) に変換
                // コンポーネント経由で手動ブリッジ
                let k_quat = kiss3d::nalgebra::Quaternion::new(quat.w, quat.i, quat.j, quat.k);

                // kiss3dのUnitQuaternion::from_quaternionはクォータニオンのみを受け取る
                let k_unit_quat = kiss3d::nalgebra::UnitQuaternion::from_quaternion(k_quat);

                cube.set_local_rotation(k_unit_quat);
            }
        }
    }

    Ok(())
}
