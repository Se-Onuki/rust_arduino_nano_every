mod madgwick;
mod serial;
mod visualization;

use anyhow::Result;
use nalgebra::{Quaternion, Vector3};
use std::sync::mpsc::TryRecvError;
use std::time::Instant;

use visualization::{ImuVisualizer, UserAction};

/*
IMUの座標系:右手系
X: 右
Y: 上
Z: 手前

kiss3dの座標系:右手系
X: 右
Y: 上
Z: 手前

Madgwickアルゴリズムの座標系:右手系
X: 右
Y: 奥
Z: 上
*/

/// IMUの座標系をMadgwick座標系に変換
fn imu_to_madgwick(imu_axis: [f32; 3]) -> Vector3<f32> {
    Vector3::new(imu_axis[0], -imu_axis[2], imu_axis[1])
}

/// Madgwickの姿勢をKiss3dの座標系に変換
fn madgwick_to_kiss3d(madgwick_quat: Quaternion<f32>) -> kiss3d::nalgebra::Quaternion<f32> {
    // Madgwick Frame (X=Right, Y=Back, Z=Up) -> Kiss3d Frame (X=Right, Y=Up, Z=Towards)
    // Mapping: X->X, Y->-Z, Z->Y
    // Quaternion components:
    // new_i = old_i
    // new_j = old_k
    // new_k = -old_j
    kiss3d::nalgebra::Quaternion::new(
        madgwick_quat.w,
        madgwick_quat.i,
        madgwick_quat.k,
        -madgwick_quat.j,
    )
}

fn main() -> Result<()> {
    // シリアル通信の開始
    // 戻り値: (受信チャネル, ポート名)
    // 利用可能なポートがない場合はエラー終了
    let (rx, port_name) = serial::spawn_serial_thread()?;
    println!("Started listening on {}", port_name);

    // 可視化の準備 (ウィンドウ、3軸モデル、カメラ)
    let mut visualizer = ImuVisualizer::new();

    // Madgwickフィルタのセットアップ
    let beta = 0.1f32;
    let mut madgwick = madgwick::Madgwick::new(beta);

    let mut last_packet_time: Option<Instant> = None;

    // メインループ: ウィンドウが閉じられるまで継続
    while visualizer.render() {
        // ユーザー入力の処理
        match visualizer.handle_input() {
            UserAction::Reset => {
                madgwick.reset();
            }
            UserAction::None => {}
        }

        // 保留中のシリアルメッセージをすべて処理
        loop {
            match rx.try_recv() {
                Ok((data, timestamp)) => {
                    let ax = data[0];
                    let ay = data[1];
                    let az = data[2];
                    let gx = data[3].to_radians();
                    let gy = data[4].to_radians();
                    let gz = data[5].to_radians();
                    let mx = data[6];
                    let my = data[7];
                    let mz = data[8];

                    // 座標変換: IMU -> Madgwick
                    let gyro_vec = imu_to_madgwick([gx, gy, gz]);
                    let accel_vec = imu_to_madgwick([ax, ay, az]);
                    let mag_vec = imu_to_madgwick([mx, my, mz]);

                    // dtを計算
                    let dt = if let Some(last) = last_packet_time {
                        timestamp.duration_since(last).as_secs_f32()
                    } else {
                        0.0 // 最初のパケット、dtなし
                    };
                    last_packet_time = Some(timestamp);

                    // dtチェック (安全策)
                    if dt > 0.0 && dt < 1.0 {
                        let quat = madgwick.update(&gyro_vec, &accel_vec, &mag_vec, dt);

                        // 計算結果をKiss3d座標系に変換
                        let k_quat = madgwick_to_kiss3d(quat);

                        // 可視化の更新
                        visualizer.update_orientation(k_quat);
                    }
                }
                Err(TryRecvError::Empty) => {
                    break;
                }
                Err(TryRecvError::Disconnected) => {
                    eprintln!("Serial port disconnected.");
                    return Ok(());
                }
            }
        }
    }

    Ok(())
}
