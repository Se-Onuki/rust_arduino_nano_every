use std::io::{BufReader, Read};
use std::sync::mpsc::{channel, Receiver};
use std::thread;
use std::time::{Duration, Instant};

/// シリアル通信を管理し、データ受信スレッドを開始します。
///
/// # Returns
///
/// * `anyhow::Result<(Receiver<([f32; 9], Instant)>, String)>` - データ受信チャネルとポート名、またはエラー
pub fn spawn_serial_thread() -> anyhow::Result<(Receiver<([f32; 9], Instant)>, String)> {
    // シリアルポートを検索
    let ports = serialport::available_ports().expect("No ports found!");
    if ports.is_empty() {
        return Err(anyhow::anyhow!("No serial ports found."));
    }

    // USBポートを優先的に検索
    let port_info = ports
        .iter()
        .find(|p| matches!(p.port_type, serialport::SerialPortType::UsbPort(_)))
        .unwrap_or(&ports[0]);

    let port_name = port_info.port_name.clone();
    println!("Connecting to serial port: {}", port_name);

    let port = serialport::new(&port_name, 115200)
        .timeout(Duration::from_millis(1000))
        .open()?;

    let (tx, rx) = channel::<([f32; 9], Instant)>();

    // シリアル読み取りスレッド
    thread::spawn(move || {
        let mut reader = BufReader::new(port);
        // float(4バイト) * 9 = 36バイト
        let mut buffer = [0u8; 36];

        loop {
            // 36バイト読み取ることの試行
            if let Ok(()) = reader.read_exact(&mut buffer) {
                let now = Instant::now(); // 時刻の記録

                // float変換 (LE:リトルエンディアン)
                let ax = f32::from_le_bytes(buffer[0..4].try_into().unwrap());
                let ay = f32::from_le_bytes(buffer[4..8].try_into().unwrap());
                let az = f32::from_le_bytes(buffer[8..12].try_into().unwrap());
                let gx = f32::from_le_bytes(buffer[12..16].try_into().unwrap());
                let gy = f32::from_le_bytes(buffer[16..20].try_into().unwrap());
                let gz = f32::from_le_bytes(buffer[20..24].try_into().unwrap());
                let mx = f32::from_le_bytes(buffer[24..28].try_into().unwrap());
                let my = f32::from_le_bytes(buffer[28..32].try_into().unwrap());
                let mz = f32::from_le_bytes(buffer[32..36].try_into().unwrap());

                // 生の数値をコンソールに出力 (デバッグ用、必要に応じてコメントアウト)
                /*
                println!(
                    "A[{:>+7.2}, {:>+7.2}, {:>+7.2}] G[{:>+7.2}, {:>+7.2}, {:>+7.2}] M[{:>+7.2}, {:>+7.2}, {:>+7.2}]",
                    ax, ay, az, gx, gy, gz, mx, my, mz
                );
                */

                let _ = tx.send(([ax, ay, az, gx, gy, gz, mx, my, mz], now));
            } else {
                // 読み取りエラー時は少し待つか、エラー処理をする
                // ここでは単純にループを継続
            }
        }
    });

    Ok((rx, port_name))
}
