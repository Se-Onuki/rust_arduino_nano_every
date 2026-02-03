use nalgebra::{Quaternion, Vector3};

/// Madgwick姿勢推定フィルタ。
pub struct Madgwick {
    /// ベータゲイン
    beta: f32,
    /// 推定された姿勢を表すクォータニオン
    quat: Quaternion<f32>,
}

impl Madgwick {
    /// 指定されたゲインで新しいMadgwickインスタンスを作成する
    ///
    /// # 引数
    /// * `beta` - ベータゲイン
    ///
    /// # 返り値
    /// * `Madgwick` - Madgwickインスタンス
    pub fn new(beta: f32) -> Self {
        Self {
            beta,
            quat: Quaternion::new(1.0, 0.0, 0.0, 0.0),
        }
    }

    /// クォータニオンを初期状態（単位クォータニオン）にリセットする
    ///
    /// # 引数
    /// * `self` - Madgwickインスタンス
    pub fn reset(&mut self) {
        self.quat = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    }

    /// 9軸データ（ジャイロ、加速度、磁気）と経過時間 dt でフィルタを更新する
    ///
    /// # 引数
    ///
    /// * `gyro` - ジャイロスコープデータ (ラジアン/秒)
    /// * `accel` - 加速度計データ (正規化の有無に関わらず、内部で正規化されます)
    /// * `mag` - 磁力計データ (正規化の有無に関わらず、内部で正規化されます)
    /// * `dt` - 前回の更新からの経過時間 (秒)
    ///
    /// # 返り値
    /// * `Quaternion<f32>` - Madgwickの姿勢を表すクォータニオン
    pub fn update(
        &mut self,
        gyro: &Vector3<f32>,
        accel: &Vector3<f32>,
        mag: &Vector3<f32>,
        dt: f32,
    ) -> Quaternion<f32> {
        let mut q = self.quat;

        // ジャイロスコープによるクォータニオンの変化率
        let q_dot = 0.5 * q * Quaternion::new(0.0, gyro.x, gyro.y, gyro.z);

        // 加速度計の測定値が有効な場合のみフィードバックを計算
        if accel.norm_squared() > 0.0 && mag.norm_squared() > 0.0 {
            // 加速度計の測定値を正規化
            let a = accel.normalize();
            // 磁力計の測定値を正規化
            let m = mag.normalize();

            // 地磁気の基準方向
            let h = q * Quaternion::new(0.0, m.x, m.y, m.z) * q.conjugate();
            let b = Vector3::new((h.i * h.i + h.j * h.j).sqrt(), 0.0, h.k);

            // 最急降下法による補正ステップ
            let _2q1 = 2.0 * q.w;
            let _2q2 = 2.0 * q.i;
            let _2q3 = 2.0 * q.j;
            let _2q4 = 2.0 * q.k;
            let _2b1 = 2.0 * b.x;
            let _2b3 = 2.0 * b.z;
            let _4b1 = 4.0 * b.x;
            let _4b3 = 4.0 * b.z;

            // 最急降下法
            // f(q, a, b, m)
            // 目的関数の構成要素
            // let q1q1 = q.w * q.w; // 未使用
            let q1q2 = q.w * q.i;
            let q1q3 = q.w * q.j;
            let q1q4 = q.w * q.k;
            let q2q2 = q.i * q.i;
            let q2q3 = q.i * q.j;
            let q2q4 = q.i * q.k;
            let q3q3 = q.j * q.j;
            let q3q4 = q.j * q.k;
            let q4q4 = q.k * q.k;

            // ヤコビアン転置乗算を行ごとに使用して勾配を実装
            // f_g (加速度部分)
            let f_g_x = 2.0 * (q2q4 - q1q3) - a.x;
            let f_g_y = 2.0 * (q1q2 + q3q4) - a.y;
            let f_g_z = 2.0 * (0.5 - q2q2 - q3q3) - a.z;

            // f_b (磁気部分)
            let f_b_x = 2.0 * b.x * (0.5 - q3q3 - q4q4) + 2.0 * b.z * (q2q4 - q1q3) - m.x;
            let f_b_y = 2.0 * b.x * (q2q3 - q1q4) + 2.0 * b.z * (q1q2 + q3q4) - m.y;
            let f_b_z = 2.0 * b.x * (q1q3 + q2q4) + 2.0 * b.z * (0.5 - q2q2 - q3q3) - m.z;

            // 勾配 s = J_g^T * f_g + J_b^T * f_b
            // sの成分を直接計算

            // J_g^T 部分
            // 1行目 (q.w に関して): [-2qy, 2qx, 0]
            let s_g_w = -_2q3 * f_g_x + _2q2 * f_g_y;
            // 2行目 (q.i に関して): [2qz, 2qw, -4qx]
            let s_g_i = _2q4 * f_g_x + _2q1 * f_g_y - 4.0 * q.i * f_g_z;
            // 3行目 (q.j に関して): [-2qw, 2qz, -4qy]
            let s_g_j = -_2q1 * f_g_x + _2q4 * f_g_y - 4.0 * q.j * f_g_z;
            // 4行目 (q.k に関して): [2qx, 2qy, 0]
            let s_g_k = _2q2 * f_g_x + _2q3 * f_g_y;

            // J_b^T 部分
            // 1行目 (q.w に関して): [-2bz*qy, -2bx*qz+2bz*qx, 2bx*qy]
            let s_b_w =
                -_2b3 * q.j * f_b_x + (-_2b1 * q.k + _2b3 * q.i) * f_b_y + _2b1 * q.j * f_b_z;
            // 2行目 (q.i に関して): [2bz*qz, 2bx*qy+2bz*qw, 2bx*qz-4bz*qx]
            let s_b_i = _2b3 * q.k * f_b_x
                + (_2b1 * q.j + _2b3 * q.w) * f_b_y
                + (_2b1 * q.k - _4b3 * q.i) * f_b_z;
            // 3行目 (q.j に関して): [-4bx*qy-2bz*qw, 2bx*qx+2bz*qz, 2bx*qw-4bz*qy]
            let s_b_j = (-4.0 * b.x * q.j - _2b3 * q.w) * f_b_x
                + (_2b1 * q.i + _2b3 * q.k) * f_b_y
                + (_2b1 * q.w - _4b3 * q.j) * f_b_z;
            // 4行目 (q.k に関して): [-4bx*qz+2bz*qi, -2bx*qw+2bz*qj, 2bx*qi]
            let s_b_k = (-4.0 * b.x * q.k + _2b3 * q.i) * f_b_x
                + (-_2b1 * q.w + _2b3 * q.j) * f_b_y
                + _2b1 * q.i * f_b_z;

            // 全体の勾配
            let mut s = Quaternion::new(s_g_w + s_b_w, s_g_i + s_b_i, s_g_j + s_b_j, s_g_k + s_b_k);

            // sを正規化
            if s.norm_squared() > 0.0 {
                s = s.normalize();

                // フィードバックを適用
                let q_dot_hat = q_dot - (self.beta * s);

                // 積分
                q = q + (q_dot_hat * dt);
            } else {
                q = q + (q_dot * dt);
            }
        } else {
            q = q + (q_dot * dt);
        }

        // クォータニオンを正規化
        self.quat = q.normalize();
        self.quat
    }
}
