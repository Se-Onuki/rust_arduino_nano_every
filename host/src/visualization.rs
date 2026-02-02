use kiss3d::event::{Action, Key, MouseButton, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Translation3, UnitQuaternion, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;
use std::rc::Rc;

/// ユーザーからの入力アクション
pub enum UserAction {
    // 入力なし
    None,
    // リセット
    Reset,
}

/// 可視化とユーザー入力を管理する構造体
pub struct ImuVisualizer {
    // ウィンドウ
    window: Window,
    // カメラ
    camera: kiss3d::camera::ArcBall,
    // 3軸モデル
    axes_group: SceneNode,
    // テキスト用フォント
    font: Rc<Font>,
}

impl ImuVisualizer {
    /// 新しいVisualizerを作成し、ウィンドウと3軸モデルを初期化する
    pub fn new() -> Self {
        let mut window = Window::new("IMU Visualization");
        window.set_light(Light::StickToCamera);

        // 軸モデルの作成
        let mut axes_group = window.add_group();

        // X軸 (赤) - Kiss3dのデフォルトY-upからZ軸周りに-90度回転してX軸方向へ
        let mut x_axis = axes_group.add_cylinder(0.05, 1.0);
        x_axis.set_color(1.0, 0.0, 0.0);
        let rot_x =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -std::f32::consts::FRAC_PI_2);
        x_axis.set_local_rotation(rot_x);
        x_axis.set_local_translation(Translation3::new(0.5, 0.0, 0.0));

        // Y軸 (緑) - Kiss3dのY (上)
        let mut y_axis = axes_group.add_cylinder(0.05, 1.0);
        y_axis.set_color(0.0, 1.0, 0.0);
        y_axis.set_local_translation(Translation3::new(0.0, 0.5, 0.0));

        // Z軸 (青) - Kiss3dのZ (手前) - Y-upからX軸周りに90度回転
        let mut z_axis = axes_group.add_cylinder(0.05, 1.0);
        z_axis.set_color(0.0, 0.0, 1.0);
        let rot_z =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_2);
        z_axis.set_local_rotation(rot_z);
        z_axis.set_local_translation(Translation3::new(0.0, 0.0, 0.5));

        // カメラ設定
        let camera = kiss3d::camera::ArcBall::new(Point3::new(2.0, 0.0, 4.0), Point3::origin());

        Self {
            window,
            camera,
            axes_group,
            font: Font::default(),
        }
    }

    /// ウィンドウを描画し継続判定を返す
    pub fn render(&mut self) -> bool {
        // Overlayの描画
        self.draw_overlay();

        self.window.render_with_camera(&mut self.camera)
    }

    /// 画面上のオーバーレイ(テキスト)を描画する
    fn draw_overlay(&mut self) {
        self.window.draw_text(
            "RESET (R)",
            &Point2::new(10.0, 10.0),
            40.0,
            &self.font,
            &Point3::new(1.0, 1.0, 1.0),
        );
    }

    /// 入力イベントを処理し、アクションを返します。
    pub fn handle_input(&mut self) -> UserAction {
        let mut action = UserAction::None;

        for event in self.window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    println!("Resetting orientation (Keyboard)!");
                    action = UserAction::Reset;
                }
                WindowEvent::MouseButton(MouseButton::Button1, Action::Press, _) => {
                    if let Some((x, y)) = self.window.cursor_pos() {
                        // 左上の簡易ヒットテスト
                        // テキストの範囲はおおよそ (10,10) から (200, 60) 程度
                        if x >= 10.0 && x <= 200.0 && y >= 10.0 && y <= 60.0 {
                            println!("Resetting orientation (Mouse Click)!");
                            action = UserAction::Reset;
                        }
                    }
                }
                _ => {}
            }
        }

        action
    }

    /// モデルの姿勢を更新する
    ///
    /// # 引数
    /// * `quat` - 表示用のクォータニオン (Kiss3d型)
    pub fn update_orientation(&mut self, quat: kiss3d::nalgebra::Quaternion<f32>) {
        let unit_quat = UnitQuaternion::from_quaternion(quat);
        self.axes_group.set_local_rotation(unit_quat);
    }
}
