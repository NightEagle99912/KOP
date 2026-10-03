// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * 所有需要測試調整的數值（速度、位置、PID、誤差範圍）都集中在這裡，
 * 測試時只要改這個檔案就好。CAN ID / DIO 接線請改 {@link DeviceId}。
 *
 * <p>速度沒特別寫單位的都是百分比輸出（-1.0 ~ 1.0）。
 */
public final class Constants {
  public static final class Drive {
    public static final double MAX_SPEED = 0.3;
    public static final double MAX_TURN_SPEED = 0.5;// TODO
    public static final double DEADBAND = 0.05;

    public static final boolean FRONT_LEFT_INVERTED = false;
    public static final boolean BACK_LEFT_INVERTED = false;
    public static final boolean FRONT_RIGHT_INVERTED = true;
    public static final boolean BACK_RIGHT_INVERTED = true;
  }

  /** Conveyor */
  public static final class Conveyor {
    public static final double FORWARD_SPEED = 0.5;  // 正轉（送料）
    public static final double REVERSE_SPEED = -0.5; // 反轉（吐出卡住的物件）
  }

  /** Intake 吸球滾輪 */
  public static final class IntakeRoller {
    public static final double INTAKE_SPEED = 0.2;
    public static final double RELEASE_SPEED = 1.0;     // 全速吐球（程式裡會加負號）
    public static final double AMP_SPEED = 0.75;        // 射 AMP（程式裡會加負號）
    public static final double FINE_TUNE_SPEED = 0.04;  // 微調 Note 位置
  }

  /** Intake 升降 Lifter，角度單位：度 */
  public static final class Lifter {
    // ===== 位置 =====
    public static final double STOW_SETPOINT = 224.5;  // 收起（最高點）
    public static final double DEPLOY_SETPOINT = 62.0; // 放下吸球（最低點）
    public static final double AMP_SETPOINT = 154.0;   // 射 AMP
    public static final double TOLERANCE = 5.0;        // 判斷是否到位的誤差範圍

    // ===== 出力上限（安全用，確認動作正常後再慢慢調高）=====
    public static final double MAX_OUTPUT = 0.4; // DEPLOY / AMP 移動時的最大出力

    // ===== 收起（STOW）=====
    // 低於 STOW_BRAKE_ANGLE：用 STOW_ASSIST PID 往 STOW_ASSIST_TARGET 推，出力介於 STOW_MIN_OUTPUT ~ STOW_MAX_OUTPUT
    // 高於 STOW_BRAKE_ANGLE：改用一般 PID 往 STOW_SETPOINT 收，出力限制在 ±STOW_BRAKE_MAX_OUTPUT
    public static final double STOW_BRAKE_ANGLE = 180.0;
    public static final double STOW_ASSIST_TARGET = 175.0;
    public static final double STOW_MIN_OUTPUT = 0.2;
    public static final double STOW_MAX_OUTPUT = 0.5;
    public static final double STOW_BRAKE_MAX_OUTPUT = 0.2;

    // ===== PID =====
    public static final double KP = 0.006;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double STOW_ASSIST_KP = 0.01;

    // ===== 編碼器校正 =====
    public static final double ZERO_OFFSET = 172.0; // 度
    public static final double FORCE_OFFSET = 0.0;  // 度，強制補償
  }

  /** Hopper */
  public static final class Hopper {
    public static final double CENTER_FEED_SPEED = 0.5;     // 中央滾輪：把物件往中間送
    public static final double CENTER_REVERSE_SPEED = -0.4; // 中央滾輪：反轉清除卡住的物件
    public static final double ROLLER_FEED_SPEED = 0.5;     // 外側滾輪：把物件從外側往內送
    public static final double ROLLER_REVERSE_SPEED = -0.4; // 外側滾輪：反轉清除卡住的物件
  }

  /** Shooter（Flywheel、Hood、Feeder、射擊表格） */
  public static final class Shooter {
    // TODO: 之後接視覺（Limelight 等）改成即時距離
    public static final double DEFAULT_DISTANCE = 2.5; // 公尺

    // ===== Feeder =====
    public static final double FEEDER_FEED_SPEED = 0.8;     // 把球推進飛輪
    public static final double FEEDER_REVERSE_SPEED = -0.4; // 反轉清除卡住的物件

    // ===== Flywheel（轉速單位：RPS，Kraken / Falcon 空載最高約 100 RPS）=====
    public static final double FLYWHEEL_TOLERANCE_RPS = 5.0;     // 誤差在這個範圍內就算到速
    public static final double FLYWHEEL_SOURCE_COLLECT_SPEED = 0.2; // 從 Source 反轉吸球（程式裡會加負號）
    // TODO: 需要實際調整
    public static final double FLYWHEEL_KS = 0.1;  // 前饋：克服靜摩擦的電壓
    public static final double FLYWHEEL_KV = 0.12; // 前饋：每 1 RPS 需要的電壓
    public static final double FLYWHEEL_KP = 0.1;  // 回饋：每 1 RPS 誤差補多少電壓

    // ===== Hood（角度單位：度）=====
    public static final double HOOD_MIN_DEGREES = 20.0; // 最低仰角（開機時仰角必須靠在這個位置）
    public static final double HOOD_MAX_DEGREES = 60.0; // 最高仰角
    public static final double HOOD_TOLERANCE_DEGREES = 1.0;
    // 位置閉環參數，需要實際調整（單位：每 1 圈機構誤差補多少電壓）
    public static final double HOOD_KP = 100.0;
    public static final double HOOD_KD = 1.0;

    // ===== 射擊表格（中間值自動線性內插，超出範圍用最近的端點）=====
    // TODO: 實際試射後填入
    // 格式：{ 距離（公尺）, 飛輪轉速（RPS）, 仰角（度） }
    public static final double[][] SHOOT_TABLE = {
      { 1.0, 45.0, 25.0 },
      { 2.0, 52.0, 32.0 },
      { 3.0, 60.0, 40.0 },
      { 4.0, 68.0, 47.0 },
      { 5.0, 75.0, 53.0 },
    };
  }

  private Constants() {
  }
}
