package frc.robot;

/**
 * 所有 CAN ID、DIO port 都集中在這裡，改接線時只要改這個檔案。
 *
 * <p>注意：同一種裝置（例如兩顆 TalonFX）的 CAN ID 不能重複；
 * 不同廠牌的裝置（TalonFX 和 SparkMax）ID 相同不會衝突，但為了好認，這裡全部都不重複。
 *
 * <p>目前使用：
 * <pre>
 *  ID | 裝置              | 控制器
 *  ---+-------------------+---------
 *   1 | 底盤 左前         | SparkMax
 *   2 | 底盤 左後         | SparkMax
 *   3 | 底盤 右前         | SparkMax
 *   4 | 底盤 右後         | SparkMax
 *   5 | Conveyor          | TalonFX
 *   6 | Intake 滾輪 右    | TalonFX
 *   7 | Intake 滾輪 左    | TalonFX
 *   8 | （空）            |
 *   9 | Shooter Hood      | TalonFX
 *  10 | Shooter Feeder    | SparkMax
 *  11 | Hopper Center     | TalonFX
 *  12 | Hopper Roller     | TalonFX
 *  13 | Lifter 右         | TalonFX
 *  14 | Lifter 左         | TalonFX
 *  15 | Flywheel 左       | TalonFX
 *  16 | Flywheel 右       | TalonFX
 * </pre>
 */
public final class DeviceId {
    /** 底盤（SparkMax） */
    public static final class DriveMotor {
        public static final int FRONT_LEFT = 1;
        public static final int BACK_LEFT = 2;
        public static final int FRONT_RIGHT = 3;
        public static final int BACK_RIGHT = 4;
    }

    /** Conveyor（TalonFX） */
    public static final class Conveyor {
        //TODO
        public static final int MOTOR = 5;
    }

    /** Intake 吸球滾輪（TalonFX） */
    public static final class IntakeRoller {
        public static final int RIGHT = 6;
        public static final int LEFT = 7;
    }

    /** Intake 升降 Lifter（TalonFX + DIO 絕對編碼器） */
    public static final class Lifter {
        // TODO: 原本是 7、8，和 IntakeRoller / Flywheel 重複，依實際接線修改
        public static final int RIGHT = 13;
        public static final int LEFT = 14;
        public static final int ENCODER_DIO = 0;
    }

    /** Hopper（TalonFX） */
    public static final class Hopper {
        //TODO
        public static final int CENTER = 11;
        public static final int ROLLER = 12;
    }

    /** Shooter */
    public static final class Shooter {
        // TODO: 原本是 7、8，和 IntakeRoller / Lifter 重複，依實際接線修改
        public static final int FLYWHEEL_LEFT = 15;  // TalonFX
        public static final int FLYWHEEL_RIGHT = 16; // TalonFX
        public static final int HOOD = 9;            // TalonFX
        public static final int FEEDER = 10;         // SparkMax
    }

    private DeviceId() {}
}
