package frc.robot.subsystems.intakeSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

/**
 * Lifter（升降滑軌）狀態機：用 setState() 切換狀態，periodic() 每個週期依目前狀態跑 PID。
 * 位置、出力、PID 等數值都在 {@link Constants.Lifter}。
 *
 * <p>安全保護：絕對編碼器斷線（DIO 線鬆脫）時讀到的角度不可信，
 * 這時不管狀態是什麼都會停住馬達，isAt 系列也一律回傳 false，接回來後自動恢復。
 */
public class Lifter extends SubsystemBase {
    public enum State {
        STOPPED, // 馬達停止（Brake）
        STOW,    // 收到最高點 STOW_SETPOINT
        DEPLOY,  // 放到最低點 DEPLOY_SETPOINT
        AMP      // 移到射 AMP 的角度
    }

    // ===== 機構固定參數（硬體沒改就不用動）=====
    private static final boolean ENCODER_INVERTED = false; // 編碼器方向反了就改 true，再重新校正 Constants.Lifter.ZERO_OFFSET
    private static final double LIFTER_GEAR_RATIO = 18.0 / 22.0;

    private final TalonFX rightLiftMotor = new TalonFX(DeviceId.Lifter.RIGHT);
    private final TalonFX leftLiftMotor = new TalonFX(DeviceId.Lifter.LEFT);

    private final DutyCycleEncoder liftEncoder = new DutyCycleEncoder(DeviceId.Lifter.ENCODER_DIO);
    private final PIDController liftPIDController = new PIDController(
        Constants.Lifter.KP, Constants.Lifter.KI, Constants.Lifter.KD);
    private final PIDController stowAssistPIDController = new PIDController(
        Constants.Lifter.STOW_ASSIST_KP, 0.0, 0.0);

    private State state = State.STOPPED;
    private boolean wasEncoderConnected = true;

    public Lifter() {
        configureMotor(this.rightLiftMotor, true);
        configureMotor(this.leftLiftMotor, false);
    }

    private static void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    // 單位：度
    public double getPosition() {
        double raw = Units.rotationsToDegrees(this.liftEncoder.get());
        if (ENCODER_INVERTED) raw = -raw;
        // 先反轉再加 offset、取 0 ~ 360，就算反轉結果也一定是正的，設定點不用跟著改
        // （Java 的 % 遇到負數會得到負的，所以要再 +360 % 360 一次）
        double position = ((raw + Constants.Lifter.ZERO_OFFSET) % 360 + 360) % 360;
        return position * LIFTER_GEAR_RATIO + Constants.Lifter.FORCE_OFFSET;
    }

    private void lift(double percentage) {
        this.rightLiftMotor.set(percentage);
        this.leftLiftMotor.set(percentage);
    }

    // 以下 liftTo 系列方法由 periodic() 依狀態每個週期呼叫，PID 才會持續作用
    private void liftTo(double setpoint) {
        double output = this.liftPIDController.calculate(this.getPosition(), setpoint);
        this.lift(MathUtil.clamp(output, -Constants.Lifter.MAX_OUTPUT, Constants.Lifter.MAX_OUTPUT));
    }

    private void liftToMax() {
        double output;
        if (this.getPosition() > Constants.Lifter.STOW_BRAKE_ANGLE) {
            output = this.liftPIDController.calculate(this.getPosition(), Constants.Lifter.STOW_SETPOINT);
            output = MathUtil.clamp(output,
                -Constants.Lifter.STOW_BRAKE_MAX_OUTPUT, Constants.Lifter.STOW_BRAKE_MAX_OUTPUT);
        } else {
            output = this.stowAssistPIDController.calculate(this.getPosition(), Constants.Lifter.STOW_ASSIST_TARGET)
                + Constants.Lifter.STOW_MIN_OUTPUT;
            output = MathUtil.clamp(output, Constants.Lifter.STOW_MIN_OUTPUT, Constants.Lifter.STOW_MAX_OUTPUT);
        }

        this.lift(output);
    }

    /** 絕對編碼器是否有接上（有收到訊號） */
    public boolean isEncoderConnected() {
        return this.liftEncoder.isConnected();
    }

    /** 編碼器斷線時角度不可信，一律回傳 false，避免 Superstructure 誤以為已經到位 */
    public boolean isAt(double setpoint, double threshold) {
        return this.isEncoderConnected()
            && Math.abs(setpoint - this.getPosition()) <= threshold;
    }

    public boolean isAtMin() {
        return this.isAt(Constants.Lifter.DEPLOY_SETPOINT, Constants.Lifter.TOLERANCE);
    }

    public boolean isAtMax() {
        return this.isAt(Constants.Lifter.STOW_SETPOINT, Constants.Lifter.TOLERANCE);
    }

    public boolean isAtAMP() {
        return this.isAt(Constants.Lifter.AMP_SETPOINT, Constants.Lifter.TOLERANCE);
    }

    private void stop() {
        this.rightLiftMotor.stopMotor();
        this.leftLiftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        boolean encoderConnected = this.isEncoderConnected();

        // 斷線 / 恢復的瞬間各在 Driver Station 顯示一次
        if (encoderConnected != this.wasEncoderConnected) {
            if (encoderConnected) {
                DriverStation.reportWarning("Lifter 編碼器已恢復連線", false);
            } else {
                DriverStation.reportError("Lifter 編碼器斷線！Lifter 已停止，請檢查 DIO "
                    + DeviceId.Lifter.ENCODER_DIO + " 的接線", false);
            }
            this.wasEncoderConnected = encoderConnected;
        }

        if (!encoderConnected) {
            this.stop();
        } else {
            switch (this.state) {
                case STOPPED -> this.stop();
                case STOW -> this.liftToMax();
                case DEPLOY -> this.liftTo(Constants.Lifter.DEPLOY_SETPOINT);
                case AMP -> this.liftTo(Constants.Lifter.AMP_SETPOINT);
            }
        }

        SmartDashboard.putNumber("Intake/Lifter/Position", this.getPosition());
        SmartDashboard.putString("Intake/Lifter/State", this.state.name());
        SmartDashboard.putBoolean("Intake/Lifter/EncoderConnected", encoderConnected);
    }
}
