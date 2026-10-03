package frc.robot.subsystems.shooterSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.DeviceId;

/** Flywheel（飛輪）：左右兩顆 TalonFX，用前饋 + PID 維持轉速 */
public class Flywheel {
    // 兩顆馬達面對面裝的話，通常其中一顆要反向
    private static final InvertedValue LEFT_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final InvertedValue RIGHT_INVERTED = InvertedValue.Clockwise_Positive;
    private static final double CURRENT_LIMIT = 40;

    // 速度單位：轉/秒（RPS）；轉速、前饋、PID 數值都在 Constants.Shooter
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.Shooter.FLYWHEEL_KS, Constants.Shooter.FLYWHEEL_KV);
    // 回饋：補償剩下的誤差（輸出單位：伏特）
    private final PIDController pidController = new PIDController(Constants.Shooter.FLYWHEEL_KP, 0.0, 0.0);

    private final TalonFX leftMotor = new TalonFX(DeviceId.Shooter.FLYWHEEL_LEFT);
    private final TalonFX rightMotor = new TalonFX(DeviceId.Shooter.FLYWHEEL_RIGHT);

    private double targetRps = 0.0;

    public Flywheel() {
        this.configureMotor(this.leftMotor, LEFT_INVERTED);
        this.configureMotor(this.rightMotor, RIGHT_INVERTED);
    }

    private void configureMotor(TalonFX motor, InvertedValue inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // 飛輪放開後自然減速，不要急停
        config.MotorOutput.Inverted = inverted;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    /** 讓飛輪維持在指定轉速，需要每個週期持續呼叫 */
    public void setVelocity(double rps) {
        this.targetRps = rps;
        final double volts = this.feedforward.calculate(rps)
            + this.pidController.calculate(this.getVelocity(), rps);

        this.leftMotor.setVoltage(volts);
        this.rightMotor.setVoltage(volts);
    }

    /** 反轉，從 Source 吸球 */
    public void collectSource() {
        this.targetRps = 0.0;
        this.leftMotor.set(-Constants.Shooter.FLYWHEEL_SOURCE_COLLECT_SPEED);
        this.rightMotor.set(-Constants.Shooter.FLYWHEEL_SOURCE_COLLECT_SPEED);
    }

    public void stop() {
        this.targetRps = 0.0;
        this.leftMotor.stopMotor();
        this.rightMotor.stopMotor();
        this.pidController.reset();
    }

    /** 兩顆馬達的平均轉速（RPS） */
    public double getVelocity() {
        return (this.leftMotor.getVelocity().getValueAsDouble()
            + this.rightMotor.getVelocity().getValueAsDouble()) / 2.0;
    }

    public double getTarget() {
        return this.targetRps;
    }

    public boolean isAtSpeed() {
        return this.targetRps > 0
            && Math.abs(this.getVelocity() - this.targetRps) < Constants.Shooter.FLYWHEEL_TOLERANCE_RPS;
    }
}
