package frc.robot.subsystems.shooterSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.DeviceId;

/** Hood（仰角）：TalonFX 位置閉環，單位為度 */
public class Hood {
    private static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive; // 仰角方向反了就改 Clockwise_Positive
    private static final double CURRENT_LIMIT = 20;

    private static final double GEAR_RATIO = 50.0;  // 馬達轉幾圈 = 仰角機構轉 1 圈

    // 角度範圍、誤差、PID 數值都在 Constants.Shooter
    private static final double MIN_DEGREES = Constants.Shooter.HOOD_MIN_DEGREES;
    private static final double MAX_DEGREES = Constants.Shooter.HOOD_MAX_DEGREES;

    private final TalonFX motor = new TalonFX(DeviceId.Shooter.HOOD);
    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    private double targetDegrees = MIN_DEGREES;

    public Hood() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 停下時鎖住角度
        config.MotorOutput.Inverted = INVERTED;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO; // 之後讀到的位置就是機構的圈數
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_DEGREES / 360.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_DEGREES / 360.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.Slot0.kP = Constants.Shooter.HOOD_KP;
        config.Slot0.kD = Constants.Shooter.HOOD_KD;
        this.motor.getConfigurator().apply(config);

        // 開機時假設仰角在最低點
        this.motor.setPosition(MIN_DEGREES / 360.0);
    }

    public void setAngle(double degrees) {
        this.targetDegrees = MathUtil.clamp(degrees, MIN_DEGREES, MAX_DEGREES);
        this.motor.setControl(this.request.withPosition(this.targetDegrees / 360.0));
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public double getAngle() {
        return this.motor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getTarget() {
        return this.targetDegrees;
    }

    public boolean isAtAngle() {
        return Math.abs(this.getAngle() - this.targetDegrees) < Constants.Shooter.HOOD_TOLERANCE_DEGREES;
    }
}
