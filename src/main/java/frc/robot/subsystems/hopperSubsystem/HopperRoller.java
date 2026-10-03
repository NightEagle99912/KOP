package frc.robot.subsystems.hopperSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.DeviceId;

/**
 * Hopper Roller（外側輸送滾輪）：TalonFX 百分比輸出。
 * 只是 Hopper 的零件，由 Hopper 控制，所以不繼承 SubsystemBase（跟 Shooter 的 Feeder、Flywheel 一樣）。
 */
public class HopperRoller {
    private static final boolean INVERTED = false; // 方向反了就改 true
    private static final int CURRENT_LIMIT = 30;

    private final TalonFX motor = new TalonFX(DeviceId.Hopper.ROLLER);

    public HopperRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 放開後立刻停住，物件不會亂滑
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        this.motor.getConfigurator().apply(config);
    }

    public void feed() {
        this.motor.set(Constants.Hopper.ROLLER_FEED_SPEED);
    }

    public void reverse() {
        this.motor.set(Constants.Hopper.ROLLER_REVERSE_SPEED);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public double getOutput() {
        return this.motor.getDutyCycle().getValueAsDouble();
    }

    public double getCurrent() {
        return this.motor.getSupplyCurrent().getValueAsDouble();
    }
}
