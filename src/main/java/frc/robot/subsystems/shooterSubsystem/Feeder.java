package frc.robot.subsystems.shooterSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.DeviceId;

/** Feeder（推球滾輪）：SparkMax 百分比輸出 */
public class Feeder {
    private static final boolean INVERTED = false; // 方向反了就改 true
    private static final int CURRENT_LIMIT = 30;

    private final SparkMax motor = new SparkMax(DeviceId.Shooter.FEEDER, MotorType.kBrushless);

    public Feeder() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake) // 停下時球不會被慢慢推進去
            .inverted(INVERTED)
            .smartCurrentLimit(CURRENT_LIMIT);

        this.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void feed() {
        this.motor.set(Constants.Shooter.FEEDER_FEED_SPEED);
    }

    public void reverse() {
        this.motor.set(Constants.Shooter.FEEDER_REVERSE_SPEED);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public double getOutput() {
        return this.motor.getAppliedOutput();
    }
}
