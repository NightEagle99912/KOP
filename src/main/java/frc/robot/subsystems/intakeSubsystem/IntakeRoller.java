package frc.robot.subsystems.intakeSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceId;

/**
 * Intake Roller（吸球滾輪）狀態機：用 setState() 切換狀態，periodic() 每個週期依目前狀態控制馬達。
 */
public class IntakeRoller extends SubsystemBase {
    public enum State {
        STOPPED,  // 停止
        INTAKE,   // 吸球
        RELEASE,  // 全速吐球
        AMP,      // 射 AMP
        FINE_TUNE // 微調 Note 位置
    }

    private final TalonFX rightIntakeMotor = new TalonFX(DeviceId.IntakeRoller.RIGHT);
    private final TalonFX leftIntakeMotor = new TalonFX(DeviceId.IntakeRoller.LEFT);

    private State state = State.STOPPED;

    public IntakeRoller() {
        // TODO: 左右兩顆目前都設成 inverted = true。如果兩顆馬達是面對面（鏡像）裝的，
        //       其中一顆通常要反過來（Lifter 就是右 true、左 false），不然兩邊會互相對抗。
        //       上機時用低速（INTAKE_SPEED）確認兩邊滾輪都是往內吸，再決定要不要改。
        configureMotor(this.rightIntakeMotor, true);
        configureMotor(this.leftIntakeMotor, true);
    }

    private static void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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

    private void set(double percentage) {
        this.rightIntakeMotor.set(percentage);
        this.leftIntakeMotor.set(percentage);
    }

    private void stop() {
        this.rightIntakeMotor.stopMotor();
        this.leftIntakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case STOPPED -> this.stop();
            case INTAKE -> this.set(Constants.IntakeRoller.INTAKE_SPEED);
            case RELEASE -> this.set(-Constants.IntakeRoller.RELEASE_SPEED);
            case AMP -> this.set(-Constants.IntakeRoller.AMP_SPEED);
            case FINE_TUNE -> this.set(Constants.IntakeRoller.FINE_TUNE_SPEED);
        }

        SmartDashboard.putString("Intake/Roller/State", this.state.name());
    }
}
