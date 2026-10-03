package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.DeviceId.DriveMotor;

/**
 * 底盤狀態機：用 setState() 切換狀態，periodic() 每個週期依目前狀態控制馬達。
 */
public class DriveSubsystem extends SubsystemBase {
    public enum State {
        STOPPED, // 停止
        TELEOP   // 搖桿 arcade drive
    }

    private final DriveMotorModule leftModule;
    private final DriveMotorModule rightModule;

    private State state = State.STOPPED;
    private double forwardInput = 0.0; // -1 ~ 1，往前為正
    private double turnInput = 0.0;    // -1 ~ 1，往右為正

    public DriveSubsystem() {
        this.leftModule = new DriveMotorModule(
            DriveMotor.FRONT_LEFT,
            DriveMotor.BACK_LEFT,
            Drive.FRONT_LEFT_INVERTED,
            Drive.BACK_LEFT_INVERTED
        );
        this.rightModule = new DriveMotorModule(
            DriveMotor.FRONT_RIGHT,
            DriveMotor.BACK_RIGHT,
            Drive.FRONT_RIGHT_INVERTED,
            Drive.BACK_RIGHT_INVERTED
        );
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    /** TELEOP 狀態使用的搖桿輸入，需要每個週期更新 */
    public void setTeleopInput(double forward, double turn) {
        this.forwardInput = forward;
        this.turnInput = turn;
    }

    private void arcadeDrive() {
        double driveSpeed = MathUtil.applyDeadband(
            this.forwardInput,
            Drive.DEADBAND
        ) * Drive.MAX_SPEED;

        double turnSpeed = MathUtil.applyDeadband(
            this.turnInput,
            Drive.DEADBAND
        ) * Drive.MAX_TURN_SPEED;

        double leftSpeed = MathUtil.clamp(
            driveSpeed + turnSpeed, -1.0, 1.0
        );
        double rightSpeed = MathUtil.clamp(
            driveSpeed - turnSpeed, -1.0, 1.0
        );

        this.drive(leftSpeed, rightSpeed);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        this.leftModule.set(leftSpeed);
        this.rightModule.set(rightSpeed);
    }

    public void stop() {
        this.leftModule.stop();
        this.rightModule.stop();
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case STOPPED -> this.stop();
            case TELEOP -> this.arcadeDrive();
        }

        SmartDashboard.putString("Drive/State", this.state.name());
    }
}
