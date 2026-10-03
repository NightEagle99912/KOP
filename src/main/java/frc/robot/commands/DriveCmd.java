package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

/**
 * ⚠️ 目前不要用：底盤由 DriveSubsystem 的狀態機控制（teleop 時 RobotContainer 設成 TELEOP 並傳入搖桿值），
 * DriveSubsystem.periodic() 每個週期都會依狀態輸出馬達（例如 STOPPED 時一直送 stop()），會跟這個 Command 搶馬達。
 * 如果之後要讓自動模式控制底盤，請先在 DriveSubsystem 加一個給 Command 用的狀態。
 */
public class DriveCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final XboxController controller;

    public DriveCmd(
        DriveSubsystem driveSubsystem,
        XboxController controller
    ) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        this.addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double driveSpeed = -MathUtil.applyDeadband(
            this.controller.getLeftY(),
            Drive.DEADBAND
        ) * Drive.MAX_SPEED;

        double turnSpeed = MathUtil.applyDeadband(
            this.controller.getRightX(),
            Drive.DEADBAND
        ) * Drive.MAX_TURN_SPEED;

        double leftSpeed = MathUtil.clamp(
            driveSpeed + turnSpeed, -1.0, 1.0
        );
        double rightSpeed = MathUtil.clamp(
            driveSpeed - turnSpeed, -1.0, 1.0
        );

        this.driveSubsystem.drive(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}