package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

/**
 * 組合所有狀態機，並把搖桿輸入轉成想要的狀態。
 *
 * <p>按鍵配置（同時按多個鍵時，越上面的越優先）：
 * <ul>
 *   <li>右 Trigger：射球（飛輪到速才自動送球）</li>
 *   <li>右 Bumper：預轉飛輪</li>
 *   <li>左 Trigger：地面吸球</li>
 *   <li>左 Bumper：反轉吐出</li>
 *   <li>Y：射 AMP</li>
 *   <li>X：從 Source 吸球</li>
 *   <li>什麼都沒按：收起、停止</li>
 * </ul>
 *
 * <p>自動模式用 Commands 組合，在 SmartDashboard 的「Auto/Mode」選要跑哪一個。
 */
public class RobotContainer {
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final GamepadJoystick joystick =
        new GamepadJoystick(GamepadJoystick.CONTROLLER_PORT);

    private final DriveSubsystem driveSubsystem =
        new DriveSubsystem();
    private final Superstructure superstructure =
        new Superstructure();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        this.configureAutos();
    }

    /** 自動模式選單：新的自動流程寫好後，在這裡用 addOption() 加進去 */
    private void configureAutos() {
        this.autoChooser.setDefaultOption("不動", Commands.none());
        this.autoChooser.addOption("射預載球", this.shootPreloadAuto());
        SmartDashboard.putData("Auto/Mode", this.autoChooser);
    }

    /**
     * 範例：原地射出預載的球。
     * 可以用 Commands.sequence()（依序）、Commands.parallel()（同時）、withTimeout()、until() 組合更多步驟。
     */
    private Command shootPreloadAuto() {
        return ShooterCommand.shoot(this.superstructure, Constants.Shooter.DEFAULT_DISTANCE)
            .withTimeout(3.0);
    }

    /** 目前在 SmartDashboard 選到的自動模式 */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }

    /** 全部停止（Disabled / Autonomous / Test 進入時） */
    public void setIdle() {
        this.driveSubsystem.setState(DriveSubsystem.State.STOPPED);
        this.superstructure.setWantedState(WantedState.IDLE);
    }

    public void teleopInit() {
        this.driveSubsystem.setState(DriveSubsystem.State.TELEOP);
        this.superstructure.setWantedState(WantedState.IDLE);
    }

    public void teleopPeriodic() {
        this.driveSubsystem.setTeleopInput(
            -this.joystick.getLeftY(),
            this.joystick.getRightX()
        );

        // 有 Command 在控制 Superstructure 時讓給 Command，不然搖桿會每個週期蓋掉 Command 設定的狀態
        if (this.superstructure.getCurrentCommand() == null) {
            this.superstructure.setShootDistance(Constants.Shooter.DEFAULT_DISTANCE);
            this.superstructure.setWantedState(this.readWantedState());
        }
    }

    private WantedState readWantedState() {
        if (this.joystick.getRightTriggerAxis() > TRIGGER_THRESHOLD) return WantedState.SHOOT;
        if (this.joystick.getRightBumperButton()) return WantedState.PREPARE_SHOT;
        if (this.joystick.getLeftTriggerAxis() > TRIGGER_THRESHOLD) return WantedState.INTAKE;
        if (this.joystick.getLeftBumperButton()) return WantedState.EJECT;
        if (this.joystick.getYButton()) return WantedState.AMP;
        if (this.joystick.getXButton()) return WantedState.SOURCE_COLLECT;
        return WantedState.IDLE;
    }
}
