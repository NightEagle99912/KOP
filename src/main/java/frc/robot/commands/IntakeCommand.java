package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

/**
 * Intake 的指令：不直接控制 Lifter / IntakeRoller，而是透過 Superstructure 設定想要的狀態，
 * 保留狀態機原本的安全邏輯（例如 Lifter 放到底才轉滾輪）。
 *
 * <p>指令執行期間每個週期都會設定一次狀態，結束時切回 IDLE（Lifter 收起、滾輪停止）。
 * 預設不會自己結束，需要搭配 whileTrue()、withTimeout() 等使用。
 */
public class IntakeCommand extends Command {
    public enum Mode {
        INTAKE, // 放下 Lifter，到底後滾輪吸球
        EJECT, // 放下 Lifter，反轉吐出卡住的物件
        AMP     // Lifter 移到 AMP 角度，到位後吐球進 AMP
    }

    private final Superstructure superstructure;
    private final Mode mode;

    public IntakeCommand(Superstructure superstructure, Mode mode) {
        this.superstructure = superstructure;
        this.mode = mode;
        this.addRequirements(this.superstructure);
    }

    @Override
    public void execute() {
        this.superstructure.setWantedState(switch (this.mode) {
            case INTAKE -> WantedState.INTAKE;
            case EJECT -> WantedState.EJECT;
            case AMP -> WantedState.AMP;
        });
    }

    @Override
    public void end(boolean interrupted) {
        this.superstructure.setWantedState(WantedState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ===== 方便使用的工廠方法 =====

    public static IntakeCommand intake(Superstructure superstructure) {
        return new IntakeCommand(superstructure, Mode.INTAKE);
    }

    public static IntakeCommand eject(Superstructure superstructure) {
        return new IntakeCommand(superstructure, Mode.EJECT);
    }

    public static IntakeCommand amp(Superstructure superstructure) {
        return new IntakeCommand(superstructure, Mode.AMP);
    }
}
