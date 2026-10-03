package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

/**
 * Shooter 的指令：不直接控制 Shooter，而是透過 Superstructure 設定想要的狀態，
 * 保留狀態機原本的安全邏輯（例如飛輪到速才送球）。
 *
 * <p>指令執行期間每個週期都會設定一次狀態，結束時切回 IDLE。
 * 預設不會自己結束，需要搭配 whileTrue()、withTimeout() 等使用。
 */
public class ShooterCommand extends Command {
    public enum Mode {
        PREPARE_SHOT,  // 預轉飛輪、調仰角，不送球
        SHOOT,         // 射球（飛輪到速、仰角到位才自動送球）
        SOURCE_COLLECT // 從 Source 經過 Shooter 吸球
    }

    private final Superstructure superstructure;
    private final Mode mode;
    private final DoubleSupplier distanceMeters;

    public ShooterCommand(
        Superstructure superstructure,
        Mode mode,
        DoubleSupplier distanceMeters
    ) {
        this.superstructure = superstructure;
        this.mode = mode;
        this.distanceMeters = distanceMeters;
        this.addRequirements(this.superstructure);
    }

    /** 固定距離（公尺） */
    public ShooterCommand(Superstructure superstructure, Mode mode, double distanceMeters) {
        this(superstructure, mode, () -> distanceMeters);
    }

    @Override
    public void execute() {
        this.superstructure.setShootDistance(this.distanceMeters.getAsDouble());
        this.superstructure.setWantedState(switch (this.mode) {
            case PREPARE_SHOT -> WantedState.PREPARE_SHOT;
            case SHOOT -> WantedState.SHOOT;
            case SOURCE_COLLECT -> WantedState.SOURCE_COLLECT;
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

    public static ShooterCommand prepareShot(Superstructure superstructure, double distanceMeters) {
        return new ShooterCommand(superstructure, Mode.PREPARE_SHOT, distanceMeters);
    }

    public static ShooterCommand shoot(Superstructure superstructure, double distanceMeters) {
        return new ShooterCommand(superstructure, Mode.SHOOT, distanceMeters);
    }

    public static ShooterCommand sourceCollect(Superstructure superstructure) {
        return new ShooterCommand(superstructure, Mode.SOURCE_COLLECT, 0.0);
    }
}
