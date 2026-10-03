package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopperSubsystem.Hopper;

/**
 * ⚠️ 目前不要用：Hopper 由 Superstructure 統一控制，Superstructure 每個週期都會重設 Hopper 的狀態，
 * 這個 Command 設定的狀態會馬上被蓋掉，還會跟射球、吸球的連動邏輯衝突。
 * 需要單獨轉 Hopper 的話，請在 Superstructure 加一個 WantedState，再用 Command 設定它。
 *
 * <p>Hopper 的 Command：執行時把 Hopper 切到指定狀態，結束（放開按鈕）時切回 STOPPED。
 * 用法：button.whileTrue(new HopperCommand(hopper, Hopper.State.FEED));
 */
public class HopperCommand extends Command {
    private final Hopper hopper;
    private final Hopper.State state;

    public HopperCommand(Hopper hopper, Hopper.State state) {
        this.hopper = hopper;
        this.state = state;
        this.addRequirements(this.hopper);
    }

    @Override
    public void initialize() {
        this.hopper.setState(this.state);
    }

    @Override
    public void end(boolean interrupted) {
        this.hopper.setState(Hopper.State.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
