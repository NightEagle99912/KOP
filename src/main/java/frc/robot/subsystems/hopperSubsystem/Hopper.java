package frc.robot.subsystems.hopperSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hopper 輸送機構：組合 HopperCenter（中央進料滾輪）、HopperRoller（外側輸送滾輪）兩個機構。
 * 用 setState() 切換狀態，periodic() 每個週期依目前狀態控制馬達。
 */
public class Hopper extends SubsystemBase {
    public enum State {
        STOPPED, // 全部停止
        FEED,    // 兩組滾輪一起送料
        REVERSE  // 兩組滾輪一起反轉，清除卡住的物件
    }

    private final HopperCenter center = new HopperCenter();
    private final HopperRoller roller = new HopperRoller();

    private State state = State.STOPPED;

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    private void feed() {
        this.center.feed();
        this.roller.feed();
    }

    private void reverse() {
        this.center.reverse();
        this.roller.reverse();
    }

    private void stopAll() {
        this.center.stop();
        this.roller.stop();
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case STOPPED -> this.stopAll();
            case FEED -> this.feed();
            case REVERSE -> this.reverse();
        }

        SmartDashboard.putString("Hopper/State", this.state.name());
        SmartDashboard.putNumber("Hopper/Center/Output", this.center.getOutput());
        SmartDashboard.putNumber("Hopper/Center/Current", this.center.getCurrent());
        SmartDashboard.putNumber("Hopper/Roller/Output", this.roller.getOutput());
        SmartDashboard.putNumber("Hopper/Roller/Current", this.roller.getCurrent());
    }
}
