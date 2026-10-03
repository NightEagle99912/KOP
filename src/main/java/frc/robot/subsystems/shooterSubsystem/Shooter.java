package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shooter 射擊機構：組合 Flywheel（飛輪）、Hood（仰角）、Feeder（推球滾輪）三個機構，
 * 並用 ShootCalculator 依距離即時查表算出射擊參數。
 * 用 setState() 切換狀態，periodic() 每個週期依目前狀態控制馬達。
 */
public class Shooter {
    public enum State {
        STOPPED,       // 全部停止
        AIMING,        // 依距離預轉飛輪、調仰角，不推球
        SHOOTING,      // 持續瞄準，準備好就自動推球
        SOURCE_COLLECT // 飛輪和推球滾輪一起反轉，從 Source 吸球
    }

    private final Flywheel flywheel = new Flywheel();
    private final Hood hood = new Hood();
    private final Feeder feeder = new Feeder();
    private final ShootCalculator calculator = new ShootCalculator();

    private State state = State.STOPPED;
    private double distanceMeters = 0.0;

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    /** 目標距離（公尺），AIMING / SHOOTING 狀態會用它查表 */
    public void setDistance(double distanceMeters) {
        this.distanceMeters = distanceMeters;
    }

    /** 依距離設定飛輪轉速與仰角 */
    private void aim() {
        ShootCalculator.ShootParams params = this.calculator.calculate(this.distanceMeters);
        this.flywheel.setVelocity(params.flywheelRps());
        this.hood.setAngle(params.hoodDegrees());
    }

    /** 飛輪到速且仰角到位，才可以推球 */
    public boolean canShoot() {
        return this.flywheel.isAtSpeed() && this.hood.isAtAngle();
    }

    /** 持續依距離瞄準，準備好就自動推球 */
    private void executeShooter() {
        this.aim();
        if (this.canShoot()) {
            this.feeder.feed();
        } else {
            this.feeder.stop();
        }
    }

    private void collectSource() {
        this.flywheel.collectSource();
        this.feeder.reverse();
    }

    private void stopAll() {
        this.flywheel.stop();
        this.hood.stop();
        this.feeder.stop();
    }

    public void periodic() {
        switch (this.state) {
            case STOPPED -> this.stopAll();
            case AIMING -> {
                this.aim();
                this.feeder.stop();
            }
            case SHOOTING -> this.executeShooter();
            case SOURCE_COLLECT -> this.collectSource();
        }

        SmartDashboard.putString("Shooter/State", this.state.name());
        SmartDashboard.putNumber("Shooter/Distance", this.distanceMeters);
        SmartDashboard.putNumber("Shooter/Flywheel/TargetRPS", this.flywheel.getTarget());
        SmartDashboard.putNumber("Shooter/Flywheel/VelocityRPS", this.flywheel.getVelocity());
        SmartDashboard.putNumber("Shooter/Hood/TargetDeg", this.hood.getTarget());
        SmartDashboard.putNumber("Shooter/Hood/AngleDeg", this.hood.getAngle());
        SmartDashboard.putNumber("Shooter/Feeder/Output", this.feeder.getOutput());
        SmartDashboard.putBoolean("Shooter/CanShoot", this.canShoot());
    }
}
