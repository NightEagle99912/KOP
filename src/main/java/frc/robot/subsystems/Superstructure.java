package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopperSubsystem.Hopper;
import frc.robot.subsystems.intakeSubsystem.IntakeRoller;
import frc.robot.subsystems.intakeSubsystem.Lifter;
import frc.robot.subsystems.shooterSubsystem.Shooter;

/**
 * Superstructure（上層機構總狀態機）：統一指揮 Intake、Lifter、Hopper、Shooter。
 *
 * <p>外部只用 setWantedState() 說「想做什麼」（WantedState），
 * 這裡依感測器條件決定機器人「實際在做什麼」（SystemState），再把每個機構切到對應的狀態。
 * 例如想吸球時，要等 Lifter 放到底才開始轉滾輪；想射球時，要等飛輪到速才開始送球。
 *
 * <p>誰可以設定 WantedState：
 * <ul>
 *   <li>teleop：RobotContainer 讀搖桿設定（沒有 Command 在控制 Superstructure 時）</li>
 *   <li>自動模式：IntakeCommand / ShooterCommand，它們都會 require 這個 Subsystem，
 *       所以同一時間只會有一個 Command 在控制</li>
 * </ul>
 */
public class Superstructure extends SubsystemBase {
    /** 操作手想要做的事 */
    public enum WantedState {
        IDLE,           // 全部收起、停止
        INTAKE,         // 地面吸球
        EJECT,         // 反轉吐出卡住的物件
        PREPARE_SHOT,   // 預轉飛輪、調仰角，不送球
        SHOOT,          // 射球
        SOURCE_COLLECT, // 從 Source 經過 Shooter 吸球
        AMP             // 射 AMP
    }

    /** 機器人實際所在的狀態 */
    public enum SystemState {
        IDLE,
        DEPLOYING_INTAKE,  // Lifter 往下放，還沒到底（吸球、吐球前都會先經過這裡）
        INTAKING,          // Lifter 到底，滾輪吸球
        EJECTING,          // Lifter 到底，滾輪反轉吐出
        SPINNING_UP,       // 飛輪加速、仰角調整中
        SHOOTING,          // 飛輪到速，送球射擊
        SOURCE_COLLECTING,
        MOVING_TO_AMP,     // Lifter 移動到 AMP 角度中
        SCORING_AMP        // Lifter 到位，吐球進 AMP
    }

    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final Lifter lifter = new Lifter();
    private final Hopper hopper = new Hopper();
    private final Shooter shooter = new Shooter();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public WantedState getWantedState() {
        return this.wantedState;
    }

    public SystemState getSystemState() {
        return this.systemState;
    }

    /** 射擊距離（公尺） */
    public void setShootDistance(double distanceMeters) {
        this.shooter.setDistance(distanceMeters);
    }

    /**
     * 由 CommandScheduler 每個週期自動呼叫一次：決定狀態 → 設定各機構狀態。
     *
     * <p>CommandScheduler 依註冊順序呼叫 periodic()。Superstructure 會先註冊（super() 在欄位初始化之前），
     * 所以這裡一定比 IntakeRoller、Lifter、Hopper 的 periodic() 先跑，同一個週期內就會輸出馬達。
     * Shooter 不是 SubsystemBase，所以在這裡自己呼叫。
     */
    @Override
    public void periodic() {
        this.systemState = this.nextSystemState();
        this.applySystemState();

        this.shooter.periodic();

        SmartDashboard.putString("Superstructure/WantedState", this.wantedState.name());
        SmartDashboard.putString("Superstructure/SystemState", this.systemState.name());
    }

    /** 狀態轉移：依 WantedState 和感測器條件決定下一個 SystemState */
    private SystemState nextSystemState() {
        // Lifter 編碼器斷線時角度不可信：不進入、也不維持需要 Lifter 到位的狀態，滾輪會跟著停
        // （isAtMin / isAtAMP 斷線時本來就回傳 false，這裡再擋住「已經在做就維持」的情況）
        final boolean lifterOk = this.lifter.isEncoderConnected();

        return switch (this.wantedState) {
            case IDLE -> SystemState.IDLE;

            // 已經在吸球就維持，避免 Lifter 被地上的物件頂一下就停滾輪
            case INTAKE -> lifterOk && (this.systemState == SystemState.INTAKING || this.lifter.isAtMin())
                ? SystemState.INTAKING
                : SystemState.DEPLOYING_INTAKE;

            // 跟吸球一樣，Lifter 放到底才開始吐；已經在吐就維持
            case EJECT -> lifterOk && (this.systemState == SystemState.EJECTING || this.lifter.isAtMin())
                ? SystemState.EJECTING
                : SystemState.DEPLOYING_INTAKE;

            case PREPARE_SHOT -> SystemState.SPINNING_UP;

            // 不維持：飛輪掉速就退回 SPINNING_UP，Hopper 也一起停，避免物件卡在 Feeder 前面
            case SHOOT -> this.shooter.canShoot()
                ? SystemState.SHOOTING
                : SystemState.SPINNING_UP;

            case SOURCE_COLLECT -> SystemState.SOURCE_COLLECTING;

            case AMP -> lifterOk && (this.systemState == SystemState.SCORING_AMP || this.lifter.isAtAMP())
                ? SystemState.SCORING_AMP
                : SystemState.MOVING_TO_AMP;
        };
    }

    /** 每個 SystemState 對應的各機構狀態 */
    private void applySystemState() {
        switch (this.systemState) {
            case IDLE -> this.setMechanisms(
                Lifter.State.STOW, IntakeRoller.State.STOPPED, Hopper.State.STOPPED, Shooter.State.STOPPED);
            case DEPLOYING_INTAKE -> this.setMechanisms(
                Lifter.State.DEPLOY, IntakeRoller.State.STOPPED, Hopper.State.STOPPED, Shooter.State.STOPPED);
            case INTAKING -> this.setMechanisms(
                Lifter.State.DEPLOY, IntakeRoller.State.INTAKE, Hopper.State.STOPPED, Shooter.State.STOPPED);
            case EJECTING -> this.setMechanisms(
                Lifter.State.DEPLOY, IntakeRoller.State.RELEASE, Hopper.State.REVERSE, Shooter.State.STOPPED);
            case SPINNING_UP -> this.setMechanisms(
                Lifter.State.STOW, IntakeRoller.State.STOPPED, Hopper.State.STOPPED, Shooter.State.AIMING);
            case SHOOTING -> this.setMechanisms(
                Lifter.State.STOW, IntakeRoller.State.STOPPED, Hopper.State.FEED, Shooter.State.SHOOTING);
            case SOURCE_COLLECTING -> this.setMechanisms(
                Lifter.State.STOW, IntakeRoller.State.STOPPED, Hopper.State.REVERSE, Shooter.State.SOURCE_COLLECT);
            case MOVING_TO_AMP -> this.setMechanisms(
                Lifter.State.AMP, IntakeRoller.State.STOPPED, Hopper.State.STOPPED, Shooter.State.STOPPED);
            case SCORING_AMP -> this.setMechanisms(
                Lifter.State.AMP, IntakeRoller.State.AMP, Hopper.State.STOPPED, Shooter.State.STOPPED);
        }
    }

    private void setMechanisms(
        Lifter.State lifterState,
        IntakeRoller.State intakeState,
        Hopper.State hopperState,
        Shooter.State shooterState
    ) {
        this.lifter.setState(lifterState);
        this.intakeRoller.setState(intakeState);
        this.hopper.setState(hopperState);
        this.shooter.setState(shooterState);
    }
}
