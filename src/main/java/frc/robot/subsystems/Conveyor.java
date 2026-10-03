package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.DeviceId;


public class Conveyor extends SubsystemBase {
  /** Conveyor 的所有狀態，每個狀態對應一個輸出 */
  public enum State {
    IDLE(0.0),     // 停止
    FORWARD(Constants.Conveyor.FORWARD_SPEED), // 正轉（送料）
    REVERSE(Constants.Conveyor.REVERSE_SPEED), // 反轉（吐出卡住的物件）
    MANUAL(0.0);   // 自訂速度，實際輸出看 manualSpeed

    public final double speed;

    State(double speed) {
      this.speed = speed;
    }
  }

  private final TalonFX motor = new TalonFX(DeviceId.Conveyor.MOTOR);

  private State state = State.IDLE;
  private double manualSpeed = 0.0;

  public Conveyor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // 放開後立刻停住
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // 方向反了就改 Clockwise_Positive
    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);
  }

  /** 每個 loop 依照目前狀態輸出馬達 */
  @Override
  public void periodic() {
    double output = switch (state) {
      case MANUAL -> manualSpeed;
      default -> state.speed;
    };
    motor.set(output);

    SmartDashboard.putString("Conveyor/State", state.name());
    SmartDashboard.putNumber("Conveyor/Output", output);
  }

  public State getState() {
    return state;
  }

  public void setState(State newState) {
    state = newState;
  }

  /** 切換到指定狀態後維持（不會自動回 IDLE） */
  public Command setStateCommand(State newState) {
    return runOnce(() -> setState(newState));
  }

  /** 按住時維持指定狀態，放開回到 IDLE */
  public Command holdState(State newState) {
    return startEnd(() -> setState(newState), () -> setState(State.IDLE));
  }

  /** 按住時以自訂速度轉動，放開就停 */
  public Command run(double speed) {
    return startEnd(
        () -> {
          manualSpeed = speed;
          setState(State.MANUAL);
        },
        () -> setState(State.IDLE));
  }

  /** 正轉（按住送料） */
  public Command forward() {
    return holdState(State.FORWARD);
  }

  /** 反轉（吐出卡住的物件） */
  public Command reverse() {
    return holdState(State.REVERSE);
  }

  public void stop() {
    setState(State.IDLE);
  }
}
