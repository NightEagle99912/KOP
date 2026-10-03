package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

/** ShootCalculator（彈道計算）：依距離查表，中間值自動線性內插。表格在 Constants.Shooter.SHOOT_TABLE */
public class ShootCalculator {
    /** 射擊參數：飛輪轉速（RPS）、仰角（度） */
    public record ShootParams(double flywheelRps, double hoodDegrees) {}

    private final InterpolatingDoubleTreeMap rpsTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    public ShootCalculator() {
        for (double[] point : Constants.Shooter.SHOOT_TABLE) {
            this.addPoint(point[0], point[1], point[2]);
        }
    }

    private void addPoint(double distanceMeters, double rps, double hoodDegrees) {
        this.rpsTable.put(distanceMeters, rps);
        this.hoodTable.put(distanceMeters, hoodDegrees);
    }

    /** 超出表格範圍時會使用最近的端點值 */
    public ShootParams calculate(double distanceMeters) {
        return new ShootParams(
            this.rpsTable.get(distanceMeters),
            this.hoodTable.get(distanceMeters)
        );
    }
}
