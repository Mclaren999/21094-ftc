package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * Калибровка турели: вручную крутим серву и записываем позиции.
 *
 * gamepad1:
 *   dpad влево/вправо — позиция -/+ 0.005     LB/RB — позиция -/+ 0.05
 *   A — поставить 0.5
 *   X — запомнить "смотрит строго НАЗАД"
 *   Y — запомнить "смотрит строго ВЛЕВО" (90° от носа)
 *   B — запомнить "смотрит строго ВПРАВО"
 *
 * Робот стоит, моторы езды не используются. Направление проще всего проверять
 * по линиям на плитках поля или угольником.
 */
@Configurable
@TeleOp(name = "Turret Calibration", group = "Tuning")
public class TurretCalibration extends OpMode {

    private ShooterSubsystem shooter;
    public static double position = 0;

    private double backPos = Double.NaN;
    private double leftPos = Double.NaN;
    private double rightPos = Double.NaN;

    private boolean lastLeft, lastRight, lastLb, lastRb, lastA, lastX, lastY, lastB;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.setTurretPosition(position);
    }

    @Override
    public void loop() {
        if (pressed(gamepad1.dpad_left, lastLeft))   position -= 0.005;
        if (pressed(gamepad1.dpad_right, lastRight)) position += 0.005;
        if (pressed(gamepad1.left_bumper, lastLb))   position -= 0.05;
        if (pressed(gamepad1.right_bumper, lastRb))  position += 0.05;
        if (pressed(gamepad1.a, lastA))              position = 0.5;
        position = Math.max(0, Math.min(1, position));

        if (pressed(gamepad1.x, lastX)) backPos = position;
        if (pressed(gamepad1.y, lastY)) leftPos = position;
        if (pressed(gamepad1.b, lastB)) rightPos = position;

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastLb = gamepad1.left_bumper;
        lastRb = gamepad1.right_bumper;
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;

        shooter.setTurretPosition(position);

        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addLine();
        telemetry.addData("НАЗАД  (X)", fmt(backPos));
        telemetry.addData("ВЛЕВО  (Y)", fmt(leftPos));
        telemetry.addData("ВПРАВО (B)", fmt(rightPos));

        if (!Double.isNaN(leftPos) && !Double.isNaN(rightPos)) {
            double range = 180.0 / Math.abs(rightPos - leftPos);
            telemetry.addLine();
            telemetry.addData("-> turretRangeDeg", "%.1f", range);
            if (!Double.isNaN(backPos)) {
                double leftSide = Math.abs(backPos - leftPos) * range;
                double rightSide = Math.abs(rightPos - backPos) * range;
                telemetry.addData("-> turretCenter", "%.3f", backPos);
                telemetry.addData("   градусов назад->влево", "%.1f (должно быть 90)", leftSide);
                telemetry.addData("   градусов назад->вправо", "%.1f (должно быть 90)", rightSide);
            }
        }
        telemetry.update();
    }

    private static boolean pressed(boolean now, boolean last) {
        return now && !last;
    }

    private static String fmt(double value) {
        return Double.isNaN(value) ? "-" : String.format("%.3f", value);
    }
}
