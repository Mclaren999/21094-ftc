package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * Калибровка турели на моторе.
 *
 * ШАГ 1 — тики на оборот (режим "РУКАМИ", включён при старте, мотор свободно крутится):
 *   1. Поставить турель строго НАЗАД и нажать X (ноль энкодера).
 *   2. Повернуть турель рукой ровно на один оборот ВЛЕВО (против часовой, если смотреть сверху).
 *   3. Тики должны стать положительными. Если отрицательные — поменять turretReversed.
 *   4. Число "-> turretTicksPerRev" вписать в ShooterSubsystem.
 *
 * ШАГ 2 — проверка регулятора (A — переключить режим на "МОТОРОМ"):
 *   dpad влево/вправо — цель +/-90°   LB/RB — цель +/-10°   Y — цель 0°
 *   kP / kD / kS подбираются в Panels, пока турель не встаёт быстро и без раскачки.
 *
 * Робот стоит, моторы езды не используются.
 */
@Configurable
@TeleOp(name = "Turret Calibration", group = "Tuning")
public class TurretCalibration extends OpMode {

    private ShooterSubsystem shooter;
    public static double targetDeg = 0;

    private boolean motorMode = false;
    private boolean lastLeft, lastRight, lastLb, lastRb, lastA, lastX, lastY;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.resetTurretEncoder();
        shooter.setTurretBrake(false);
        targetDeg = 0;
    }

    @Override
    public void loop() {
        if (pressed(gamepad1.a, lastA)) {
            motorMode = !motorMode;
            shooter.setTurretBrake(motorMode);
            targetDeg = shooter.getTurretDeg(); // не дёргаться при переключении
        }
        if (pressed(gamepad1.x, lastX)) {
            shooter.resetTurretEncoder();
            targetDeg = 0;
        }
        if (pressed(gamepad1.dpad_left, lastLeft))  targetDeg += 90;
        if (pressed(gamepad1.dpad_right, lastRight)) targetDeg -= 90;
        if (pressed(gamepad1.left_bumper, lastLb))  targetDeg += 10;
        if (pressed(gamepad1.right_bumper, lastRb)) targetDeg -= 10;
        if (pressed(gamepad1.y, lastY))             targetDeg = 0;
        targetDeg = Math.max(ShooterSubsystem.turretMinDeg, Math.min(ShooterSubsystem.turretMaxDeg, targetDeg));

        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastLb = gamepad1.left_bumper;
        lastRb = gamepad1.right_bumper;
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (motorMode) {
            shooter.setTurretAngle(targetDeg);
            shooter.update(); // маховик выключен — крутится только турель
        } else {
            shooter.stop();
        }

        int ticks = shooter.getTurretTicks();
        if (shooter.isTurretFault()) {
            telemetry.addLine("!!! АВАРИЯ: турель ушла за ход, мотор выключен — поменяйте turretReversed и перезапустите");
        }
        telemetry.addData("Режим (A)", motorMode ? "МОТОРОМ" : "РУКАМИ");
        telemetry.addData("Тики", ticks);
        telemetry.addData("Угол турели", "%.1f°", shooter.getTurretDeg());
        telemetry.addData("Скорость турели", "%.0f °/с", shooter.getTurretVelDeg());
        if (motorMode) {
            telemetry.addData("Цель", "%.1f°", targetDeg);
            telemetry.addData("Ошибка", "%.2f°", targetDeg - shooter.getTurretDeg());
        } else {
            telemetry.addLine();
            telemetry.addLine("Повернуть рукой ровно на 1 оборот влево:");
            telemetry.addData("-> turretTicksPerRev", Math.abs(ticks));
            telemetry.addData("-> передача (мотор:турель)", "%.2f : 1", Math.abs(ticks) / 145.1);
            telemetry.addData("-> turretReversed", ticks < 0
                    ? "поменять на " + !ShooterSubsystem.turretReversed + " (тики отрицательные)"
                    : "оставить " + ShooterSubsystem.turretReversed);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) shooter.stop();
    }

    private static boolean pressed(boolean now, boolean last) {
        return now && !last;
    }
}
