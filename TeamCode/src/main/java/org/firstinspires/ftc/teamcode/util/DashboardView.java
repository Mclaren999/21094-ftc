package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * FTC Dashboard: http://192.168.43.1:8080/dash (телефон/ноут подключён к Wi-Fi робота).
 *
 * Каждый цикл рисует на поле робота, турель, цель и точку прицела (SWM),
 * а числа отправляет в Telemetry/Graph. Вызывать send(...) один раз за цикл.
 *
 * Цвета на поле:
 *   синий/красный квадрат — робот, линия — куда смотрит нос
 *   жёлтая линия          — направление турели на точку прицела
 *   зелёный круг          — настоящая цель, оранжевый — точка прицела с поправкой на движение
 */
@Config
public final class DashboardView {

    /** Выключить, если Dashboard тормозит цикл. */
    public static boolean ENABLED = true;

    /**
     * Поворот рисунка относительно поля Dashboard, градусы (0, 90, 180, 270).
     * Если робот на картинке ездит "не туда", подберите значение — на сам робот это не влияет.
     */
    public static double FIELD_ROTATION_DEG = 0;

    public static double ROBOT_SIZE = 18; // дюймы

    private static final double FIELD_HALF = 72; // Pedro: 0..144, Dashboard: -72..72

    private DashboardView() {
    }

    public static void send(Pose pose, ShooterSubsystem shooter, IntakeSubsystem intake, boolean blue) {
        if (!ENABLED) return;

        TelemetryPacket packet = new TelemetryPacket();

        // Числа — их можно выводить на графики (вкладка Graph)
        packet.put("x", pose.x());
        packet.put("y", pose.y());
        packet.put("heading (deg)", Math.toDegrees(pose.heading()));
        packet.put("flywheel target", ShooterSubsystem.vel);
        packet.put("flywheel actual", shooter.getVel());
        packet.put("distance to goal", shooter.getDistance());
        packet.put("turret angle", shooter.getAngle());
        packet.put("turret servo", shooter.getPos());
        packet.put("ready to shoot", shooter.isReady());
        packet.put("intake", intake.getState().toString());
        packet.put("flower", intake.getFlowerState().toString());

        Canvas field = packet.fieldOverlay();
        drawGoal(field, shooter);
        drawRobot(field, pose, blue ? "#3F51B5" : "#E53935");
        drawTurret(field, shooter);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /* ================= DRAWING ================= */

    private static void drawRobot(Canvas field, Pose pose, String color) {
        double half = ROBOT_SIZE / 2;
        double cos = Math.cos(pose.heading());
        double sin = Math.sin(pose.heading());

        double[] cornersX = {half, half, -half, -half};
        double[] cornersY = {half, -half, -half, half};
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++) {
            double fx = pose.x() + cornersX[i] * cos - cornersY[i] * sin;
            double fy = pose.y() + cornersX[i] * sin + cornersY[i] * cos;
            xs[i] = toDashX(fx, fy);
            ys[i] = toDashY(fx, fy);
        }

        field.setStrokeWidth(1);
        field.setStroke(color);
        field.strokePolygon(xs, ys);

        // Нос робота
        double noseX = pose.x() + half * cos;
        double noseY = pose.y() + half * sin;
        line(field, pose.x(), pose.y(), noseX, noseY);
    }

    private static void drawTurret(Canvas field, ShooterSubsystem shooter) {
        field.setStroke("#FDD835");
        field.setStrokeWidth(1);
        line(field, shooter.getTurretX(), shooter.getTurretY(), shooter.getAimX(), shooter.getAimY());
    }

    private static void drawGoal(Canvas field, ShooterSubsystem shooter) {
        field.setStrokeWidth(1);
        field.setStroke("#43A047");
        circle(field, shooter.getGoalX(), shooter.getGoalY(), 4);

        field.setStroke("#FB8C00");
        circle(field, shooter.getAimX(), shooter.getAimY(), 2);
    }

    /* ================= COORDINATES ================= */

    private static void line(Canvas field, double x1, double y1, double x2, double y2) {
        field.strokeLine(toDashX(x1, y1), toDashY(x1, y1), toDashX(x2, y2), toDashY(x2, y2));
    }

    private static void circle(Canvas field, double x, double y, double radius) {
        field.strokeCircle(toDashX(x, y), toDashY(x, y), radius);
    }

    /** Pedro (угол поля = 0,0) -> Dashboard (центр поля = 0,0), с поворотом FIELD_ROTATION_DEG. */
    private static double toDashX(double x, double y) {
        double r = Math.toRadians(FIELD_ROTATION_DEG);
        return (x - FIELD_HALF) * Math.cos(r) - (y - FIELD_HALF) * Math.sin(r);
    }

    private static double toDashY(double x, double y) {
        double r = Math.toRadians(FIELD_ROTATION_DEG);
        return (x - FIELD_HALF) * Math.sin(r) + (y - FIELD_HALF) * Math.cos(r);
    }
}
