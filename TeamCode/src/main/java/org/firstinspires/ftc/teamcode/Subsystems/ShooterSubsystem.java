package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config        // FTC Dashboard
@Configurable  // Panels
public class ShooterSubsystem {

    /* ================= HARDWARE ================= */

    // Единственный мотор маховика. Его энкодер должен быть подключён.
    private static final String FLYWHEEL_NAME = "flywheel";

    private final DcMotorEx flywheel;
    private final Servo turretLeft;
    private final Servo turretRight;

    /* ================= CONSTANTS ================= */
    // Не final — чтобы их можно было менять через Panels (@Configurable)

    // Смещение турели относительно центра робота
    public static double turretOffsetX = -3;
    public static double turretOffsetY = 0;

    // Позиция сервы в середине хода
    public static double turretCenter = 0.5;
    // Куда смотрит турель при turretCenter, градусы от носа робота:
    // 0 — вперёд, 180 — назад (турель установлена развёрнутой)
    public static double turretCenterHeadingDeg = 180;
    // Сколько градусов поворота турели дают позиции сервы 0..1
    public static double turretRangeDeg = 360;

    // Точки прицеливания
    public static double blueGoalX = 83;
    public static double redGoalX  = 60;
    public static double goalY1 = 60; // когда робот в нижней половине поля (y < 72)
    public static double goalY2 = 84; // когда робот в верхней половине поля

    // Скорость маховика по расстоянию до цели (тики/с).
    // Если в SHOT_TABLE есть хотя бы две строки, скорость берётся из таблицы
    // (линейная интерполяция между строками). Иначе: fSlope * расстояние + fIntercept.
    public static double fIntercept = 2070;
    public static double fSlope = 0;

    // { расстояние до цели в дюймах, скорость маховика в тиках/с }, по возрастанию расстояния.
    // Заполняется по замерам на поле: встать на дистанцию, подобрать скорость, записать строку.
    public static double[][] SHOT_TABLE = {
            // { 40, 1900 },
            // { 70, 2070 },
            // { 100, 2300 },
    };

    // Стрельба на ходу (SWM): прицел и дистанция считаются по "виртуальной цели",
    // сдвинутой против движения робота на время полёта мяча.
    public static boolean swmEnabled = true;
    // Горизонтальная скорость мяча, дюймы/с. ОЦЕНКА — подобрать на поле:
    // если на ходу мяч уходит по направлению движения робота — уменьшить, если против — увеличить.
    public static double ballSpeed = 220;
    // Задержка сервы турели и цикла, с: на столько вперёд предсказывается позиция робота.
    public static double aimLatencySec = 0.0;

    // Регулятор маховика
    public static double P = 0.003;
    public static double V = 0.00032;
    public static double S = 0.05;
    public static int threshold = 80;

    private static final double FIELD_MID_Y = 72;

    /* ================= STATE ================= */

    public static int vel = 0;            // целевая скорость маховика, тики/с
    public static double targetAngleDeg;  // позиция сервы турели (0..1)
    public static double angleToGoal;     // угол до цели относительно робота, градусы

    private boolean flywheelEnabled = false;
    private double distanceToGoal = 0;

    // Последнее наведение (координаты поля, дюймы) — для отрисовки в Dashboard
    private double lastTurretX, lastTurretY;
    private double lastAimX, lastAimY;
    private double lastGoalX, lastGoalY;

    /* ================= INIT ================= */

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        turretLeft = hardwareMap.get(Servo.class, "leftTurret");
        turretRight = hardwareMap.get(Servo.class, "rightTurret");
        turretLeft.setDirection(Servo.Direction.FORWARD);
        turretRight.setDirection(Servo.Direction.FORWARD);
    }

    /* ================= TURRET ================= */

    public void setTurretPosition(double position) {
        turretLeft.setPosition(position);
        turretRight.setPosition(position);
    }

    /** Ставит турель в середину хода сервы (при развёрнутой турели она смотрит назад). */
    public void centerTurret() {
        angleToGoal = turretCenterHeadingDeg;
        targetAngleDeg = turretCenter;
        setTurretPosition(targetAngleDeg);
    }

    /**
     * Наводит турель на цель по позиции робота.
     * Маховиком не управляет — для этого есть update().
     *
     * @param heading курс робота в радианах
     */
    public void alignTurret(double x, double y, double heading, boolean blue) {
        alignTurret(x, y, heading, 0, 0, 0, blue);
    }

    /**
     * Наводит турель с учётом движения робота (стрельба на ходу).
     *
     * @param heading курс робота в радианах
     * @param vx      скорость робота по X поля, дюймы/с
     * @param vy      скорость робота по Y поля, дюймы/с
     * @param omega   угловая скорость робота, рад/с
     */
    public void alignTurret(double x, double y, double heading,
                            double vx, double vy, double omega, boolean blue) {
        // Где робот окажется, когда команда дойдёт до турели
        x += vx * aimLatencySec;
        y += vy * aimLatencySec;
        heading += omega * aimLatencySec;

        // Позиция турели на поле с учётом её смещения от центра робота
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double offsetX = turretOffsetX * cos - turretOffsetY * sin;
        double offsetY = turretOffsetX * sin + turretOffsetY * cos;
        double turretX = x + offsetX;
        double turretY = y + offsetY;

        // Скорость самой турели: скорость робота + вклад от его вращения
        double turretVx = vx - omega * offsetY;
        double turretVy = vy + omega * offsetX;

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = turretY < FIELD_MID_Y ? goalY1 : goalY2;

        // Виртуальная цель: мяч наследует скорость робота, поэтому целимся
        // в точку, сдвинутую против движения на (скорость * время полёта).
        double aimX = goalX;
        double aimY = goalY;
        if (swmEnabled && ballSpeed > 0) {
            for (int i = 0; i < 8; i++) { // время полёта зависит от дистанции — уточняем
                double flightTime = Math.hypot(aimX - turretX, aimY - turretY) / ballSpeed;
                aimX = goalX - turretVx * flightTime;
                aimY = goalY - turretVy * flightTime;
            }
        }

        // Угол до точки прицеливания на поле -> угол относительно робота
        double fieldAngle = Math.toDegrees(Math.atan2(aimY - turretY, aimX - turretX));
        angleToGoal = normalizeDegrees(fieldAngle - Math.toDegrees(heading));

        // Угол, на который турель должна отвернуться от своего положения при turretCenter
        double turretRelative = normalizeDegrees(angleToGoal - turretCenterHeadingDeg);
        targetAngleDeg = clamp(turretCenter + turretRelative / turretRangeDeg, 0, 1);
        distanceToGoal = Math.hypot(aimX - turretX, aimY - turretY);

        lastTurretX = turretX;
        lastTurretY = turretY;
        lastAimX = aimX;
        lastAimY = aimY;
        lastGoalX = goalX;
        lastGoalY = goalY;

        setTurretPosition(targetAngleDeg);
    }

    /** Старый вызов: наводит турель и сразу крутит маховик. */
    @Deprecated
    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry) {
        setFlywheelEnabled(true);
        alignTurret(x, y, heading, blue);
        update();
    }

    /* ================= FLYWHEEL ================= */

    public void setFlywheelEnabled(boolean enabled) {
        flywheelEnabled = enabled;
    }

    public boolean isFlywheelEnabled() {
        return flywheelEnabled;
    }

    /** Вызывать каждый цикл: держит скорость маховика. */
    public void update() {
        vel = flywheelEnabled ? (int) Math.max(0, velocityForDistance(distanceToGoal)) : 0;

        flywheel.setPower(vel > 0 ? PVSController(P, V, S, threshold, vel, getVel()) : 0);
    }

    /** Нужная скорость маховика (тики/с) для расстояния до цели (дюймы). */
    public static double velocityForDistance(double distance) {
        double[][] table = SHOT_TABLE;
        if (table.length < 2) {
            return fSlope * distance + fIntercept;
        }

        // За краями таблицы держим крайние значения
        if (distance <= table[0][0]) return table[0][1];
        if (distance >= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 1; i < table.length; i++) {
            if (distance <= table[i][0]) {
                double ratio = (distance - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
                return table[i - 1][1] + ratio * (table[i][1] - table[i - 1][1]);
            }
        }
        return table[table.length - 1][1];
    }

    /** true, когда маховик включён и набрал нужную скорость — можно стрелять. */
    public boolean isReady() {
        return flywheelEnabled && vel > 0 && Math.abs(vel - getVel()) <= threshold;
    }

    /** Выключает маховик. */
    public void stop() {
        flywheelEnabled = false;
        vel = 0;
        flywheel.setPower(0);
    }

    public static double PVSController(double kP, double kV, double kS,
                                       double threshold, double target, double current) {
        double error = target - current;

        // Далеко от цели: полная мощность, если медленно; выбег, если быстро
        if (Math.abs(error) > threshold) {
            return error > 0 ? 1.0 : 0.0;
        }

        return clamp(kV * target + kP * error + kS, 0, 1);
    }

    /* ================= GETTERS ================= */

    public double getVel()   { return flywheel.getVelocity(); }
    public double getPos()   { return targetAngleDeg; }
    public double getAngle() { return angleToGoal; }
    public double getDistance() { return distanceToGoal; }

    public double getTurretX() { return lastTurretX; }
    public double getTurretY() { return lastTurretY; }
    /** Точка, куда реально целится турель (с поправкой SWM). */
    public double getAimX()    { return lastAimX; }
    public double getAimY()    { return lastAimY; }
    /** Настоящая цель. */
    public double getGoalX()   { return lastGoalX; }
    public double getGoalY()   { return lastGoalY; }

    /* ================= HELPERS ================= */

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
