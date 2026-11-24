package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {

    /* ================= HARDWARE ================= */

    // Единственный мотор маховика. Его энкодер должен быть подключён.
    private static final String FLYWHEEL_NAME = "flywheel";
    // Мотор турели (goBILDA 1150 rpm). Его энкодер должен быть подключён.
    private static final String TURRET_NAME = "turret";

    private final DcMotorEx flywheel;
    private final DcMotorEx turret;

    /* ================= TURRET CONSTANTS ================= */
    // Не final — чтобы их можно было менять через Panels (@Configurable)

    // Смещение турели относительно центра робота
    public static double turretOffsetX = -2.6;
    public static double turretOffsetY = 0;

    // Куда смотрит турель в нулевом положении, градусы от носа робота:
    // 0 — вперёд, 180 — назад (турель установлена развёрнутой).
    // Ноль энкодера ставится при resetTurretEncoder(): в этот момент турель должна смотреть сюда.
    public static double turretCenterHeadingDeg = 180;

    // Тики энкодера на ОДИН оборот турели.
    // goBILDA 1150 rpm = 145.1 тика на оборот вала мотора, умножить на передачу до турели.
    // Точное число меряется в OpMode "Turret Calibration".
    public static double turretTicksPerRev = 145.1;
    // true, если при повороте турели влево (против часовой, если смотреть сверху) тики уменьшаются.
    public static boolean turretReversed = false;

    // Ход турели от нулевого положения, градусы (+ = влево / против часовой).
    public static double turretMinDeg = -180;
    public static double turretMaxDeg = 180;
    // Цель у края хода: турель стоит в краю и перекидывается на другую сторону,
    // только когда цель ушла за край больше чем на столько градусов (чтобы не дёргалась туда-сюда).
    public static double turretFlipHysteresisDeg = 15;
    // Если турель ушла за ход дальше чем на столько градусов — авария: мощность снимается.
    public static double TURRET_FAULT_MARGIN_DEG = 30;

    // Регулятор положения турели:
    //   kP — мощность на 1° ошибки, kD — гасит раскачку (мощность на 1°/с скорости турели),
    //   kS — минимальная мощность, чтобы сдвинуть турель с места (трение),
    //   omegaFF — подкручивать турель навстречу повороту робота (0 — выкл, 1 — полностью).
    // Турель качается вокруг цели — уменьшить kP или увеличить kD. Не доходит — увеличить kS.
    public static double turretKP = 0.015;
    public static double turretKD = 0.0006;
    public static double turretKS = 0.04;
    public static double turretOmegaFF = 1.0;
    public static double turretMaxPower = 0.8;
    // Ошибка меньше этого (градусы) не исправляется — чтобы турель не дрожала.
    public static double turretToleranceDeg = 0.75;
    // goBILDA 1150 rpm: 145.1 тика на оборот вала, 1150 об/мин = 6900 °/с без нагрузки.
    // Отсюда и из turretTicksPerRev считается макс. скорость турели (нужна только для omegaFF).
    private static final double MOTOR_TICKS_PER_REV = 145.1;
    private static final double MOTOR_FREE_SPEED_DEG_PER_SEC = 1150 / 60.0 * 360;

    /* ================= AIM CONSTANTS ================= */

    // Точки прицеливания
    // X цели по альянсу. Красная — зеркало синей относительно x = 72 (144 - 57 = 87).
    public static double blueGoalX = 57;
    public static double redGoalX  = 87;
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
    // Задержка турели и цикла, с: на столько вперёд предсказывается позиция робота.
    public static double aimLatencySec = 0;

    /* ================= FLYWHEEL CONSTANTS ================= */

    // Регулятор маховика.
    //   V — мощность на 1 тик/с (V = 1 / макс. скорость; 0.00033 -> ~3000 тиков/с на полной мощности)
    //   S — мощность на трение, P — поправка на ошибку скорости
    //   threshold — если ошибка больше, мотор просто даёт 1.0 (разгон) или 0 (выбег)
    public static double P = 0.002;
    public static double V = 0.00033;
    public static double S = 0.05;
    public static int threshold = 200;
    // Насколько скорость может отличаться от цели, чтобы считать маховик готовым (isReady), тики/с.
    public static int readyTolerance = 100;

    private static final double FIELD_MID_Y = 72;

    /* ================= STATE ================= */

    public static int vel = 0;              // целевая скорость маховика, тики/с
    public static double turretTargetDeg;   // куда крутим турель, градусы от нуля (+ = влево)
    public static double angleToGoal;       // угол до цели относительно робота, градусы

    private boolean flywheelEnabled = false;
    private double distanceToGoal = 0;
    private double fixedVelocity = 0;
    // Угловая скорость робота из последнего alignTurret, °/с — для omegaFF
    private double robotOmegaDeg = 0;
    // Авария турели (ушла за ход) — мотор выключен до перезапуска OpMode
    private boolean turretFault = false;

    /* ================= INIT ================= */

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);

        // Энкодер здесь НЕ сбрасывается: TeleOp после автонома продолжает с его нулём.
        // Сброс — resetTurretEncoder() (автономы и TeleOp без автонома).
        turret = hardwareMap.get(DcMotorEx.class, TURRET_NAME);
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        turretTargetDeg = 0;
        angleToGoal = turretCenterHeadingDeg;
    }

    /* ================= TURRET ================= */

    /**
     * Текущее положение турели принимается за ноль (смотрит на turretCenterHeadingDeg).
     * Вызывать в init, когда турель стоит в нулевом положении. Мотор не двигается.
     */
    public void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretTargetDeg = 0;
        turretFault = false; // новый ноль — снимаем аварию
        angleToGoal = turretCenterHeadingDeg;
    }

    /** Крутить турель на угол от нуля, градусы (+ = влево). Едет в update(). */
    public void setTurretAngle(double degrees) {
        turretTargetDeg = clamp(degrees, turretMinDeg, turretMaxDeg);
        robotOmegaDeg = 0;
    }

    /** Турель в ноль (при развёрнутой турели смотрит назад). */
    public void centerTurret() {
        setTurretAngle(0);
        angleToGoal = turretCenterHeadingDeg;
    }

    /** BRAKE — держит положение без мощности, FLOAT — крутится от руки (для калибровки). */
    public void setTurretBrake(boolean brake) {
        turret.setZeroPowerBehavior(brake
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Наводит турель на цель по позиции робота. Мотор крутится в update().
     *
     * @param heading курс робота в радианах
     */
    public void alignTurret(double x, double y, double heading, boolean blue) {
        alignTurret(x, y, heading, 0, 0, 0, blue);
    }

    /**
     * Наводит турель с учётом движения робота (стрельба на ходу). Мотор крутится в update().
     *
     * @param heading курс робота в радианах
     * @param vx      скорость робота по X поля, дюймы/с
     * @param vy      скорость робота по Y поля, дюймы/с
     * @param omega   угловая скорость робота, рад/с
     */
    public void alignTurret(double x, double y, double heading, double vx, double vy, double omega, boolean blue) {
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

        // Угол, на который турель должна отвернуться от нулевого положения
        turretTargetDeg = chooseTurretRelative(normalizeDegrees(angleToGoal - turretCenterHeadingDeg));
        distanceToGoal = Math.hypot(aimX - turretX, aimY - turretY);
        robotOmegaDeg = Math.toDegrees(omega);
    }

    /** Старый вызов: наводит турель и сразу крутит маховик. */
    @Deprecated
    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry) {
        setFlywheelEnabled(true);
        alignTurret(x, y, heading, blue);
        update();
    }

    /** Держит турель на turretTargetDeg. Вызывается из update(). */
    private void updateTurret() {
        double actual = getTurretDeg();

        // Защита: турель ушла далеко за ход — скорее всего, перепутан turretReversed
        // (регулятор крутит "от цели") или отвалился энкодер. Снимаем мощность, чтобы
        // не намотать и не порвать провода. Снимается перезапуском OpMode или resetTurretEncoder().
        if (actual > turretMaxDeg + TURRET_FAULT_MARGIN_DEG || actual < turretMinDeg - TURRET_FAULT_MARGIN_DEG) {
            turretFault = true;
        }
        if (turretFault) {
            setTurretPower(0);
            return;
        }

        double error = turretTargetDeg - actual;

        // Робот крутится на omega — относительно робота цель уезжает на -omega.
        double desiredRate = -robotOmegaDeg * turretOmegaFF;
        double maxSpeed = MOTOR_FREE_SPEED_DEG_PER_SEC * MOTOR_TICKS_PER_REV / turretTicksPerRev;
        double feedForward = desiredRate / maxSpeed;
        double damping = turretKD * (desiredRate - getTurretVelDeg());

        double power;
        if (Math.abs(error) <= turretToleranceDeg) {
            power = feedForward;
        } else {
            power = turretKP * error + damping + feedForward + Math.signum(error) * turretKS;
        }
        setTurretPower(clamp(power, -turretMaxPower, turretMaxPower));
    }

    private void setTurretPower(double power) {
        turret.setPower(turretReversed ? -power : power);
    }

    /* ================= FLYWHEEL ================= */

    public void setFlywheelEnabled(boolean enabled) {
        flywheelEnabled = enabled;
    }

    public boolean isFlywheelEnabled() {
        return flywheelEnabled;
    }

    /**
     * Фиксированная скорость маховика (тики/с) вместо скорости по дистанции.
     * 0 — снова считать по дистанции.
     */
    public void setFixedVelocity(double ticksPerSecond) {
        fixedVelocity = ticksPerSecond;
    }

    /** Вызывать каждый цикл: держит скорость маховика и положение турели. */
    public void update() {
        double target = fixedVelocity > 0 ? fixedVelocity : velocityForDistance(distanceToGoal);
        vel = flywheelEnabled ? (int) Math.max(0, target) : 0;

        flywheel.setPower(vel > 0 ? PVSController(P, V, S, threshold, vel, getVel()) : 0);
        updateTurret();
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
        return flywheelEnabled && vel > 0 && Math.abs(vel - getVel()) <= readyTolerance;
    }

    /** true, когда турель доехала до цели (с запасом в 3 допуска). */
    /** true — турель ушла за ход, мотор выключен (проверьте turretReversed и энкодер). */
    public boolean isTurretFault() {
        return turretFault;
    }

    public boolean isTurretOnTarget() {
        return Math.abs(turretTargetDeg - getTurretDeg()) <= 3 * turretToleranceDeg;
    }

    /** Выключает маховик и снимает мощность с турели (до следующего update()). */
    public void stop() {
        flywheelEnabled = false;
        vel = 0;
        flywheel.setPower(0);
        turret.setPower(0);
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
    /** Цель турели, градусы от нуля. */
    public double getPos()   { return turretTargetDeg; }
    public double getAngle() { return angleToGoal; }
    public double getDistance() { return distanceToGoal; }

    /** Сырые тики энкодера турели (со знаком turretReversed). */
    public int getTurretTicks() {
        int ticks = turret.getCurrentPosition();
        return turretReversed ? -ticks : ticks;
    }

    /** Текущий угол турели от нуля, градусы (+ = влево). */
    public double getTurretDeg() {
        return getTurretTicks() * 360.0 / turretTicksPerRev;
    }

    /** Текущая скорость турели, °/с. */
    public double getTurretVelDeg() {
        double ticksPerSec = turret.getVelocity();
        if (turretReversed) ticksPerSec = -ticksPerSec;
        return ticksPerSec * 360.0 / turretTicksPerRev;
    }

    /* ================= HELPERS ================= */

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Один и тот же угол можно получить как a, a+360 или a-360. Берём вариант, который
     * помещается в ход турели и ближе всего к текущей цели:
     * так она не делает полный оборот, когда цель проходит через край хода.
     */
    private double chooseTurretRelative(double relative) {
        double min = turretMinDeg;
        double max = turretMaxDeg;
        double current = turretTargetDeg;

        double best = relative;
        boolean found = false;
        for (double candidate : new double[]{relative, relative + 360, relative - 360}) {
            if (candidate < min || candidate > max) continue;
            if (!found || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
                found = true;
            }
        }
        if (!found) {
            // Цель в "мёртвой зоне" хода — встаём в край, который к ней ближе по кругу.
            double toMin = Math.abs(normalizeDegrees(relative - min));
            double toMax = Math.abs(normalizeDegrees(relative - max));
            return toMin < toMax ? min : max;
        }

        // Переброс на другую сторону (поворот больше чем на 180°): делаем его, только если цель
        // ушла за край хода дальше гистерезиса. Иначе держим турель в ближнем краю.
        if (Math.abs(best - current) > 180) {
            double sameSide = best + (best < current ? 360 : -360); // тот же угол со стороны турели
            double edge = sameSide > max ? max : min;
            if (Math.abs(sameSide - edge) < turretFlipHysteresisDeg) {
                return edge;
            }
        }
        return best;
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
