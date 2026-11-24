package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
//bismillah
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.FlowerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.BooleanSupplier;

/**
 * Общая логика TeleOp для обоих альянсов.
 * BlueSoloDrive и RedSoloDrive задают только альянс и стартовые позиции.
 *
 * Управление (gamepad1):
 *   стики — езда (field-centric)      L3 — сброс позиции
 *   D-pad ↑ — переключить field-centric / robot-centric
 *   Турель наводится на цель сама (мотор, с учётом движения робота).
 *     △ (треугольник) — выключить автоприцел и поставить турель в ноль / включить обратно
 *   R1 — маховик вкл/выкл
 *   R2 — интейк | L2 — трансфер | L1 — реверс
 *   R3 — flower открыть/закрыть
 *
 * Удержание угла: пока правый стик отпущен, робот держит свой курс
 * (не разворачивается от толчков и при езде). Повернули стиком — держит новый курс.
 */
@Configurable
public abstract class SoloDrive extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.2;

    /* ================= HEADING HOLD ================= */
    // Меняются через Panels (@Configurable)

    public static boolean HEADING_HOLD = true;
    /** Мощность поворота на 1 радиан ошибки. Робот "виляет" — уменьшить, держит вяло — увеличить. */
    public static double HEADING_KP = 1.5;
    /** Гасит раскачку: мощность на 1 рад/с угловой скорости. */
    public static double HEADING_KD = 0.1;
    /** Ошибка меньше этого (градусы) не исправляется — чтобы робот не дрожал. */
    public static double HEADING_TOLERANCE_DEG = 1.0;
    /** Правый стик меньше этого считается отпущенным. */
    public static double TURN_DEADBAND = 0.05;
    /** Сколько ждать после отпускания стика, прежде чем запомнить курс (мс): робот успевает остановить поворот. */
    public static long HEADING_SETTLE_MS = 250;

    /* ================= ALLIANCE ================= */

    protected abstract boolean isBlue();

    /** Позиция при запуске без автонома. */
    protected abstract Pose startPose();

    /** Позиция сброса (L3). */
    protected abstract Pose resetPose();

    /**
     * Поворот осей field-centric под сторону, где стоит водитель (радианы).
     * 0 — «вперёд» на стике это +X поля; PI — это -X поля.
     */
    protected abstract double driverHeadingOffset();

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private Follower follower;

    private boolean autoAim = true; // false — турель стоит в нуле

    private double targetHeading = 0;     // курс, который держим, рад
    private boolean holdingHeading = false;
    private long lastTurnTimeMs = 0;

    private boolean robotCentric = false; // false — field-centric, true — robot-centric

    /** Сколько мс повторять setPose после L3, пока Pinpoint не примет новую позицию. */
    public static long RESET_REPEAT_MS = 100;
    private long resetUntilMs = 0;

    private final Button resetButton    = new Button(() -> gamepad1.left_stick_button);
    private final Button driveModeButton = new Button(() -> gamepad1.dpad_up);
    private final Button autoAimButton  = new Button(() -> gamepad1.triangle);
    private final Button shooterButton  = new Button(() -> gamepad1.right_bumper);
    private final Button flowerButton   = new Button(() -> gamepad1.right_stick_button);

    /* ================= LIFECYCLE ================= */

    @Override
    public void init() {
        // Bulk caching: все чтения с хаба за цикл идут одним запросом.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // После автонома продолжаем с его конечной позиции, иначе — со startPose.
        boolean afterAuto = PoseStorage.autoEndPose != null;
        follower.setStartingPose(afterAuto ? PoseStorage.autoEndPose : startPose());
        PoseStorage.autoEndPose = null; // чтобы следующий запуск без автонома взял startPose

        // После автонома энкодер турели уже отсчитан от его нуля — не трогаем.
        // Без автонома турель должна стоять в нуле (назад) при init.
        if (!afterAuto) {
            shooter.resetTurretEncoder();
        }

        telemetry.addLine(isBlue() ? "BLUE TeleOp Ready" : "RED TeleOp Ready");
        telemetry.addLine(afterAuto ? "Pose: from autonomous" : "Pose: startPose");
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
    }

    @Override
    public void start() {
        // Pedro 2: включить ручную езду (true — BRAKE при отпущенных стиках)
        follower.startTeleopDrive(true);

        shooter.setFlywheelEnabled(true);
        autoAim = true;
    }

    @Override
    public void loop() {
        drive();
        aim();
        controlShooter();
        controlIntake();

        shooter.update();
        intake.update();

        showTelemetry();
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }

        if (intake != null) {
            intake.stop();
            intake.update();
        }
    }

    /* ================= CONTROLS ================= */

    private void drive() {
        // Pinpoint применяет новую позицию не сразу: первое чтение после setPose
        // может вернуть старую. Поэтому повторяем setPose RESET_REPEAT_MS мс.
        long now = System.currentTimeMillis();
        if (resetButton.wasPressed()) {
            resetUntilMs = now + RESET_REPEAT_MS;
        }
        if (now < resetUntilMs) {
            follower.setPose(resetPose());
            holdingHeading = false; // курс сменился — запомнить заново
            lastTurnTimeMs = now;   // не запоминать курс, пока сброс не применился
        }

        // D-pad вверх — переключение field-centric / robot-centric.
        if (driveModeButton.wasPressed()) {
            robotCentric = !robotCentric;
        }

        // Field-centric: оси повёрнуты под сторону водителя.
        // Robot-centric: «вперёд» на стике — вперёд робота, смещение не нужно.
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower(-gamepad1.right_stick_x),
                robotCentric,
                robotCentric ? 0 : driverHeadingOffset()
        );
        follower.update();
    }

    /**
     * Мощность поворота: со стика, а если стик отпущен — удержание курса.
     * Плюс = против часовой (курс растёт).
     */
    private double turnPower(double stickTurn) {
        long now = System.currentTimeMillis();
        double heading = follower.getPose().getHeading();

        if (!HEADING_HOLD || Math.abs(stickTurn) > TURN_DEADBAND) {
            lastTurnTimeMs = now;
            holdingHeading = false;
            return stickTurn;
        }

        // Стик только что отпустили: даём роботу остановиться и запоминаем курс.
        if (!holdingHeading) {
            if (now - lastTurnTimeMs < HEADING_SETTLE_MS) {
                return 0;
            }
            targetHeading = heading;
            holdingHeading = true;
        }

        double error = normalizeRadians(targetHeading - heading);
        if (Math.abs(error) < Math.toRadians(HEADING_TOLERANCE_DEG)) {
            return 0;
        }

        double omega = follower.getAngularVelocity();
        double power = HEADING_KP * error - HEADING_KD * omega;
        return Math.max(-1, Math.min(1, power));
    }

    /** Автоприцел турели по позиции и скорости робота. △ — выкл (турель в ноль) / вкл. */
    private void aim() {
        if (autoAimButton.wasPressed()) {
            autoAim = !autoAim;
        }

        if (autoAim) {
            Pose pose = follower.getPose();
            Vector velocity = follower.getVelocity(); // в координатах поля, дюймы/с
            shooter.alignTurret(
                    pose.getX(), pose.getY(), pose.getHeading(),
                    velocity.getXComponent(), velocity.getYComponent(), follower.getAngularVelocity(),
                    isBlue()
            );
        } else {
            shooter.centerTurret();
        }
    }

    private void controlShooter() {
        if (shooterButton.wasPressed()) {
            shooter.setFlywheelEnabled(!shooter.isFlywheelEnabled());
        }
    }

    private void controlIntake() {
        // Приоритет: L2 (трансфер) > R2 (интейк) > L1 (реверс).
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            intake.transfer();
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.intake();
        } else if (gamepad1.left_bumper) {
            intake.reverse();
        } else {
            intake.stop();
        }

        if (flowerButton.wasPressed()) {
            if (intake.getFlowerState() == FlowerState.OPEN) {
                intake.flowerC();
            } else {
                intake.flowerO();
            }
        }
    }

    /* ================= TELEMETRY ================= */

    private void showTelemetry() {
        telemetry.addData("Drive mode", robotCentric ? "ROBOT-centric" : "FIELD-centric");
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Heading hold", holdingHeading
                ? String.format("%.1f°", Math.toDegrees(targetHeading)) : "off");
        telemetry.addData("Shooter On", shooter.isFlywheelEnabled());

        if (shooter.isTurretFault()) {
            telemetry.addLine("!!! ТУРЕЛЬ: АВАРИЯ — ушла за ход, мотор выключен. Проверьте turretReversed / энкодер");
        }
        telemetry.addData("Auto aim", autoAim);
        telemetry.addData("Turret target / actual", "%.1f° / %.1f°",
                shooter.getPos(), shooter.getTurretDeg());
        telemetry.addData("Distance to goal", shooter.getDistance());

        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
        telemetry.addData("Flywheel ticks/s", shooter.getVel());
        telemetry.addData("Ready to shoot", shooter.isReady());

        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Flower", intake.getFlowerState());
        telemetry.update();
    }

    /* ================= HELPERS ================= */

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /** Кнопка, которая срабатывает один раз за нажатие. */
    private static class Button {
        private final BooleanSupplier input;
        private boolean last = false;

        Button(BooleanSupplier input) {
            this.input = input;
        }

        /** Вызывать ровно один раз за цикл. */
        boolean wasPressed() {
            boolean current = input.getAsBoolean();
            boolean pressed = current && !last;
            last = current;
            return pressed;
        }
    }
}
