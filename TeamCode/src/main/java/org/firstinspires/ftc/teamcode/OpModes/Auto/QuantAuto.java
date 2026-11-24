package org.firstinspires.ftc.teamcode.OpModes.Auto;
//bismillah
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Автоном Quant. Турель НЕ наводится сама — стоит в фиксированных позициях.
 *
 * 1. Турель в ноль, маховик разгоняется, ждём 5 с.
 * 2. Выстрел. Опускаем flower, включаем интейк.
 * 3. go to flower 1 -> go to flower 2 -> intake flower, там ждём 5 с (собираем).
 * 4. go to shoot, турель на TURRET_SHOOT_DEG.
 * 5. shoot: доезжаем, ждём 1 с, стреляем 1 с.
 * 6. Парковка.
 *
 * Скорость маховика для каждого выстрела — FIRST_SHOOT_VELOCITY и SHOOT_VELOCITY.
 */
@Configurable
@Autonomous(name = "Quant Auto", group = "Autonomous")
public class QuantAuto extends LinearOpMode {

    /* ================= ТУРЕЛЬ (градусы от нуля, + = влево) ================= */
    // Было для сервы: 0.5 и 0.225. По старой формуле (0.4933 - pos) * 409.09 это ≈ 0° и ≈ 110°.

    public static double TURRET_START_DEG = 0;
    public static double TURRET_SHOOT_DEG = 110;

    /* ================= СКОРОСТЬ МАХОВИКА (тики/с) ================= */
    // У каждого выстрела своя. 0 — брать скорость по умолчанию (fIntercept в ShooterSubsystem).

    public static double FIRST_SHOOT_VELOCITY = 2070; // выстрел 1 — со старта
    public static double SHOOT_VELOCITY = 2070;       // выстрел 2 — на точке shoot

    /* ================= ВРЕМЕНА (мс) ================= */

    public static long START_WAIT_MS = 5000;       // ожидание на старте (маховик разгоняется)
    public static long FIRST_SHOOT_MS = 1000;      // первая серия
    public static long INTAKE_FLOWER_WAIT_MS = 5000; // сбор на точке intake flower
    public static long SHOOT_SETTLE_MS = 1000;     // пауза на точке shoot перед выстрелом
    public static long SHOOT_MS = 1000;            // вторая серия
    public static long PATH_TIMEOUT_MS = 4000;     // страховка, если робот застрял на пути

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    private double turretDeg = TURRET_START_DEG;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.resetTurretEncoder(); // турель при init должна смотреть назад (ноль)
        intake = new IntakeSubsystem(hardwareMap);

        follower.setStartingPose(QuantPaths.START);

        intake.off();
        intake.flowerC();
        intake.update();

        telemetry.addLine("Quant Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- 1. Турель 0.5, разгон маховика, ждём 5 с ----
        turretDeg = TURRET_START_DEG;
        shooter.setFixedVelocity(FIRST_SHOOT_VELOCITY);
        shooter.setFlywheelEnabled(true);
        waitMs(START_WAIT_MS);

        // ---- 2. Выстрел, затем опустить flower и включить интейк ----
        shoot(FIRST_SHOOT_MS);
        intake.flowerO();
        intake.intake();

        // ---- 3. К flower и сбор ----
        follow(QuantPaths.gotoFlower1(follower));
        follow(QuantPaths.gotoFlower2(follower));
        follow(QuantPaths.intakeFlower(follower));
        waitMs(INTAKE_FLOWER_WAIT_MS);

        // ---- 4. go to shoot, турель 0.225 ----
        // Скорость второго выстрела ставим заранее: маховик успеет разогнаться/сбросить, пока едем.
        turretDeg = TURRET_SHOOT_DEG;
        shooter.setFixedVelocity(SHOOT_VELOCITY);
        follow(QuantPaths.gotoShoot(follower));

        // ---- 5. shoot: ждём 1 с, стреляем 1 с ----
        follow(QuantPaths.shoot(follower));
        waitMs(SHOOT_SETTLE_MS);
        shoot(SHOOT_MS);

        // ---- 6. Всё выключить и на парковку ----
        intake.off();
        shooter.stop();
        follow(QuantPaths.park(follower));

        PoseStorage.autoEndPose = follower.getPose();
    }

    /* ================= ACTIONS ================= */

    /** Подаёт мячи в маховик заданное время, потом возвращает интейк на сбор. */
    private void shoot(long shootMs) {
        intake.transfer();
        waitMs(shootMs);
        intake.intake();
    }

    /** Едет по пути и ждёт его конца; в конце удерживает точку. */
    private void follow(PathChain path) {
        follower.followPath(path, true);
        long deadline = System.currentTimeMillis() + PATH_TIMEOUT_MS;

        do {
            updateAll();
        } while (opModeIsActive()
                && follower.isBusy()
                && System.currentTimeMillis() < deadline);
    }

    /** Ждёт, не переставая обновлять системы. */
    private void waitMs(long milliseconds) {
        long deadline = System.currentTimeMillis() + milliseconds;

        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            updateAll();
        }
    }

    /* ================= UPDATE ================= */

    private void updateAll() {
        follower.update();

        shooter.setTurretAngle(turretDeg);
        shooter.update();
        intake.update();

        // Сохраняем каждый цикл: позиция останется для TeleOp, даже если авто прервут.
        PoseStorage.autoEndPose = follower.getPose();

        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Turret target / actual", "%.1f° / %.1f°", turretDeg, shooter.getTurretDeg());
        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
        telemetry.addData("Flywheel ticks/s", shooter.getVel());
        telemetry.addData("Ready to shoot", shooter.isReady());
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Flower", intake.getFlowerState());
        telemetry.update();
    }
}
