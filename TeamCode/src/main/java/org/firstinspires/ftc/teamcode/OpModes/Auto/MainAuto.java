package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Main Auto", group = "Autonomous")
public class MainAuto extends LinearOpMode {

    // Альянс (для телеметрии)
    public static boolean BLUE = true;


    // Времена (мс)
    public static long SPIN_UP_MS = 2000;
    public static long SHOOT_MS = 1000;

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        follower.setPose(AutoPaths.START);

        intake.off();
        intake.update();
        shooter.centerTurret();

        if (!Constants.isTuned()) {
            // Pedro 3: без результата Foresight Tuner робот по путям не поедет.
            telemetry.addLine("Foresight НЕ НАСТРОЕН: запустите Foresight Tuner");
            telemetry.addLine("и вставьте результат в Constants.foresightConfig");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Auto Initialized");
        telemetry.addData("Alliance", BLUE ? "BLUE" : "RED");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- Разгон маховика ----
        shooter.setFlywheelEnabled(true);
        waitMs(SPIN_UP_MS);

        // ---- Выстрел 1 ----
        shoot();

        // ---- Сбор ----
        intake.intake();
        followPath(AutoPaths.path11(), true, 1500);
        waitMs(300);

        // ---- Выстрел 2 ----
        followPath(AutoPaths.path22(), true, 2500);
        waitMs(1000);
        shoot();
        intake.off();

        // ---- Парковка ----
        followPath(AutoPaths.path33(), false, 1500);
        followPath(AutoPaths.path55(), false, 1500);

        // ---- Конец: всё выключить, позицию отдать в TeleOp ----
        intake.off();
        intake.update();
        shooter.stop();
        PoseStorage.autoEndPose = follower.pose();
    }

    /* ================= ACTIONS ================= */

    /** Подаёт мячи в маховик в течение SHOOT_MS. */
    private void shoot() {
        intake.transfer();
        waitMs(SHOOT_MS);
    }

    /**
     * Едет по пути и ждёт его конца.
     *
     * @param holdEnd   удерживать ли конечную точку после приезда
     * @param timeoutMs сколько ждать максимум, чтобы не зависнуть, если робот застрял
     */
    private void followPath(Path path, boolean holdEnd, long timeoutMs) {
        follower.holdEnd.set(holdEnd);
        follower.follow(path);
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (opModeIsActive() && follower.isBusy() && System.currentTimeMillis() < deadline) {
            updateAll();
        }
    }

    /** Ждёт, не переставая обновлять системы. */
    private void waitMs(long milliseconds) {
        long deadline = System.currentTimeMillis() + milliseconds;

        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            updateAll();
            idle();
        }
    }

    /* ================= UPDATE ================= */

    /** Обновление всех систем — каждый цикл. */
    private void updateAll() {
        follower.update();

        // Турель всё время наводится сама (с учётом развёрнутой установки).
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        shooter.alignTurret(
                pose.x(), pose.y(), pose.heading(),
                velocity.vx, velocity.vy, velocity.omega,
                BLUE
        );
        shooter.update();
        intake.update();

        // Сохраняем каждый цикл: позиция останется, даже если авто прервут.
        PoseStorage.autoEndPose = follower.pose();

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Turret Position", shooter.getPos());
        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
        telemetry.addData("Flywheel ticks/s", shooter.getVel());
        telemetry.addData("Intake", intake.getState());
        telemetry.update();
    }
}
