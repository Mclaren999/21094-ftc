//package org.firstinspires.ftc.teamcode.OpModes.Auto;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
//import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Configurable
//@Autonomous(name = "Main Auto", group = "Autonomous")
//public class MainAuto extends LinearOpMode {
//
//    // Альянс (для телеметрии)
//    public static boolean BLUE = true;
//
//
//    // Времена (мс)
//    public static long SPIN_UP_MS = 2000;
//    public static long SHOOT_MS = 1000;
//
//    private Follower follower;
//    private ShooterSubsystem shooter;
//    private IntakeSubsystem intake;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        follower = Constants.createFollower(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);
//
//        follower.setStartingPose(AutoPaths.START);
//
//        intake.off();
//        intake.update();
//        shooter.centerTurret();
//
//        telemetry.addLine("Auto Initialized");
//        telemetry.addData("Alliance", BLUE ? "BLUE" : "RED");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // ---- Разгон маховика ----
//        shooter.setFlywheelEnabled(true);
//        waitMs(SPIN_UP_MS);
//
//        // ---- Выстрел 1 ----
//        shoot();
//
//        // ---- Сбор ----
//        intake.intake();
//        followPath(AutoPaths.path11(follower), true, 1500);
//        waitMs(300);
//
//        // ---- Выстрел 2 ----
//        followPath(AutoPaths.path22(follower), true, 2500);
//        waitMs(1000);
//        shoot();
//        intake.off();
//
//        // ---- Парковка ----
//        followPath(AutoPaths.path33(follower), false, 1500);
//        followPath(AutoPaths.path55(follower), false, 1500);
//
//        // ---- Конец: всё выключить, позицию отдать в TeleOp ----
//        intake.off();
//        intake.update();
//        shooter.stop();
//        PoseStorage.autoEndPose = follower.getPose();
//    }
//
//    /* ================= ACTIONS ================= */
//
//    /** Подаёт мячи в маховик в течение SHOOT_MS. */
//    private void shoot() {
//        intake.transfer();
//        waitMs(SHOOT_MS);
//    }
//
//    /**
//     * Едет по пути и ждёт его конца.
//     *
//     * @param holdEnd   удерживать ли конечную точку после приезда
//     * @param timeoutMs сколько ждать максимум, чтобы не зависнуть, если робот застрял
//     */
//    private void followPath(PathChain path, boolean holdEnd, long timeoutMs) {
//        follower.followPath(path, holdEnd);
//        long deadline = System.currentTimeMillis() + timeoutMs;
//
//        while (opModeIsActive() && follower.isBusy() && System.currentTimeMillis() < deadline) {
//            updateAll();
//        }
//    }
//
//    /** Ждёт, не переставая обновлять системы. */
//    private void waitMs(long milliseconds) {
//        long deadline = System.currentTimeMillis() + milliseconds;
//
//        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
//            updateAll();
//            idle();
//        }
//    }
//
//    /* ================= UPDATE ================= */
//
//    /** Обновление всех систем — каждый цикл. */
//    private void updateAll() {
//        follower.update();
//
//        // Турель всё время наводится сама (с учётом развёрнутой установки).
//        Pose pose = follower.getPose();
//        Vector velocity = follower.getVelocity();
//        shooter.alignTurret(
//                pose.getX(), pose.getY(), pose.getHeading(),
//                velocity.getXComponent(), velocity.getYComponent(), follower.getAngularVelocity(),
//                BLUE
//        );
//        shooter.update();
//        intake.update();
//
//        // Сохраняем каждый цикл: позиция останется, даже если авто прервут.
//        PoseStorage.autoEndPose = follower.getPose();
//
//        telemetry.addData("Pose", follower.getPose());
//        telemetry.addData("Turret Position", shooter.getPos());
//        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
//        telemetry.addData("Flywheel ticks/s", shooter.getVel());
//        telemetry.addData("Intake", intake.getState());
//        telemetry.update();
//    }
//}
