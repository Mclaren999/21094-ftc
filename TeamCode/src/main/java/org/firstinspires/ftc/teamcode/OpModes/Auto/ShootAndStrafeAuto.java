package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.BlueSoloDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Простой автоном: турель сама наводится, маховик 2350 тиков/с, серия выстрелов,
 * потом робот едет стрейфом вправо 1 секунду.
 * По путям не ездит — работает даже без настроенного Foresight.
 */
@Configurable
@Autonomous(name = "Shoot + Strafe Right", group = "Autonomous")
public class ShootAndStrafeAuto extends LinearOpMode {

    public static boolean IS_BLUE = true;

    // Старт — та же позиция, что в TeleOp (BlueSoloDrive.startPose): меняется там, меняется и здесь.

    public static double FLYWHEEL_VELOCITY = 2350; // тики/с

    public static long WAIT_BEFORE_SHOOT_MS = 10000; // пауза перед выстрелом (маховик крутится, турель наводится)
    public static long SPIN_UP_TIMEOUT_MS = 3000;  // максимум ждать разгона маховика
    public static long AIM_SETTLE_MS = 500;        // пауза, чтобы турель навелась
    public static long SHOOT_MS = 2000;            // сколько подавать мячи

    public static double STRAFE_POWER = 0.5;       // мощность стрейфа вправо
    public static long STRAFE_MS = 1000;           // время стрейфа

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

        Pose start = BlueSoloDrive.startPose;
        follower.setPose(start);

        intake.off();
        intake.flowerC();
        intake.update();
        shooter.centerTurret();

        telemetry.addLine("Shoot + Strafe Right готов");
        telemetry.addData("Старт (как в TeleOp)", start);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- 1. Разгон маховика до 2350, турель наводится ----
        shooter.setFixedVelocity(FLYWHEEL_VELOCITY);
        shooter.setFlywheelEnabled(true);

        // Ждём 10 секунд: маховик за это время разгоняется, турель наводится.
        waitMs(WAIT_BEFORE_SHOOT_MS, 0);

        long deadline = System.currentTimeMillis() + SPIN_UP_TIMEOUT_MS;
        while (opModeIsActive() && !shooter.isReady() && System.currentTimeMillis() < deadline) {
            updateAll(0);
        }
        waitMs(AIM_SETTLE_MS, 0);

        // ---- 2. Выстрел ----
        intake.transfer();
        waitMs(SHOOT_MS, 0);

        // ---- 3. Всё выключить ----
        intake.off();
        shooter.stop();

        // ---- 4. Стрейф вправо 1 секунду ----
        waitMs(STRAFE_MS, -STRAFE_POWER);
        follower.manual(0, 0, 0);
        follower.update();

        PoseStorage.autoEndPose = follower.pose();
    }

    /** Ждёт, обновляя всё; strafePower — стрейф (+ влево, - вправо, 0 — стоим). */
    private void waitMs(long milliseconds, double strafePower) {
        long deadline = System.currentTimeMillis() + milliseconds;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            updateAll(strafePower);
        }
    }

    private void updateAll(double strafePower) {
        // Robot-centric: только боковой ход относительно робота, без езды вперёд и поворота.
        // Знак как в TeleOp: там strafe = -left_stick_x, т.е. стик вправо даёт минус.
        follower.manual(0, strafePower, 0);
        follower.update();

        Pose pose = follower.pose();
        if (shooter.isFlywheelEnabled()) {
            shooter.alignTurret(pose.x(), pose.y(), pose.heading(), IS_BLUE);
        }
        shooter.update();
        intake.update();

        PoseStorage.autoEndPose = pose;

        telemetry.addData("Pose", pose);
        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
        telemetry.addData("Flywheel ticks/s", shooter.getVel());
        telemetry.addData("Ready to shoot", shooter.isReady());
        telemetry.addData("Turret Angle", shooter.getAngle());
        telemetry.addData("Intake", intake.getState());
        telemetry.update();
    }
}
