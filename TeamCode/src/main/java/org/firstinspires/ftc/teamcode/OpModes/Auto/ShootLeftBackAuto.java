package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Старт (30.4, 180), курс 0°. Турель всё время наводится сама.
 * Ждёт 3 с (маховик разгоняется) -> стреляет -> 5 с едет влево -> 2 с назад.
 * Направления — относительно робота. По путям не ездит, Foresight не нужен.
 */
@Configurable
@Autonomous(name = "Shoot + Left + Back", group = "Autonomous")
public class ShootLeftBackAuto extends LinearOpMode {

    public static boolean IS_BLUE = true;

    public static double START_X = 30.4;
    public static double START_Y = 180; // поле Pedro 0..144 — проверьте эту координату
    public static double START_HEADING_DEG = 0;

    public static double FLYWHEEL_VELOCITY = 2200; // тики/с

    public static long WAIT_BEFORE_SHOOT_MS = 3000; // маховик крутится, турель наводится
    public static long SHOOT_MS = 2000;             // сколько подавать мячи

    public static double DRIVE_POWER = 0.5;
    public static long LEFT_MS = 5000;              // влево
    public static long BACK_MS = 2000;              // назад

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

        Pose start = PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG);
        follower.setPose(start);

        intake.off();
        intake.flowerC();
        intake.update();
        shooter.centerTurret();

        telemetry.addLine("Shoot + Left + Back готов");
        telemetry.addData("Старт", start);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- 1. Маховик 2350, турель наводится, ждём 3 секунды ----
        shooter.setFixedVelocity(FLYWHEEL_VELOCITY);
        shooter.setFlywheelEnabled(true);
        drive(WAIT_BEFORE_SHOOT_MS, 0, 0);

        // ---- 2. Выстрел ----
        intake.transfer();
        drive(SHOOT_MS, 0, 0);

        // ---- 3. Всё выключить ----
        intake.off();
        shooter.stop();

        // ---- 4. Влево 5 секунд ----
        drive(LEFT_MS, 0, DRIVE_POWER);

        // ---- 5. Назад 2 секунды ----
        drive(BACK_MS, -DRIVE_POWER, 0);

        follower.manual(0, 0, 0);
        follower.update();
        PoseStorage.autoEndPose = follower.pose();
    }

    /**
     * Едет заданное время относительно робота, обновляя всё остальное.
     * forward: + вперёд, - назад. strafe: + влево, - вправо (знаки как в TeleOp).
     */
    private void drive(long milliseconds, double forward, double strafe) {
        long deadline = System.currentTimeMillis() + milliseconds;
        while (opModeIsActive() && System.currentTimeMillis() < deadline) {
            follower.manual(forward, strafe, 0);
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
}
