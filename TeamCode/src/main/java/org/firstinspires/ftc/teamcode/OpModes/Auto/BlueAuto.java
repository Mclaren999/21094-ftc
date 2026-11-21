package org.firstinspires.ftc.teamcode.OpModes.Auto;
//bismillah
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

/**
 * Автоном синего альянса: 4 серии выстрелов и парковка (path7).
 * Турель всё время наводится сама (alignTurret), маховик держит скорость по дистанции.
 */
@Configurable
@Autonomous(name = "Autooon ArlKarl", group = "Autonomous")
public class BlueAuto extends LinearOpMode {

    private static final boolean IS_BLUE = true;

    /* ================= ВРЕМЕНА (мс) ================= */

    public static long SPIN_UP_MS = 1000;        // разгон маховика на старте
    public static long AIM_SETTLE_MS = 500;      // пауза перед каждым выстрелом, чтобы турель навелась
    public static long SHOOT_MS = 800;           // обычная серия
    public static long SHOOT_LONG_MS = 1300;     // серия после сбора у flower
    public static long INTAKE_AT_SHOOT_MS = 1000; // сбор на точке intake flower + shoot
    public static long INTAKE_FLOWER_MS = 1500;  // сбор на точке intake flower
    public static long PATH_TIMEOUT_MS = 4000;   // страховка, если робот застрял на пути

    /* ================= ТОЧКИ ================= */

    // Старт = точка первого выстрела (shoot)
    private static final Pose START = Poses.deg(30.1881, 7.4679, 180);
    private static final Pose INTAKE = Poses.deg(17.4578, 7.6495, 180);

    private static final Pose GO_TO_SHOOT_START = Poses.deg(17.4578, 7.6495, -100);
    private static final Pose GO_TO_SHOOT = Poses.deg(17.3968, 98.1264, -100);

    private static final Pose INTAKE_FLOWER_SHOOT_CONTROL = Poses.deg(37.2806, 73.1173, 0);
    private static final Pose INTAKE_FLOWER_SHOOT = Poses.deg(37.3023, 105.1767, 89.9612);

    private static final Pose GO_TO_FLOWER_START = Poses.deg(37.3023, 105.1767, 90);
    private static final Pose GO_TO_FLOWER = Poses.deg(23.6823, 36.3344, 180);
    private static final Pose INTAKE_FLOWER = Poses.deg(13.1808, 36.1463, 180);

    private static final Pose SHOOT_2 = Poses.deg(18.1415, 98.965, -90);

    private static final Pose PARK = Poses.deg(7.2209, 97.9804, 5.152);

    /* ================= SUBSYSTEMS ================= */

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

        follower.setStartingPose(START);

        intake.off();
        intake.flowerC();
        intake.update();

        telemetry.addLine("Blue Auto Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- 1. Старт: автоприцел, разгон маховика, первая серия ----
        shooter.setFlywheelEnabled(true);
        waitMs(SPIN_UP_MS);
        shoot(SHOOT_MS);

        // ---- 2. Сбор на точке intake ----
        intake.intake();
        follow(intakePath());

        // ---- 3. Точка go to shoot: вторая серия ----
        follow(goToShootPath());
        aimAndShoot(SHOOT_MS);

        // ---- 4. Открыть flower, с интейком на точку intake flower + shoot ----
        intake.flowerO();
        intake.intake();
        follow(intakeFlowerShootPath());
        waitMs(INTAKE_AT_SHOOT_MS); // собираем стоя; турель за это время успевает навестись
        shoot(SHOOT_LONG_MS);

        // ---- 5. Go to flower и сразу intake flower: сбор, затем интейк выключить ----
        intake.intake();
        follow(goToAndIntakeFlowerPath());
        waitMs(INTAKE_FLOWER_MS);
        intake.off();

        // ---- 6. Точка shoot: четвёртая серия ----
        follow(shoot2Path());
        aimAndShoot(SHOOT_MS);

        // ---- 7. Всё выключить и на парковку ----
        intake.off();
        shooter.stop();
        follow(parkPath());

        PoseStorage.autoEndPose = follower.getPose();
    }

    /* ================= ACTIONS ================= */

    /** Ждёт, пока турель наведётся, и стреляет. */
    private void aimAndShoot(long shootMs) {
        waitMs(AIM_SETTLE_MS);
        shoot(shootMs);
    }

    /** Подаёт мячи в маховик заданное время. */
    private void shoot(long shootMs) {
        intake.transfer();
        waitMs(shootMs);
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

    /** Каждый цикл: езда, автоприцел, маховик, интейк. */
    private void updateAll() {
        follower.update();

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity(); // в координатах поля, дюймы/с
        shooter.alignTurret(
                pose.getX(), pose.getY(), pose.getHeading(),
                velocity.getXComponent(), velocity.getYComponent(), follower.getAngularVelocity(),
                IS_BLUE
        );

        shooter.update();
        intake.update();

        // Сохраняем каждый цикл: позиция останется для TeleOp, даже если авто прервут.
        PoseStorage.autoEndPose = pose;

        telemetry.addData("Pose", pose);
        telemetry.addData("Turret Angle", shooter.getAngle());
        telemetry.addData("Distance to goal", shooter.getDistance());
        telemetry.addData("Target ticks/s", ShooterSubsystem.vel);
        telemetry.addData("Flywheel ticks/s", shooter.getVel());
        telemetry.addData("Ready to shoot", shooter.isReady());
        telemetry.addData("Intake", intake.getState());
        telemetry.update();
    }

    /* ================= PATHS ================= */
    // Pedro 2: пути строятся через follower.pathBuilder(), поэтому это методы экземпляра.

    /** Прямая с плавным поворотом от курса start к курсу end. */
    private PathChain line(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private PathChain intakePath() {
        return line(START, INTAKE);
    }

    private PathChain goToShootPath() {
        return line(GO_TO_SHOOT_START, GO_TO_SHOOT);
    }

    private PathChain intakeFlowerShootPath() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(GO_TO_SHOOT, INTAKE_FLOWER_SHOOT_CONTROL, INTAKE_FLOWER_SHOOT))
                .setTangentHeadingInterpolation()
                .build();
    }

    /** Go to flower и сразу intake flower — одной цепочкой, без остановки между ними. */
    private PathChain goToAndIntakeFlowerPath() {
        return follower.pathBuilder()
                .addPath(new BezierLine(GO_TO_FLOWER_START, GO_TO_FLOWER))
                .setLinearHeadingInterpolation(GO_TO_FLOWER_START.getHeading(), GO_TO_FLOWER.getHeading())
                .addPath(new BezierLine(GO_TO_FLOWER, INTAKE_FLOWER))
                .setLinearHeadingInterpolation(GO_TO_FLOWER.getHeading(), INTAKE_FLOWER.getHeading())
                .build();
    }

    private PathChain shoot2Path() {
        // Курс: от 180° в конце сбора к -90° на точке выстрела.
        return line(INTAKE_FLOWER, SHOOT_2);
    }

    private PathChain parkPath() {
        // Задом вперёд по касательной.
        return follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_2, PARK))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
}
