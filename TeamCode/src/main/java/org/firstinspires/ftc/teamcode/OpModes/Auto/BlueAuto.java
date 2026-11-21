package org.firstinspires.ftc.teamcode.OpModes.Auto;
//bismillah
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
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

/**
 * Автоном синего альянса: 5 серий выстрелов и парковка.
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

    private static final PoseFactory P = PoseFactory.degrees();

    private static final Pose START = P.of(29.8336, 6.6034, 180);
    private static final Pose INTAKE = P.of(6.1015, 6.7843, 180);

    private static final Pose GO_TO_SHOOT_START = P.of(6.1015, 6.7843, -100);
    private static final Pose GO_TO_SHOOT = P.of(30.3758, 118.2687, -100);

    private static final Pose INTAKE_FLOWER_SHOOT_CONTROL = P.of(46.9841, 97.8978, 0);
    private static final Pose INTAKE_FLOWER_SHOOT = P.of(47.0406, 125.4825, 90);

    private static final Pose GO_TO_FLOWER = P.of(21.5909, 46, 180);
    private static final Pose INTAKE_FLOWER = P.of(13.206, 46, 180);

    private static final Pose SHOOT_2 = P.of(30.736, 119.0509, -90);

    private static final Pose INTAKE_SHOOT_CONTROL = P.of(85.2239, 99.7828, 0);
    private static final Pose INTAKE_SHOOT = P.of(56.7606, 20.9021, -109.8415);

    private static final Pose PARK = P.of(8.2221, 91.6222, -55.5364);

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

        follower.setPose(START);

        intake.off();
        intake.flowerC();
        intake.update();

        if (!Constants.isTuned()) {
            // Pedro 3: без результата Foresight Tuner робот по путям не поедет.
            telemetry.addLine("Foresight НЕ НАСТРОЕН: запустите Foresight Tuner");
            telemetry.addLine("и вставьте результат в Constants.foresightConfig");
            telemetry.update();
            waitForStart();
            return;
        }

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
        follow(Paths.path(goToFlowerPath(), intakeFlowerPath()));
        waitMs(INTAKE_FLOWER_MS);
        intake.off();

        // ---- 6. Точка shoot: четвёртая серия ----
        follow(shoot2Path());
        aimAndShoot(SHOOT_MS);

        // ---- 7. С интейком на точку intake + shoot: пятая серия ----
        intake.intake();
        follow(intakeShootPath());
        aimAndShoot(SHOOT_MS);

        // ---- 8. Всё выключить и на парковку ----
        intake.off();
        shooter.stop();
        follow(parkPath());

        PoseStorage.autoEndPose = follower.pose();
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
    private void follow(Path path) {
        follower.follow(path);
        long deadline = System.currentTimeMillis() + PATH_TIMEOUT_MS;

        do {
            updateAll();
        } while (opModeIsActive()
                && (follower.following() || follower.isBusy())
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

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        shooter.alignTurret(
                pose.x(), pose.y(), pose.heading(),
                velocity.vx, velocity.vy, velocity.omega,
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

    private static Path intakePath() {
        return Paths.line(START, INTAKE).linear(START, INTAKE);
    }

    private static Path goToShootPath() {
        return Paths.line(GO_TO_SHOOT_START, GO_TO_SHOOT).linear(GO_TO_SHOOT_START, GO_TO_SHOOT);
    }

    private static Path intakeFlowerShootPath() {
        return Paths.curve(GO_TO_SHOOT, INTAKE_FLOWER_SHOOT_CONTROL, INTAKE_FLOWER_SHOOT).tangent();
    }

    private static Path goToFlowerPath() {
        return Paths.line(INTAKE_FLOWER_SHOOT, GO_TO_FLOWER).linear(INTAKE_FLOWER_SHOOT, GO_TO_FLOWER);
    }

    private static Path intakeFlowerPath() {
        return Paths.line(GO_TO_FLOWER, INTAKE_FLOWER).linear(GO_TO_FLOWER, INTAKE_FLOWER);
    }

    private static Path shoot2Path() {
        // Курс: от 180° в конце сбора к -90° на точке выстрела.
        return Paths.line(INTAKE_FLOWER, SHOOT_2).linear(INTAKE_FLOWER, SHOOT_2);
    }

    private static Path intakeShootPath() {
        return Paths.curve(SHOOT_2, INTAKE_SHOOT_CONTROL, INTAKE_SHOOT).tangent();
    }

    private static Path parkPath() {
        return Paths.line(INTAKE_SHOOT, PARK).reverseTangent();
    }
}
