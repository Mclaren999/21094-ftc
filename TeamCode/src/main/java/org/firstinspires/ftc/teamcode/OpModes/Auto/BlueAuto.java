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

    private static final PoseFactory P = PoseFactory.degrees();

    // Старт = точка первого выстрела (shoot)
    private static final Pose START = P.of(30.1881, 7.4679, 180);
    private static final Pose INTAKE = P.of(10.6075, 7.8802, 180);

    private static final Pose GO_TO_SHOOT_START = P.of(10.6075, 7.8802, -100);
    private static final Pose GO_TO_SHOOT = P.of(19.2442, 111.222, -100);

    private static final Pose INTAKE_FLOWER_SHOOT_CONTROL = P.of(37.4884, 93.6423, 0);
    private static final Pose INTAKE_FLOWER_SHOOT = P.of(37.5053, 117.1884, 89.9589);

    private static final Pose GO_TO_FLOWER_START = P.of(37.5053, 117.1884, 90);
    private static final Pose GO_TO_FLOWER = P.of(23.6823, 36.3344, 180);
    private static final Pose INTAKE_FLOWER = P.of(13.1808, 36.1463, 180);

    private static final Pose SHOOT_2 = P.of(18.392, 111.3234, -90);

    private static final Pose PARK = P.of(10.3385, 111.5488, -1.603);

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

        // ---- 7. Всё выключить и на парковку ----
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
        return Paths.line(GO_TO_FLOWER_START, GO_TO_FLOWER).linear(GO_TO_FLOWER_START, GO_TO_FLOWER);
    }

    private static Path intakeFlowerPath() {
        return Paths.line(GO_TO_FLOWER, INTAKE_FLOWER).linear(GO_TO_FLOWER, INTAKE_FLOWER);
    }

    private static Path shoot2Path() {
        // Курс: от 180° в конце сбора к -90° на точке выстрела.
        return Paths.line(INTAKE_FLOWER, SHOOT_2).linear(INTAKE_FLOWER, SHOOT_2);
    }

    private static Path parkPath() {
        return Paths.line(SHOOT_2, PARK).reverseTangent();
    }
}
