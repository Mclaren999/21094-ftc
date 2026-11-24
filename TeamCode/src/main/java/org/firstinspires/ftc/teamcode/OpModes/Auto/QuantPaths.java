package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

/**
 * Пути для QuantAuto (Pedro Pathing 2.1).
 * Точки перенесены из экспорта визуализатора (PoseFactory.degrees()) один в один:
 * Poses.deg(x, y, курс в градусах).
 */
public final class QuantPaths {

    public static final Pose START = Poses.deg(60, 133, 90);

    private static final Pose GOTO_FLOWER_1 = Poses.deg(60, 116, 90);

    private static final Pose GOTO_FLOWER_2_START = Poses.deg(60, 116, 90);
    private static final Pose GOTO_FLOWER_2 = Poses.deg(45.1148, 116.7013, 90);

    private static final Pose INTAKE_FLOWER_START = Poses.deg(45.1148, 116.7013, 90);
    private static final Pose INTAKE_FLOWER = Poses.deg(45.3274, 131.2218, 90);

    private static final Pose GOTO_SHOOT_START = Poses.deg(45.3274, 131.2218, 90);
    private static final Pose GOTO_SHOOT = Poses.deg(64.3913, 80.9078, 90);

    // В визуализаторе у начала пути курс 0°, а предыдущий путь заканчивается на 90°:
    // робот довернётся на 0° по ходу этого пути.
    private static final Pose SHOOT_START = Poses.deg(64.3913, 80.9078, 0);
    private static final Pose SHOOT = Poses.deg(57.0943, 10.0387, 0);

    // То же: начало парковки с курсом -60°, хотя shoot заканчивается на 0°.
    private static final Pose PARK_START = Poses.deg(57.0943, 10.0387, -60);
    private static final Pose PARK = Poses.deg(9.4888, 93.3275, 0);

    private QuantPaths() {
    }

    public static PathChain gotoFlower1(Follower f)  { return line(f, START, GOTO_FLOWER_1); }
    public static PathChain gotoFlower2(Follower f)  { return line(f, GOTO_FLOWER_2_START, GOTO_FLOWER_2); }
    public static PathChain intakeFlower(Follower f) { return line(f, INTAKE_FLOWER_START, INTAKE_FLOWER); }
    public static PathChain gotoShoot(Follower f)    { return line(f, GOTO_SHOOT_START, GOTO_SHOOT); }
    public static PathChain shoot(Follower f)        { return line(f, SHOOT_START, SHOOT); }
    public static PathChain park(Follower f)         { return line(f, PARK_START, PARK); }

    /** Прямая с плавным поворотом от курса start к курсу end (аналог .linear(start, end)). */
    private static PathChain line(Follower f, Pose start, Pose end) {
        return f.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
