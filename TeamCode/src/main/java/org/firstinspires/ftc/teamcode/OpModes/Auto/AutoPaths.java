package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Poses;

/** Пути для MainAuto (Pedro Pathing 2.1). Пути строятся через follower.pathBuilder(). */
public final class AutoPaths {

    public static final Pose START = Poses.deg(31, 9, 180);

    private static final Pose POINT_11_START = Poses.deg(31, 9, 180);
    private static final Pose POINT_11 = Poses.deg(2, 5, 185);

    private static final Pose POINT_22_START = Poses.deg(6.8565, 6.8565, -120);
    private static final Pose POINT_22_CONTROL = Poses.deg(3, 75.2185, 0);
    private static final Pose POINT_22 = Poses.deg(50, 143, -90);

    private static final Pose POINT_33_START = Poses.deg(50, 143, -90);
    private static final Pose POINT_33 = Poses.deg(43, 110, 0);

    private static final Pose POINT_44 = Poses.deg(7, 106, 90);

    private static final Pose POINT_55_START = Poses.deg(43, 110, 0);
    private static final Pose POINT_55 = Poses.deg(4, 110, 0);

    private static final Pose POINT_66_START = Poses.deg(43, 129, 70);
    private static final Pose POINT_66 = Poses.deg(12, 42, 70);

    private static final Pose POINT_77_START = Poses.deg(12, 42, 180);
    private static final Pose POINT_77 = Poses.deg(3, 42, 180);

    private static final Pose POINT_88_START = Poses.deg(3, 42, 180);
    private static final Pose POINT_88 = Poses.deg(54, 1, 125);

    private static final Pose POINT_99_START = Poses.deg(54, 2, 125);
    private static final Pose POINT_99 = Poses.deg(9, 94, 125);

    private AutoPaths() {
    }

    public static PathChain path11(Follower f) {
        return line(f, POINT_11_START, POINT_11);
    }

    public static PathChain path22(Follower f) {
        return f.pathBuilder()
                .addPath(new BezierCurve(POINT_22_START, POINT_22_CONTROL, POINT_22))
                .setLinearHeadingInterpolation(POINT_22_START.getHeading(), POINT_22.getHeading())
                .build();
    }

    public static PathChain path33(Follower f) {
        // Кривая от конца path22; POINT_33_START служит контрольной точкой, курс по касательной.
        return f.pathBuilder()
                .addPath(new BezierCurve(POINT_22, POINT_33_START, POINT_33))
                .setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain path44(Follower f) {
        return line(f, POINT_33, POINT_44);
    }

    public static PathChain path55(Follower f) {
        return line(f, POINT_55_START, POINT_55);
    }

    public static PathChain path66(Follower f) {
        return line(f, POINT_66_START, POINT_66);
    }

    public static PathChain path77(Follower f) {
        return line(f, POINT_77_START, POINT_77);
    }

    public static PathChain path88(Follower f) {
        return line(f, POINT_88_START, POINT_88);
    }

    public static PathChain path99(Follower f) {
        return line(f, POINT_99_START, POINT_99);
    }

    /** Прямая с плавным поворотом от курса start к курсу end. */
    private static PathChain line(Follower f, Pose start, Pose end) {
        return f.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}
