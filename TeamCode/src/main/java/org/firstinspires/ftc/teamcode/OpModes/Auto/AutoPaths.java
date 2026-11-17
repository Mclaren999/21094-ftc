package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** Пути для MainAuto (Pedro Pathing 3). Точки те же, что были в Pedro 2. */
public final class AutoPaths {

    private static final PoseFactory P = PoseFactory.degrees();

    public static final Pose START = P.of(31, 9, 180);

    private static final Pose POINT_11_START = P.of(31, 9, 180);
    private static final Pose POINT_11 = P.of(2, 5, 185);

    private static final Pose POINT_22_START = P.of(6.8565, 6.8565, -120);
    private static final Pose POINT_22_CONTROL = P.of(3, 75.2185, 0);
    private static final Pose POINT_22 = P.of(50, 143, -90);

    private static final Pose POINT_33_START = P.of(50, 143, -90);
    private static final Pose POINT_33 = P.of(43, 110, 0);

    private static final Pose POINT_44 = P.of(7, 106, 90);

    private static final Pose POINT_55_START = P.of(43, 110, 0);
    private static final Pose POINT_55 = P.of(4, 110, 0);

    private static final Pose POINT_66_START = P.of(43, 129, 70);
    private static final Pose POINT_66 = P.of(12, 42, 70);

    private static final Pose POINT_77_START = P.of(12, 42, 180);
    private static final Pose POINT_77 = P.of(3, 42, 180);

    private static final Pose POINT_88_START = P.of(3, 42, 180);
    private static final Pose POINT_88 = P.of(54, 1, 125);

    private static final Pose POINT_99_START = P.of(54, 2, 125);
    private static final Pose POINT_99 = P.of(9, 94, 125);

    private AutoPaths() {
    }

    public static Path path11() {
        return line(POINT_11_START, POINT_11);
    }

    public static Path path22() {
        return Paths.curve(POINT_22_START, POINT_22_CONTROL, POINT_22)
                .linear(POINT_22_START, POINT_22);
    }

    public static Path path33() {
        // Кривая от конца path22; POINT_33_START служит контрольной точкой, курс по касательной.
        return Paths.curve(POINT_22, POINT_33_START, POINT_33).tangent();
    }

    public static Path path44() {
        return line(POINT_33, POINT_44);
    }

    public static Path path55() {
        return line(POINT_55_START, POINT_55);
    }

    public static Path path66() {
        return line(POINT_66_START, POINT_66);
    }

    public static Path path77() {
        return line(POINT_77_START, POINT_77);
    }

    public static Path path88() {
        return line(POINT_88_START, POINT_88);
    }

    public static Path path99() {
        return line(POINT_99_START, POINT_99);
    }

    /** Прямая с плавным поворотом от курса start к курсу end. */
    private static Path line(Pose start, Pose end) {
        return Paths.line(start, end).linear(start, end);
    }
}
