package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

/** Позиции в градусах (в Pedro 2 Pose хранит курс в радианах). */
public final class Poses {

    /** Середина поля по X: красные позиции — зеркало синих относительно x = 72. */
    public static final double FIELD_MID_X = 72;

    private Poses() {
    }

    /** Pose(x, y, курс в градусах). */
    public static Pose deg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    /** Зеркало синей позиции для красных: x -> 144 - x, курс -> 180° - курс. */
    public static Pose mirroredDeg(double x, double y, double headingDeg) {
        return deg(2 * FIELD_MID_X - x, y, 180 - headingDeg);
    }
}
