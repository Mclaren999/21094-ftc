package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;

/**
 * Передаёт позицию робота из автонома в TeleOp.
 * Статическое поле живёт, пока приложение Robot Controller не перезапущено.
 */
public final class PoseStorage {

    // Последняя позиция из автонома; null, если автоном не запускался.
    public static Pose autoEndPose = null;

    private PoseStorage() {
    }
}
