package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Poses;
//bismillah

/**
 * TeleOp красного альянса. Вся логика — в SoloDrive.
 * Позиции — зеркало синих (BlueSoloDrive) относительно середины поля x = 72.
 */
@Configurable
@TeleOp(name = "Red Solo TeleOp", group = "TeleOp")
public class RedSoloDrive extends SoloDrive {

    // Зеркало синих: x -> 144 - x, курс -> 180° - курс. Синяя (7.1, 135, 0°) -> красная (136.9, 135, 180°).
    public static Pose startPose = Poses.mirroredDeg(7.1, 135, 0);
    public static Pose resetPose = Poses.mirroredDeg(7.1, 135, 0);

    @Override protected boolean isBlue()               { return false; }
    @Override protected Pose startPose()               { return startPose; }
    @Override protected Pose resetPose()               { return resetPose; }
    // Водитель красных стоит с другой стороны поля: "вперёд" на стике — это -X поля.
    @Override protected double driverHeadingOffset()   { return Math.PI; }
}
