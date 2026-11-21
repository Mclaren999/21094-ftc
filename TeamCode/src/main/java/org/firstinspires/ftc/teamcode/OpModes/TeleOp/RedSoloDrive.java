//package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Poses;
////bismillah
///**
// * TeleOp красного альянса. Вся логика — в SoloDrive.
// * Позиции — зеркало синих относительно середины поля (x = 72).
// */
//@Configurable
//@TeleOp(name = "AAARed Solo TeleOp", group = "TeleOp")
//public class RedSoloDrive extends SoloDrive {
//
//    // Зеркало синих: x -> 144 - x, курс -> 180° - курс.
//    public static Pose startPose = Poses.mirroredDeg(7.1, 135, 0);
//    public static Pose resetPose = Poses.mirroredDeg(5.4, 38, 0);
//
//    @Override protected boolean isBlue()               { return false; }
//    @Override protected Pose startPose()               { return startPose; }
//    @Override protected Pose resetPose()               { return resetPose; }
//    @Override protected double driverHeadingOffset()   { return Math.PI; }
//}
