//package org.firstinspires.ftc.teamcode.OpModes.TeleOp;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////bismillah
///**
// * TeleOp красного альянса. Вся логика — в SoloDrive.
// * Позиции — зеркало синих относительно середины поля (x = 72).
// */
//@Configurable
//@TeleOp(name = "AAARed Solo TeleOp", group = "TeleOp")
//public class RedSoloDrive extends SoloDrive {
//
//    // Зеркалит x и курс: синяя (6, 135, 0°) -> красная (138, 135, 180°).
//    private static final PoseFactory P = PoseFactory.degrees().mirrorX(72);
//
//    public static Pose startPose = P.of(7.1, 135, 0);
//    public static Pose resetPose = P.of(5.4, 38, 0);
//
//    @Override protected boolean isBlue()               { return false; }
//    @Override protected Pose startPose()               { return startPose; }
//    @Override protected Pose resetPose()               { return resetPose; }
//    @Override protected double driverHeadingOffset()   { return Math.PI; }
//}
