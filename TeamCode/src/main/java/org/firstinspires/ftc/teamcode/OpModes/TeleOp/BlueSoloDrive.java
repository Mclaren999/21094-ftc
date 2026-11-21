package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Poses;
//bismillah

/** TeleOp синего альянса. Вся логика — в SoloDrive. */
@Configurable
@TeleOp(name = "Bismillah Inshallah ", group = "TeleOp")
public class BlueSoloDrive extends SoloDrive {

    public static Pose startPose = Poses.deg(7.1, 135, 0);
    public static Pose resetPose = Poses.deg(7.1, 135, 0);

    @Override protected boolean isBlue()               { return true; }
    @Override protected Pose startPose()               { return startPose; }
    @Override protected Pose resetPose()               { return resetPose; }
    @Override protected double driverHeadingOffset()   { return 0; }
}
