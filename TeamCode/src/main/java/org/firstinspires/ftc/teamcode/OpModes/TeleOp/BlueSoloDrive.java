package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//bismillah

/** TeleOp синего альянса. Вся логика — в SoloDrive. */
@Configurable
@TeleOp(name = "AAABlue Solo TeleOp", group = "TeleOp")
public class BlueSoloDrive extends SoloDrive {

    private static final PoseFactory P = PoseFactory.degrees();

    public static Pose startPose = P.of(6, 135, 0);
    public static Pose resetPose = P.of(5.4, 38, 0);

    @Override protected boolean isBlue()               { return true; }
    @Override protected Pose startPose()               { return startPose; }
    @Override protected Pose resetPose()               { return resetPose; }
    @Override protected double driverHeadingOffset()   { return 0; }
}
