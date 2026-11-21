package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//bismillah

/** TeleOp синего альянса. Вся логика — в SoloDrive. */
@Configurable
@TeleOp(name = "Bismillah Inshallah ", group = "TeleOp")
public class BlueSoloDrive extends SoloDrive {

    private static final PoseFactory P = PoseFactory.degrees();

    public static Pose startPose = P.of(7.1, 135, 0);
    public static Pose resetPose = P.of(7.4, 57.5, 0);

    @Override protected boolean isBlue()               { return true; }
    @Override protected Pose startPose()               { return startPose; }
    @Override protected Pose resetPose()               { return resetPose; }
    @Override protected double driverHeadingOffset()   { return 0; }
}
