package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Настройки Pedro Pathing 2.1.
 *
 * PIDF — из старой версии на Pedro 2 (12.11.2026).
 * Скорости и торможение (xVelocity, yVelocity, ZeroPowerAcceleration) — свежий замер
 * Foresight Tuner от 21.11.2026 (те же величины, измерены на текущем роботе).
 * Перепроверить через меню "Tuning" (Pedro Pathing) на Driver Station.
 */
@Configurable
public class Constants {

    /* ================= FOLLOWER (PIDF) ================= */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.3)
            .forwardZeroPowerAcceleration(-32.049)
            .lateralZeroPowerAcceleration(-62.557)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.006, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.012, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.18, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.08, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0000015, 0.6, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.000002, 0.6, 0))
            .centripetalScaling(0.0004);

    /* ================= ОГРАНИЧЕНИЯ ПУТИ ================= */

    // tValueConstraint, timeoutConstraint (мс), brakingStrength, brakingStart — как было в Pedro 2
    public static PathConstraints pathConstraints = new PathConstraints(0.9, 100, 5, 0.8);

    /* ================= ХОДОВАЯ ================= */

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .leftRearMotorName("leftRear")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(82.733)
            .yVelocity(63.162);

    /* ================= PINPOINT ================= */

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.44)
            .strafePodX(3.05)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /* ================= СОЗДАНИЕ ================= */

    /** Новый Follower на каждый запуск OpMode (не кешируем: у каждого запуска свой hardwareMap). */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
