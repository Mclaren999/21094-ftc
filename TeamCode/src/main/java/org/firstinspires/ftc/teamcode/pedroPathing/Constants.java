package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Настройки Pedro Pathing 3.
 *
 * Ходовая и Pinpoint перенесены из констант Pedro 2.
 * Foresight (следование по пути) в Pedro 3 настраивается ТОЛЬКО на роботе:
 * запустите "Foresight Tuner" из AutoTune и вставьте его результат в foresightConfig.
 */
public class Constants {

    /* ================= ХОДОВАЯ (перенесено из Pedro 2) ================= */

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true); // BRAKE в TeleOp
    });

    /* ================= PINPOINT (перенесено из Pedro 2) ================= */

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.44); // было forwardPodY
        c.yPodOffset.set(3.05); // было strafePodX
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /* ================= FORESIGHT (НЕ НАСТРОЕНО) ================= */

    // Результат Foresight Tuner (AutoTune), 21.11.2026.
    // Перетюнить, если поменялись вес робота, колёса или моторы.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(1.7731860350721977);
                Controller secondaryTranslationalForward = Controller.proportional(0.6551448595557027);
                Controller primaryTranslationalLateral = Controller.proportional(0.5060054455165391);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18695549140384246);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012364950802041485));
                c.brake.set(Controller.proportionalFeedforward(0.010510208181735262));

                c.headingFeedback.set(Controller.proportional(5.5640585285038755));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0653564325032162, 0.005968978781383945));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09748195799994455, 0.04839689763514247));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012613219930042274, 0.0024681829027186583));

                c.maxAchievableForwardVelocity.set(82.73315582879744);
                c.maxAchievableStrafeVelocity.set(63.161851642374835);
                c.naturalForwardDeceleration.set(32.04932982110785);
                c.naturalStrafeDeceleration.set(62.55689881279272);
            }
    );
    /** true, когда Foresight настроен и роботу можно ездить по путям. */
    public static boolean isTuned() {
        return foresightConfig != null;
    }

    /* ================= СОЗДАНИЕ ================= */

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm createAlgorithm() {
        return isTuned() ? new Foresight(foresightConfig) : new UntunedAlgorithm();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                createLocalizer(hardwareMap),
                createDrivetrain(hardwareMap),
                createAlgorithm()
        );
    }
}
