package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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

    // TODO: запустить "Foresight Tuner" на роботе и вставить сюда его код:
    //   public static ForesightConfig foresightConfig = new ForesightConfig(c -> { ... });
    // Пока здесь null, автономы не поедут, а TeleOp работает.
    public static ForesightConfig foresightConfig = null;

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
