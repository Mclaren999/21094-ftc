package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Процедуры AutoTune для Pedro Pathing 3.
 *
 * Порядок на роботе:
 *   1. Mecanum Tuner  — проверить направления моторов (уже перенесены из Pedro 2).
 *   2. Pinpoint Tuner — проверить одометрию (уже перенесена из Pedro 2).
 *   3. Foresight Tuner — ОБЯЗАТЕЛЬНО: результат вставить в Constants.foresightConfig.
 *   4. Tests — проверить езду по линии и кривой.
 */
public class Tuning {

    @Tuner
    static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner
    static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner
    static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner
    static Procedure tests() {
        // Пока Foresight не настроен, доступны только тесты без следования по пути.
        return new Tests(
                Constants::createDrivetrain,
                Constants::createLocalizer,
                Constants.isTuned() ? Constants::createAlgorithm : null
        );
    }
}
