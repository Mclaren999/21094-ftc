package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

import java.util.Collections;
import java.util.Map;

/**
 * Заглушка на время, пока Foresight не настроен.
 * При следовании по пути и удержании точки всегда даёт нулевую мощность,
 * поэтому ненастроенный робот никуда не поедет сам. Ручное управление работает.
 */
public class UntunedAlgorithm implements Algorithm {

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker,
                                     MotionState state, double deltaTime) {
        return DrivePowers.zero();
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state,
                                     boolean useScaling, double deltaTime) {
        return DrivePowers.zero();
    }

    @Override public double completion()        { return 0; }
    @Override public Pose closestPose()         { return Pose.zero(); }
    @Override public Vector2D closestTangent()  { return Vector2D.cartesian(1, 0); }
    @Override public Vector2D closestNormal()   { return Vector2D.cartesian(0, 1); }
    @Override public double curvature()         { return 0; }
    @Override public double remainingDistance() { return 0; }
    @Override public boolean atParametricEnd()  { return true; }
    @Override public void reset()               { }
    @Override public boolean isBusy()           { return false; }

    @Override
    public Map<String, Object> debug() {
        return Collections.emptyMap();
    }
}
