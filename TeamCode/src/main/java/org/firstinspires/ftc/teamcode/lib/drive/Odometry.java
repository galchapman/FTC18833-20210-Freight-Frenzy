package org.firstinspires.ftc.teamcode.lib.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Odometry extends ThreeTrackingWheelLocalizer {
    private final DoubleSupplier[] m_encoders;
    private final DoubleSupplier[] m_velocities;

    public Odometry(DoubleSupplier[] encoders, Pose2d[] wheels, DoubleSupplier[] velocities) {
        super(Arrays.asList(wheels.clone()));
        this.m_encoders = encoders;
        this.m_velocities = velocities;
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                m_encoders[0].getAsDouble(),
                m_encoders[1].getAsDouble(),
                m_encoders[2].getAsDouble()
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                m_velocities[0].getAsDouble(),
                m_velocities[1].getAsDouble(),
                m_velocities[2].getAsDouble()
        );
    }
}