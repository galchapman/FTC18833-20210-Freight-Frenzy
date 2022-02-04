package org.firstinspires.ftc.teamcode.lib.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Odometry extends ThreeTrackingWheelLocalizer {
    private final DoubleSupplier[] m_encoders;

    public Odometry(DoubleSupplier[] encoders, Pose2d[] wheels) {
        super(Arrays.asList(wheels.clone()));
        this.m_encoders = encoders;
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
}