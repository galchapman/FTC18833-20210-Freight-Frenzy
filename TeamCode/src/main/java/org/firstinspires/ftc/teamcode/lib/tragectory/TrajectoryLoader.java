package org.firstinspires.ftc.teamcode.lib.tragectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import kotlin.jvm.functions.Function2;

public class TrajectoryLoader {
    private final Function2<Pose2d, Boolean, TrajectoryBuilder> m_trajectoryBuilderSupplier;

    public TrajectoryLoader(Function2<Pose2d, Boolean, TrajectoryBuilder> trajectoryBuilderSupplier) {
        this.m_trajectoryBuilderSupplier = trajectoryBuilderSupplier;
    }

    public Trajectory[] load(String text) {
        try {
            JSONObject trajectory = new JSONObject(text);
            JSONArray sequences = trajectory.getJSONArray("sequences");
            Trajectory[] trajectories = new Trajectory[sequences.length()];

            for (int i = 0; i < sequences.length(); i++) {
                JSONObject sequence = sequences.getJSONObject(i);
                JSONArray segments = sequence.getJSONArray("segments");
                JSONObject start_pos = sequence.getJSONObject("start_pos");
                TrajectoryBuilder builder = m_trajectoryBuilderSupplier.invoke(
                        new Pose2d( start_pos.getDouble("x"),
                                    start_pos.getDouble("y"),
                                    start_pos.getDouble("heading")),
                        sequence.getBoolean("reversed"));

                for (int j = 0; j < segments.length(); j++) {
                    JSONObject segment = segments.getJSONObject(j);
                    switch (segment.getString("name")) {
                        case "forward":
                            builder.forward(segment.getDouble("distance"));
                            break;
                        case "back":
                            builder.back(segment.getDouble("distance"));
                            break;
                        case "spline":
                            builder.splineTo(new Vector2d(segment.getDouble("x"), segment.getDouble("y")), segment.getDouble("heading"));
                            break;
                        case "strafeLeft":
                            builder.strafeLeft(segment.getDouble("distance"));
                            break;
                        case "strafeRight":
                            builder.strafeRight(segment.getDouble("distance"));
                            break;
                    }
                }

                trajectories[i] = builder.build();
            }

            return trajectories;
        } catch (JSONException ignored) {}
        return null;
    }
}
