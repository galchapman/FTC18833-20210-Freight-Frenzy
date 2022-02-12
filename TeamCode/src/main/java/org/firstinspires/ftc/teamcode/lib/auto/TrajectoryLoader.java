package org.firstinspires.ftc.teamcode.lib.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import kotlin.jvm.functions.Function2;

public class TrajectoryLoader {
    private final String text;

    public TrajectoryLoader(String path) {
        StringBuilder builder = new StringBuilder();
        try {
            File file = new File(path);

            BufferedReader br = new BufferedReader(new FileReader(file));
            String line = br.readLine();

            while (line != null) {
                builder.append(line);
                builder.append(System.lineSeparator());
                line = br.readLine();
            }
            br.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        text = builder.toString();
    }

    public Map<String, Trajectory> getTrajectories(Function2<Pose2d, Boolean, TrajectoryBuilder> trajectoryBuilderSupplier) {
        try {
            JSONObject trajectory = new JSONObject(text);
            JSONArray sequences = trajectory.getJSONArray("sequences");
            Map<String, Trajectory> trajectories = new HashMap<>();

            for (int i = 0; i < sequences.length(); i++) {
                JSONObject sequence = sequences.getJSONObject(i);
                JSONArray segments = sequence.getJSONArray("segments");
                JSONObject start_pos = sequence.getJSONObject("start_pos");
                String name = sequence.getString("name");
                TrajectoryBuilder builder = trajectoryBuilderSupplier.invoke(
                        new Pose2d( start_pos.getDouble("x"),
                                start_pos.getDouble("y"),
                                Math.toRadians(start_pos.getDouble("heading"))),
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
                            builder.splineTo(new Vector2d(segment.getDouble("x"), segment.getDouble("y")), Math.toRadians(segment.getDouble("heading")));
                            break;
                        case "strafeLeft":
                            builder.strafeLeft(segment.getDouble("distance"));
                            break;
                        case "strafeRight":
                            builder.strafeRight(segment.getDouble("distance"));
                            break;
                        case "strafe":
                            builder.strafeTo(new Vector2d(
                                    segment.getDouble("x"),
                                    segment.getDouble("y")));
                            break;
                    }
                }

                trajectories.put(name, builder.build());
            }

            return trajectories;
        } catch (JSONException ignored) {}
        return null;
    }
}
