package frc.robot.utils.parsing;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ParseAutoStartIntoPose2d {

    public static Pose2d parseJson(Path file) throws IOException {
        String content = new String(Files.readAllBytes(file));
        JSONObject jsonObject = new JSONObject(content);
        JSONObject startingPose = jsonObject.getJSONObject("startingPose");
        JSONObject position = startingPose.getJSONObject("position");
        double x = position.getDouble("x");
        double y = position.getDouble("y");
        double rotation = startingPose.getDouble("rotation");
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(rotation)));
    }
}
