package frc.robot.Config.Drive;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Config.ConfigLoader;

public class DriveConfigWriter {
    private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    private DriveConfigWriter() {
    }

    public static void writeAbsoluteOffsets(DriveConfig.DoubleGroup offsets) {
        try {
            File file = new File(Filesystem.getDeployDirectory(), "driveConfig.json");
            DriveConfig config = ConfigLoader.loadDriveConfig();
            config.encoders.absoluteOffsetsRad = offsets;

            try (Writer writer = new FileWriter(file)) {
                gson.toJson(config, writer);
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to save drive config", e);
        }
    }
}
