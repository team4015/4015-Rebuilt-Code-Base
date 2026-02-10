package frc.robot;

import java.io.File;
import java.io.FileReader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigLoader {
    public static DriveConfig loadDriveConfig(){
        try {
            File file = new File(
                Filesystem.getDeployDirectory(),
                "driveConfig.json"
            );

            return new Gson().fromJson(new FileReader(file), DriveConfig.class);
        } catch (Exception e){
            throw new RuntimeException("Failed to load drive config", e);
        }
    }
}
