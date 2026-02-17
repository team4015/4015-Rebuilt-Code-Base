/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: ConfigLoader.java
 @Description: This class is used to load all the configurations from the JSON file from the
 deploy folder then is used in the Constants.java. This program will read the JSON file and
 converts the Java object and validates all the fields present
 ***********************************************************************************************/

package frc.robot.Config;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Config.Drive.DriveConfig;
import frc.robot.Config.IntakeExtendable.IntakeExtendableConfig;

public class ConfigLoader {
    public static DriveConfig loadDriveConfig() {
        //this method (Filesystem) will return where the robot code is located and tells which JSON file to read from
        try {
            File file = new File(
                Filesystem.getDeployDirectory(),
                "driveConfig.json"
            );

            //This will also automatically close the file reader  and convert the JSON file into DriveConfig objects
            try (Reader reader = new FileReader(file)) {
                DriveConfig config = new Gson().fromJson(reader, DriveConfig.class);
                validateConfig(config);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load drive config", e); //tell if it couldn't load the configurations
        }
    }

    public static IntakeExtendableConfig loadIntakeExtendableConfig(){
        //this method (Filesystem) will return where the robot code is located and tells which JSON file to read from
        try {
            File file = new File(
                    Filesystem.getDeployDirectory(),
                    "intakeExtendableConfig.json"
            );

            //This will also automatically close the file reader  and convert the JSON file into DriveConfig objects
            try (Reader reader = new FileReader(file)) {
                IntakeExtendableConfig config = new Gson().fromJson(reader, IntakeExtendableConfig.class);
                return config;
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to load drive config", e); //tell if it couldn't load the configurations
        }
    }

    //This method checks each critical  section of the JSON is present . Throws and exception if anything is missing
    private static void validateConfig(DriveConfig config) {
        if (config == null
            || config.module == null
            || config.dimensions == null
            || config.motors == null
            || config.motors.drive == null
            || config.motors.turning == null
            || config.encoders == null
            || config.encoders.absolutePorts == null
            || config.encoders.absoluteOffsetsRad == null
            || config.encoders.driveReversed == null
            || config.encoders.turningReversed == null
            || config.encoders.absoluteReversed == null
            || config.limits == null) {
            throw new IllegalArgumentException("driveConfig.json is missing one or more required sections");
        }
    }
}
