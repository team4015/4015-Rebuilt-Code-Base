/*************************************************************************************************
 @Name: Gursahaj Chawla
 @Date: 2/10/2026
 @File: DriveConfig.java
 @Description: This class in the program to get thee DriveConfig from the json file and get its
 equivalent and put them into each object class. It used the @SerializedName which tells the Gson
 what JSON key correspond to this java field. Each of these class related to the header of the
 JSON file and each variable corresponds to the variables below it and value associated with it
 ***********************************************************************************************/

package frc.robot;

import com.google.gson.annotations.SerializedName;

public class DriveConfig {
    public Module module;
    public Dimensions dimensions;
    public Motors motors;

    @SerializedName(value = "encoders", alternate = {"encoder"})
    public Encoders encoders;

    public Limits limits;

    public static class Module {
        public double wheelDiameterInches;
        public double driveGearRatio;
        public double turningGearRatio;

        @SerializedName(value = "turningKP", alternate = {"tuningKP"})
        public double turningKP;
    }

    public static class Dimensions {
        @SerializedName(value = "trackWidthInches", alternate = {"trachWidthInches"})
        public double trackWidthInches;

        public double wheelBaseInches;
    }

    public static class Motors {
        public IntGroup drive;
        public IntGroup turning;
    }

    public static class Encoders {
        @SerializedName(value = "absolutePorts", alternate = {"absolutePort"})
        public IntGroup absolutePorts;

        @SerializedName(value = "absoluteOffsetsRad", alternate = {"absolutePortsOffsetRad", "absoluteOffsetRad"})
        public DoubleGroup absoluteOffsetsRad;

        public BooleanGroup driveReversed;
        public BooleanGroup turningReversed;
        public BooleanGroup absoluteReversed;
    }

    public static class IntGroup {
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;
    }

    public static class DoubleGroup {
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;
    }

    public static class BooleanGroup {
        public boolean frontLeft;
        public boolean frontRight;
        public boolean backLeft;
        public boolean backRight;
    }

    public static class Limits {
        @SerializedName(value = "maxSpeedMps", alternate = {"maxSpeed"})
        public double maxSpeedMps;

        public double maxAngularSpeedRadPerSec;
        public double teleopSpeedScale;
    }
}
