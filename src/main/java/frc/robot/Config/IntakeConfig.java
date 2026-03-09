package frc.robot.Config;

import com.google.gson.annotations.SerializedName;

public class IntakeConfig {
    public GearRatio gearRatio;
    @SerializedName(value = "port", alternate = {"ports"})
    public Port port;
    public PID pid;
    public Settings settings;

    public static class GearRatio{
        public double intake;
    }

    public static class Port{
        public Motors intake;
    }

    public static class Motors{
        public int system;
    }

    public static class DigitalInput{
        public int limitSwitch;
    }

    public static class PID{
        public PIDValue intake;
    }

    public static class PIDValue{
        public double p;
        public double i;
        public double d;
    }

    public static class Settings {
        public boolean intakeMotorReversed;
    }
}
