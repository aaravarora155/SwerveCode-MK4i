package frc.robot.subsystems.swerve;
import org.littletonrobotics.junction.AutoLog;
public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs{
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;
        public double driveCurrentAmps;
        public double steerAbsolutePositionRad;
        public double steerRelativePositionRad;
        public double steerVelocityRadPerSec;
        public double steerAppliedVolts;
        public double steerCurrentAmps;
    }
    public default void updateInputs(ModuleIOInputs inputs){}
    public default void setDriveVoltage(double volts){}
    public default void setSteerVoltage(double volts){}
    public default void setDriveBrakeMode(boolean enable){}
}
