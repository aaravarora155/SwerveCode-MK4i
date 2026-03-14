package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
        public double connected;
        public double yawPositionRad;
        public double yawVelocityRadPerSec;
    }
    public default void updateInputs(GyroIOInputs inputs){}
    public default void resetYaw(){}

}
