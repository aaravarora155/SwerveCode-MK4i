package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConstants {
    public static final double WHEEL_RADIUS_METERS = 0.0508; //TODO:Tune
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); //TODO:Tune
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0; //TODO:Tune
    
    //TODO:Tune
    public static final Translation2d[] MODULE_OFFSETS = new Translation2d[] {
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3),
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3)
    };
    
    public static final Slot0Configs DRIVE_PID_GAINS = new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0); //TODO:Tune
}
