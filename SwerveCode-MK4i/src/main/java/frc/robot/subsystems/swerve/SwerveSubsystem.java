package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    public SwerveSubsystem(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO) {
        this.gyroIO = gyroIO;
        modules = new Module[] {
            new Module(flIO, 0),
            new Module(frIO, 1),
            new Module(blIO, 2),
            new Module(brIO, 3)
        };

        kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_OFFSETS);
        
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            new Rotation2d(), 
            getModulePositions(), 
            new Pose2d()
        );
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (Module module : modules) {
            module.periodic();
        }

        poseEstimator.update(Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions());
        Logger.recordOutput("Drive/Pose", getPose());
        
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(
                modules[i].getPosition().distanceMeters, 
                Rotation2d.fromRadians(0) // PlaceHolder, actually we should get it cleanly from module, but getPosition doesn't return velocity.
            );
        }
        // Logger.recordOutput("Drive/ModuleStates", states);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.5);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        };
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
