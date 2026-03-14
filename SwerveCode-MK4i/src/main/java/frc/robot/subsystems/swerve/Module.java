package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final PIDController driveController = new PIDController(0.1, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
    private final PIDController steerController = new PIDController(5.0, 0, 0);

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(Rotation2d.fromRadians(inputs.steerAbsolutePositionRad));

        double driveVolts = driveController.calculate(inputs.driveVelocityRadPerSec, state.speedMetersPerSecond) +
                            driveFeedforward.calculate(state.speedMetersPerSecond);
        io.setDriveVoltage(driveVolts);

        double steerVolts = steerController.calculate(inputs.steerAbsolutePositionRad, state.angle.getRadians());
        io.setSteerVoltage(steerVolts);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * SwerveConstants.WHEEL_RADIUS_METERS, 
            Rotation2d.fromRadians(inputs.steerAbsolutePositionRad)
        );
    }
}