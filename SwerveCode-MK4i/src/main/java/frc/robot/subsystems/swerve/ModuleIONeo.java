package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ModuleIONeo implements ModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final CANcoder absoluteEncoder;

    public ModuleIONeo(int driveId, int steerId, int encoderId) {
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(encoderId);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveMotor.configure(driveConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerMotor.configure(steerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getEncoder().getPosition() / SwerveConstants.DRIVE_GEAR_RATIO);
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getEncoder().getVelocity() / 60.0 / SwerveConstants.DRIVE_GEAR_RATIO);
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.steerAbsolutePositionRad = Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        inputs.steerRelativePositionRad = Units.rotationsToRadians(steerMotor.getEncoder().getPosition() / SwerveConstants.STEER_GEAR_RATIO);
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerMotor.getEncoder().getVelocity() / 60.0 / SwerveConstants.STEER_GEAR_RATIO);
        inputs.steerAppliedVolts = steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
        inputs.steerCurrentAmps = steerMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setSteerVoltage(double volts) {
        steerMotor.setVoltage(volts);
    }
}
