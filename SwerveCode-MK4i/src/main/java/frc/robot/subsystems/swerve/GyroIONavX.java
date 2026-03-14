package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXUpdateRate;

public class GyroIONavX implements GyroIO {
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected() ? 1.0 : 0.0;
        inputs.yawPositionRad = edu.wpi.first.math.util.Units.degreesToRadians(-navx.getYaw());
        inputs.yawVelocityRadPerSec = edu.wpi.first.math.util.Units.degreesToRadians(navx.getRate());
    }

    @Override
    public void resetYaw() {
        navx.reset();
    }
}
