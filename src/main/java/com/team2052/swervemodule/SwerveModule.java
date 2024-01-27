package com.team2052.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModule {
   private final CANSparkMax driveMotor;
   private final CANSparkMax steerMotor;
   private final CANcoder canCoder;
   private String debugName;

   public SwerveModule(
        String debugName,
        int driveMotorChannel,
        int steerMotorChannel,
        int canCoderChannel,
        Rotation2d steerOffset
   ) {

    this.debugName = debugName;

        /*
         * CANCoder Initialization
         */
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfiguration.MagnetSensor.MagnetOffset = -steerOffset.getRotations();

        canCoder = new CANcoder(canCoderChannel);
        
        checkError(
            "Failed to configure CANCoder",
            canCoder.getConfigurator().apply(
                canCoderConfiguration,
                SwerveConstants.CAN_TIMEOUT_MS
            )
        );

        // TODO: Reduce CAN status frame rates

    /*
     * Drive Motor Initialization
     */

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    checkError("Failed to restore drive motor factory defaults", driveMotor.restoreFactoryDefaults());

    checkError(
        "Failed to set drive motor periodic status frame rate",
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
    );

    checkError(
        "Failed to set drive motor idle mode",
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
    );

    driveMotor.setInverted(SwerveConstants.NeoSwerveModule.DRIVE_INVERTED);

    checkError(
        "Failed to enable drive motor voltage compensation",
        driveMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
    );

    checkError(
        "Failed to set steer motor current limit",
        driveMotor.setSmartCurrentLimit(
            SwerveConstants.DRIVE_STALL_CURRENT_LIMIT_AMPS, 
            SwerveConstants.DRIVE_FREE_CURRENT_LIMIT_AMPS
        )
    );

    // Drive Motor encoder initialization
    RelativeEncoder driveEncoder = driveMotor.getEncoder();

    // Conversion factor for switching between ticks and meters in terms of meters per tick
    double drivePositionConversionFactor = Math.PI * SwerveConstants.NeoSwerveModule.WHEEL_DIAMETER_METERS * 
        SwerveConstants.NeoSwerveModule.DRIVE_REDUCTION;
    
    checkError(
        "Failed to set drive motor encoder conversion factors",
        // Set the position conversion factor so the encoder will automatically convert ticks to meters
        driveEncoder.setPositionConversionFactor(drivePositionConversionFactor),
        // Velocity of the encoder in meters per second
        driveEncoder.setVelocityConversionFactor(drivePositionConversionFactor / 60.0)
    );

    /*
     * Steer Motor Initialization
     */

    steerMotor = new CANSparkMax(steerMotorChannel, MotorType.kBrushless);
    checkError("Failed to restore drive motor factory defaults", steerMotor.restoreFactoryDefaults());

    checkError(
        "Failed to set drive motor periodic status frame rate",
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
    );

    checkError(
        "Failed to set drive motor idle mode",
        steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
    );

    steerMotor.setInverted(SwerveConstants.NeoSwerveModule.STEER_INVERTED);

    checkError(
        "Failed to enable steer motor voltage compensation",
        steerMotor.enableVoltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS)
    );

    checkError(
        "Failed to set steer motor current limit",
        steerMotor.setSmartCurrentLimit((int) SwerveConstants.STEER_CURRENT_LIMIT_AMPS)
    );

    // Drive Motor encoder initialization
    RelativeEncoder steerEncoder = steerMotor.getEncoder();

    // Conversion factor for switching between ticks and meters in terms of meters per tick
        double steerPositionConversionFactor = 2.0 * Math.PI * SwerveConstants.NeoSwerveModule.STEER_REDUCTION;
    
    checkError(
        "Failed to set drive motor encoder conversion factors",
        // Set the position conversion factor so the encoder will automatically convert ticks to meters
        steerEncoder.setPositionConversionFactor(steerPositionConversionFactor),
        // Velocity of the encoder in meters per second
        steerEncoder.setVelocityConversionFactor(steerPositionConversionFactor / 60.0)
    );

    checkError(
        "Failed to set steer motor encoder position",
        steerEncoder.setPosition(Math.toRadians(canCoder.getAbsolutePosition().getValueAsDouble()))
    );

    SparkPIDController steerController = steerMotor.getPIDController();
    checkError(
        "Failed to configure steer motor PID",
        steerController.setP(SwerveConstants.NeoSwerveModule.STEER_MOTOR_P),
        steerController.setI(SwerveConstants.NeoSwerveModule.STEER_MOTOR_I),
        steerController.setD(SwerveConstants.NeoSwerveModule.STEER_MOTOR_D),
        steerController.setPositionPIDWrappingMinInput(-Math.PI),
        steerController.setPositionPIDWrappingMaxInput(Math.PI),
        steerController.setPositionPIDWrappingEnabled(true)
    );

    checkError(
        "Failed to set steer motor PID feedback device",
        steerController.setFeedbackDevice(steerEncoder)
    );
   }

    public SwerveModuleState getState() {
        // Both encoder values are automatically in units of meters per second and
        // radians because of the position and velocity conversion factors
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition() % (2.0 * Math.PI)
            )
        );
    }

    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) {
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState = SwerveModuleState.optimize(
            desiredState,
            getState().angle
        );

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(
            desiredState.speedMetersPerSecond / getMaxVelocityMetersPerSecond()
        );

        steerMotor.getPIDController().setReference(
            desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
            )
        );
    }

    public static double getMaxVelocityMetersPerSecond() {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.NeoSwerveModule.NEO_ROUNDS_PER_MINUTE / 60 * SwerveConstants.NeoSwerveModule.DRIVE_REDUCTION * 
            SwerveConstants.NeoSwerveModule.WHEEL_DIAMETER_METERS * Math.PI;
    }

    @SuppressWarnings("unchecked")
    protected <E> void checkError(String message, E... errors) {
        for (E error : errors) {
            if (error != REVLibError.kOk) {
                DriverStation.reportError(
                    message + " on [" + debugName + "] module: " + error.toString(),
                    false
                );
            }
        }
    }
}  
