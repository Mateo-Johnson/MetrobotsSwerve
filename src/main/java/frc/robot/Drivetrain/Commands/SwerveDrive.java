package frc.robot.Drivetrain.Commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDrive {
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule rearLeftModule;
    private SwerveModule rearRightModule;
    private final static CommandXboxController primaryDriver = new CommandXboxController(0);

    public SwerveDrive() {
        // Initialize the SparkMAX controllers for each swerve module
        CANSparkMax frontLeftMotor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax frontLeftMotor2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeftModule = new SwerveModule(frontLeftMotor1, frontLeftMotor2);

        CANSparkMax frontRightMotor1 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax frontRightMotor2 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRightModule = new SwerveModule(frontRightMotor1, frontRightMotor2);

        CANSparkMax rearLeftMotor1 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax rearLeftMotor2 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearLeftModule = new SwerveModule(rearLeftMotor1, rearLeftMotor2);

        CANSparkMax rearRightMotor1 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax rearRightMotor2 = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRightModule = new SwerveModule(rearRightMotor1, rearRightMotor2);

    }

    public void teleopPeriodic() {
        // Get the joystick inputs
        double forwardSpeed = primaryDriver.getLeftY(); // Not negative because the joystick isn't inverted
        double strafeSpeed = primaryDriver.getLeftX();
        double rotationSpeedX = primaryDriver.getRightX();
        double rotationSpeedY = primaryDriver.getRightY();

        // Calculate the desired speeds and angles for each module based on the joystick inputs
        double frontLeftSpeed = forwardSpeed - strafeSpeed - rotationSpeedY;
        double frontLeftAngle = Math.atan2(forwardSpeed - strafeSpeed, forwardSpeed + strafeSpeed) * 180 / Math.PI;

        double frontRightSpeed = forwardSpeed + strafeSpeed + rotationSpeedY;
        double frontRightAngle = Math.atan2(forwardSpeed + strafeSpeed, forwardSpeed - strafeSpeed) * 180 / Math.PI;

        double rearLeftSpeed = forwardSpeed + strafeSpeed - rotationSpeedX;
        double rearLeftAngle = Math.atan2(forwardSpeed + strafeSpeed, forwardSpeed - strafeSpeed) * 180 / Math.PI;

        double rearRightSpeed = forwardSpeed - strafeSpeed + rotationSpeedX;
        double rearRightAngle = Math.atan2(forwardSpeed - strafeSpeed, forwardSpeed + strafeSpeed) * 180 / Math.PI;

        // Set the desired speeds and angles for each module
        frontLeftModule.setSpeedAndAngle(frontLeftSpeed, frontLeftAngle);
        frontRightModule.setSpeedAndAngle(frontRightSpeed, frontRightAngle);
        rearLeftModule.setSpeedAndAngle(rearLeftSpeed, rearLeftAngle);
        rearRightModule.setSpeedAndAngle(rearRightSpeed, rearRightAngle);
    }

    public static void main(String[] args) {
        // Create an instance of SwerveDrive and run the teleopPeriodic method
        SwerveDrive swerveDrive = new SwerveDrive();
        while (true) {
            swerveDrive.teleopPeriodic();
        }
    }
}

class SwerveModule {
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private final double MAX_SPEED = 1.0; // Maximum motor speed

    public SwerveModule(CANSparkMax motor1, CANSparkMax motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    public void setSpeedAndAngle(double speed, double angle) {
        // Normalize the speed within the range [-1.0, 1.0]
        speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speed));
    
        // Calculate the individual motor speeds based on the desired speed and angle
        double motor1Speed = speed * Math.cos(Math.toRadians(angle));
        double motor2Speed = speed * Math.sin(Math.toRadians(angle));
    
        // Apply fine-tuning adjustments
        double fineTuneFactor = calculateFineTuneFactor(speed);
        motor1Speed *= fineTuneFactor;
        motor2Speed *= fineTuneFactor;
    
        // Determine which motor to invert based on movement direction
        if (speed >= 0.0) {
            if (angle >= 0.0 && angle < 90.0) {
                motor1Speed *= -1.0; // Invert motor1 when moving forward diagonally to the right
            } else if (angle >= 90.0 && angle < 180.0) {
                motor2Speed *= -1.0; // Invert motor2 when moving forward diagonally to the left
            }
        } else {
            if (angle >= 0.0 && angle < 90.0) {
                motor2Speed *= -1.0; // Invert motor2 when moving backward diagonally to the right
            } else if (angle >= 90.0 && angle < 180.0) {
                motor1Speed *= -1.0; // Invert motor1 when moving backward diagonally to the left
            }
        }
    
        // Set the motor speeds
        motor1.set(motor1Speed);
        motor2.set(motor2Speed);
    }

    private double calculateFineTuneFactor(double speed) {
        // Fine-tuning logic based on speed
        if (speed >= 0.5) {
            return 0.8; // Reduce speed by 20% for fine control
        } else if (speed <= -0.5) {
            return 0.6; // Reduce speed by 40% for fine control
        } else {
            return 1.0; // No fine-tuning needed
        }
    }
}