public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final CANEncoder driveEncoder;
    private final CANEncoder steerEncoder;
    
    private double moduleAngle;
    private double moduleSpeed;
    
    public SwerveModule(int driveMotorID, int steerMotorID) {
      driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
      steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
      
      driveEncoder = new CANEncoder(driveMotor);
      steerEncoder = new CANEncoder(steerMotor);
      
      moduleAngle = 0.0;
      moduleSpeed = 0.0;
    }
    
    public void setModuleAngle(double angle) {
      moduleAngle = angle;
      // Code to set the steering motor to the desired angle
    }
    
    public double getModuleAngle() {
      return moduleAngle;
    }
    
    public void setModuleSpeed(double speed) {
      moduleSpeed = speed;
      // Code to set the drive motor to the desired speed
    }
    
    public double getModuleSpeed() {
      return moduleSpeed;
    }
  }