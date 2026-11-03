/* 
 THIS IS THE ACTUAL TURRET FILE, NOT THE OTHER ONE

public class Turret {

    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;

    public Turret(HardwareMap hardwareMap) {
        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        flywheelB.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private Servo turretAzimuthA;
    private Servo turretAzimuthB;
    private Servo shooterAngleA;
    private Servo shooterAngleB;

    public Turret(Servo turretAzimuthA, Servo turretAzimuthB, Servo shooterAngleA, Servo shooterAngleB) {
        this.turretAzimuthA = turretAzimuthA;
        this.turretAzimuthB = turretAzimuthB;
        this.shooterAngleA = shooterAngleA;
        this.shooterAngleB = shooterAngleB;
    }

    public void Initialize() {
        turretAzimuthA.setPosition(0);
        turretAzimuthB.setPosition(0);

        shooterAngleA.setPosition(0);
        shooterAngleB.setPosition(0);
    }

    public void Target(double tx, double vx, double launchVelocity, double launchAngle) {
        double currentAngle = turretAzimuthA.getPosition() * 180; // Convert servo position to angle (degrees)
        double targetAngle = currentAngle + tx; // Calculate the target angle

        // Calculate the aim offset based on the robot's translational velocity
        double velocityOffset = -Math.atan(vx/(launchVelocity * Math.cos(launchAngle)));

        targetAngle += Math.toDegrees(velocityOffset); // Adjust the target angle based on the velocity offset
        
        if (targetAngle > 360) {
            targetAngle -= 360; // Wrap around if the angle exceeds 360 degrees
        } else if (targetAngle < 0) {
            targetAngle += 360; // Wrap around if the angle is negative
        }
        double servoPosition = (targetAngle - 180) / 180; // Convert angle (degrees) to servo position (-1 to 1)

        // Set the turret angle
        turretAzimuthA.setPosition(servoPosition);
        turretAzimuthB.setPosition(servoPosition);
    }
    
}
*/