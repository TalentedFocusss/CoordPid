package frc.robot.commands.auto.driveCommands.CoordEncoders;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Titan;

public class CoordPid extends CommandBase {
    private final Titan titan;

    private double targetX;
    private double targetY;
    private final boolean isRelative;

    private double startX;
    private double startY;
    private double startTime;

    private final double kP = 0.015;
    private final double MAX_SPEED = 0.4;
    private final double MIN_SPEED = 0.12;
    private final double RAMP_TIME_SEC = 0.8;

    public CoordPid(Titan titan, double x, double y) {
        this(titan, x, y, false);
    }

    public CoordPid(Titan titan, double x, double y, boolean isRelative) {
        this.titan = titan;
        addRequirements(titan);
        
        this.targetX = x;
        this.targetY = y;
        this.isRelative = isRelative;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public void initialize() {
        this.startX = titan.getAx();
        this.startY = titan.getAy();

        if (this.isRelative) {
            this.targetX += this.startX;
            this.targetY += this.startY;
        }

        this.startTime = Timer.getFPGATimestamp(); 
        DriverStation.reportWarning("CoordPid timer logic started...", false);
    }

    @Override
    public void execute() {
        double currentX = titan.getAx();
        double currentY = titan.getAy();

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        
        double distance = Math.hypot(errorX, errorY);

        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        double ramp = Math.min(1.0, timeElapsed / RAMP_TIME_SEC); 
        ramp = 0.2 + (0.8 * ramp); 

        double speedX = errorX * kP;
        double speedY = errorY * kP;

        double speedMag = Math.hypot(speedX, speedY);

        if (speedMag > 0.01) {
            if (speedMag > MAX_SPEED) {
                speedX = (speedX / speedMag) * MAX_SPEED;
                speedY = (speedY / speedMag) * MAX_SPEED;
            } else if (speedMag < MIN_SPEED && distance > 5) {
                speedX = (speedX / speedMag) * MIN_SPEED;
                speedY = (speedY / speedMag) * MIN_SPEED;
            }
        }

        speedX *= ramp;
        speedY *= ramp;

        titan.drivePID(speedY, speedX, 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        titan.stopWheels();
    }

    @Override
    public boolean isFinished() {
        double distanceToTarget = Math.hypot(targetX - titan.getAx(), targetY - titan.getAy());
        return titan.isDifferenceZero() && (distanceToTarget < 10);
    }
}