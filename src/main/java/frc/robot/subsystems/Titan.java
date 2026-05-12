package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Titan extends SubsystemBase {
    
    public TitanQuad leftMotor;
    public TitanQuad rightMotor;
    public TitanQuad backMotor;
    
    public TitanQuadEncoder enc_left, enc_right, enc_back;
    
    private AHRS navX;

    private double ax = 0, ay = 0;
    private double prevEncR = 0, prevEncL = 0, prevEncB = 0;
    private final double[] angles = {30, 150, 270};

    public boolean stopped = true;
    private float curYaw = 0;
    private boolean holdYaw = false;
    private float needYaw = 0, lastNeedYaw = 0;
    
    public Titan(int titanId, int leftPort, int rightPort, int backPort, double distPerTick){
        leftMotor = new TitanQuad(titanId, leftPort);
        rightMotor = new TitanQuad(titanId, rightPort);
        backMotor = new TitanQuad(titanId, backPort);

        enc_left = new TitanQuadEncoder(leftMotor, leftPort, distPerTick);
        enc_right = new TitanQuadEncoder(rightMotor, rightPort, distPerTick);
        enc_back = new TitanQuadEncoder(backMotor, backPort, distPerTick);

        navX = new AHRS(SPI.Port.kMXP);
    }

    public void setHoldYaw(boolean value){holdYaw = value;}
    public void toogleHoldYaw() {setHoldYaw(!holdYaw);}
    public boolean getHoldYaw() {return holdYaw;}

    public float getYaw() {return curYaw;}
    public float getNeedYaw() {return needYaw;}
    public float getLastNeedYaw() {return lastNeedYaw;}
    public float getDifferenceYaw() {return normalizeYaw(needYaw - curYaw);}
    public float getDifferenceLastYaw() {return normalizeYaw(lastNeedYaw - curYaw);}
    public boolean isDifferenceZero() {return Math.abs(getDifferenceYaw())<0.55;}
    public float normalizeYaw(float yaw) {
        while (yaw>180 || yaw<-180) {yaw += yaw<180?360:-360;}
        return yaw;
    }
    
    public void drivePID(double y, double x, double rotate) {
        double l = Math.toRadians(300+getYaw());
        double r = Math.toRadians(60+getYaw());
        double b = Math.toRadians(180+getYaw());
        setWheelsSpeed(
            x * Math.cos(l) - y * Math.sin(l),
            x * Math.cos(r) - y * Math.sin(r),
            x * Math.cos(b) - y * Math.sin(b),
            rotate
        );
    } 

    public void setWheelsSpeed(double L, double R, double B, double rotate) {
        if (holdYaw) {
            float diff = getDifferenceYaw();
            float correction = (float)Math.min(Math.max(Math.signum(diff)*(Math.abs(diff)/30f + 0.08f), -0.2f), 0.2f);
            if (Math.abs(correction) > 0.05f) rotate += correction;
        }
    
        L += rotate; R += rotate; B += rotate;
    
        double max = Math.max(Math.abs(L), Math.max(Math.abs(R), Math.abs(B)));
        if (max > 1) { L /= max; R /= max; B /= max; }
    
        leftMotor.set(L); 
        rightMotor.set(R); 
        backMotor.set(B);
        stopped = false;
    }
    
    public void stopWheels() {
        leftMotor.set(0);
        rightMotor.set(0);
        backMotor.set(0);
        stopped = true;
    }

    public void setWheelEncZero() {
        enc_left.reset(); enc_right.reset(); enc_back.reset();
        prevEncL = prevEncR = prevEncB = 0; ax = ay = 0;
    }

    public double getAx() { return ax; }
    public double getAy() { return ay; }

    public void resetYaw() {
        navX.zeroYaw();
        Timer.delay(0.1);
    }

    @Override
    public void periodic() {
        curYaw = navX.getYaw();

        double curR = enc_right.getEncoderDistance(), deltaR = curR - prevEncR; prevEncR = curR;
        double curL = enc_left.getEncoderDistance(), deltaL = curL - prevEncL; prevEncL = curL;
        double curB = enc_back.getEncoderDistance(), deltaB = curB - prevEncB; prevEncB = curB;

        double[] deltas = {deltaR, deltaL, deltaB};

        for (int i=0; i<3; i++) {
            double rad = Math.toRadians(angles[i] - curYaw);
            ax += Math.sin(rad) * deltas[i] * 2/3;
            ay += -Math.cos(rad) * deltas[i] * 2/3;
        }
    }
}