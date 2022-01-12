package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.supers.Direction;
import org.firstinspires.ftc.teamcode.supers.Robot;

public class Odometry {
    private Robot r;

    // backend mouse stuff + constants
    private final int dpi = 1000;
    private final int fieldLength = 144000;
    private final double trackWidth = 10.0; // TODO: measure actual track width and offset
    private final double horizOffset = 10.0;

    // turning vars
    public double angleError;
    // TODO: set threshold + tuning
    private double turnThreshold = 1.0;
    private PIDController turnPid = new PIDController(0.015, 0.0, 0.0);

    // odo vars
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentAngle = Math.PI / 2.0; // IN RADIANS
    private double coordError;
    // TODO: set threshold + tuning
    private double coordThreshold = 0b110010000; // 0.4 in
    private PIDController posPid = new PIDController(0, 0, 0);

    public Odometry(Robot robot){
        this.r = robot;
    }

    public void setVelocity(double forward, double clockwise){
        // TODO: make sure directions are correct
        double right = forward - clockwise;
        double left = forward + clockwise;
        double max = Math.max(Math.max(Math.abs(left), Math.abs(right)), 1.0);

        r.lf.setPower(left / max);
        r.lb.setPower(left / max);
        r.rf.setPower(right / max);
        r.rb.setPower(right / max);
    }

    public void turnTo(double angle) {
        // Convert angle to radians
        angle = angle * 180.0 / Math.PI;

        // counter to keep track of consecutive times robot was within threshold
        int counter = 0;

        turnPid.start();
        angleError = getError(angle, currentAngle);

        while (r.opMode.isStarted() && !r.opMode.isStopRequested()) {
            // Threshold check
            if (counter >= 2) break;

            if (Math.abs(angleError) < turnThreshold){
                counter++;
                setVelocity(0, 0);
            }
            // Reset threshold counter (i.e. overshoot)
            else{
                counter = 0;
                // Set motor velocities with controller output
                setVelocity(0, turnPid.getOutput(angleError));
            }

            r.opMode.telemetry.addData("Current error: ", angleError);
            r.opMode.telemetry.addData("Current angle: ", currentAngle);
            r.opMode.telemetry.addData("Target: ", angle);
            r.opMode.telemetry.update();

            // Update error using normalized angle
            angleError = getError(angle, currentAngle);

            // Call an update for angle and position
            update();
        }

        setVelocity(0, 0); // stop motion

        turnPid.reset();

        r.opMode.telemetry.addData("Current error: ", angleError);
        r.opMode.telemetry.addData("Current angle: ", currentAngle);
        r.opMode.telemetry.addData("Target: ", angle);
        r.opMode.telemetry.update();
    }
    
    public double normalize(double angle){
        angle = angle % 360;
        if(angle < 0) return angle + 360;
        else return angle;
    }

    public double getError(double target, double actual){
        return actual - target;
    }

    public void update() {
        // Calculating angular change
        // TODO: make sure these are the right motors
        int d1 = r.rwheel.getCurrentPosition(); // x wheel 1
        int d2 = r.lwheel.getCurrentPosition(); // x wheel 2
        int d3 = r.carousel.getCurrentPosition(); // y wheel
        double deltaAngle = d1 + d2 / trackWidth;

        // Calculating translational change
        double cDX = d1 + d2 / 2.0;
        double cDY = d3 - (deltaAngle * horizOffset);

        // Update angle and position (pose)
        currentAngle += deltaAngle;

        double deltaX = cDX * Math.sin(currentAngle) + cDY * Math.cos(currentAngle);
        double deltaY = cDX * Math.cos(currentAngle) + cDY * Math.sin(currentAngle);

        currentX += deltaX;
        currentY += deltaY;
    }

    // TODO: make it not deltas
    public void drive(double deltaX, double deltaY){

        // get the angle to turn for aligning with the hypotenuse from inverse tan, in radians
        double targetAngle = Math.atan2(deltaY, deltaX);

        // optimize the angle being passed into turnTo() (positive angle vs. negative counterpart, finds whichever is closest)
        if(targetAngle < 0){
            if(Math.abs(targetAngle - currentAngle) > Math.abs((targetAngle + (2*Math.PI)) - currentAngle)) targetAngle += (2*Math.PI);
        }
        else{
            if(Math.abs(targetAngle - currentAngle) > Math.abs((targetAngle - (2*Math.PI)) - currentAngle)) targetAngle -= (2*Math.PI);
        }

        // align with hypotenuse
        turnTo(targetAngle);


        // get the target distance as the current "y" value plus the hypotenuse of the desired change in coordinates
//        double targetDist = currentY + Math.hypot(Math.abs(deltaX), Math.abs(deltaY)); // irrelevant now?

        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;

        double lf;
        double lb;
        double rf;
        double rb;
        double coordOut;
        double curAng;

        int counter = 0;

        // TODO: eventually add maintaining of angle (see PushbotAutoDriveByGyro)

        posPid.start();

        coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
        double angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - Math.PI / 4 - Math.toRadians(currentAngle);

        while(r.opMode.opModeIsActive()){
//            setVelocity(posPid.getOutput(coordError), 0);
            if(Math.abs(coordError) < coordThreshold) counter++;
            else counter = 0;

            if(counter >= 2) break;

            coordOut = posPid.getOutput(coordError);

            lf = coordOut * Math.sin(angleToTarget);
            lb = coordOut * Math.cos(angleToTarget);
            rf = coordOut * Math.cos(angleToTarget);
            rb = coordOut * Math.sin(angleToTarget);

            r.lf.setPower(lf);
            r.lb.setPower(lb);
            r.rf.setPower(rf);
            r.rb.setPower(rb);

            // update coordinates
            update();
            curAng = Math.toRadians(currentAngle);

            coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
            // subtracts the current angle for field-centric
            angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - Math.PI / 4 - curAng;
        }
        setVelocity(0, 0);

        posPid.reset();
    }

    public void resetEncoders(){
        r.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        r.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(Direction direction, double distance, double speed){

        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;

        double wheelDiam = 4.0;
        double ticksPerRev = 537.6;
        double inchesPerRev = wheelDiam * Math.PI;
        double ticksPerInch = ticksPerRev/inchesPerRev;

        distance *= ticksPerInch;

        switch(direction){
            case FORWARD:
                newLeftFrontTarget = r.lf.getCurrentPosition() + (int) distance;
                newLeftBackTarget = r.lb.getCurrentPosition() + (int) distance;
                newRightFrontTarget = r.rf.getCurrentPosition() + (int) distance;
                newRightBackTarget = r.rb.getCurrentPosition() + (int) distance;
                break;
            case BACK:
                newLeftFrontTarget = r.lf.getCurrentPosition() - (int) distance;
                newLeftBackTarget = r.lb.getCurrentPosition() - (int) distance;
                newRightFrontTarget = r.rf.getCurrentPosition() - (int) distance;
                newRightBackTarget = r.rb.getCurrentPosition() - (int) distance;
                break;
            case LEFT:
                newLeftFrontTarget = r.lf.getCurrentPosition() - (int) distance;
                newLeftBackTarget = r.lb.getCurrentPosition() + (int) distance;
                newRightFrontTarget = r.rf.getCurrentPosition() + (int) distance;
                newRightBackTarget = r.rb.getCurrentPosition() - (int) distance;
                break;
            case RIGHT:
                newLeftFrontTarget = r.lf.getCurrentPosition() + (int) distance;
                newLeftBackTarget = r.lb.getCurrentPosition() - (int) distance;
                newRightFrontTarget = r.rf.getCurrentPosition() - (int) distance;
                newRightBackTarget = r.rb.getCurrentPosition() + (int) distance;
                break;
            default:
                return;
        }

        // Ensure that the OpMode is still active
        if (r.opMode.opModeIsActive()) {
            r.lf.setTargetPosition(newLeftFrontTarget);
            r.lb.setTargetPosition(newLeftBackTarget);
            r.rf.setTargetPosition(newRightFrontTarget);
            r.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            r.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset timer and begin to run the motors
            if(direction == Direction.LEFT || direction == Direction.RIGHT){
                r.lf.setPower(Math.abs(speed));
                r.lb.setPower(Math.abs(speed));
                r.rf.setPower(Math.abs(speed));
                r.rb.setPower(Math.abs(speed));
            }
            else {
                r.lf.setPower(Math.abs(speed));
                r.rf.setPower(Math.abs(speed));
                r.lb.setPower(Math.abs(speed));
                r.rb.setPower(Math.abs(speed));
            }

            // Keep looping until the motor is at the desired position that was inputted
            while (r.opMode.opModeIsActive() &&
                    (r.lf.isBusy() || r.lb.isBusy() || r.rf.isBusy() || r.rb.isBusy())) {

                // Display current status of motor paths
                r.opMode.telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                r.opMode.telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", r.lf.getCurrentPosition(), r.lb.getCurrentPosition(), r.rf.getCurrentPosition(), r.rb.getCurrentPosition());
                r.opMode.telemetry.addData("right back", r.rb.getPower());
                r.opMode.telemetry.addData("right front", r.rf.getPower());
                r.opMode.telemetry.addData("left back", r.lb.getPower());
                r.opMode.telemetry.addData("left front", r.lf.getPower());
                r.opMode.telemetry.update();
            }

            // Stop all motion
            if(direction == Direction.LEFT || direction == Direction.RIGHT) {
                r.lf.setPower(0);
                r.lb.setPower(0);
                r.rf.setPower(0);
                r.rb.setPower(0);
            }
            else {
                r.lf.setPower(0);
                r.rf.setPower(0);
                r.lb.setPower(0);
                r.rb.setPower(0);
            }
        }
    }
}