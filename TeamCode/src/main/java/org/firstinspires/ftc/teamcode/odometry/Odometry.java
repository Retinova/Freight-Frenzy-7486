package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.supers.ConfigVars;
import org.firstinspires.ftc.teamcode.supers.Direction;
import org.firstinspires.ftc.teamcode.supers.Robot;

import java.util.Arrays;
public class Odometry {
    private Robot r;

    // constants for odometry
    private final double wheelDiameter = 0.0;
    private final double ticksPerRev = 360.0;
    private final double gearRatio = 1.0;
    private final double trackWidth = 10.0; // TODO: measure actual track width and offset
    private final double horizOffset = 10.0;

    // turning vars
    public double angleError;
    // TODO: set threshold + tuning
    private double turnThreshold = 1.0;
    private PIDController turnPid = new PIDController(ConfigVars.turnP, ConfigVars.turnI, ConfigVars.turnD);

    // odo vars
    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentAngle = Math.PI / 2.0; // IN RADIANS
    private double coordError;
    // TODO: set threshold + tuning
    private double coordThreshold = 0b110010000; // 0.4 in
    private PIDController posPid = new PIDController(ConfigVars.posP, ConfigVars.posI, ConfigVars.posD);

    public Odometry(Robot robot){
        this.r = robot;
    }

    public void setVelocity(double forward, double lateral, double clockwise){
        // TODO: make sure directions are correct
        double lfp, lbp, rfp, rbp;
        rfp = -forward + lateral - clockwise;
        rbp = -forward - lateral - clockwise;
        lfp = -forward - lateral + clockwise;
        lbp = -forward + lateral + clockwise;

        // Sorts to find power of highest magnitude
        double[] powers = {Math.abs(lfp), Math.abs(lbp), Math.abs(rfp), Math.abs(rbp), 1.0};
        Arrays.sort(powers);

        // Divide by the maximum to maintain ratios between wheel powers that would normally get
        // ruined by clipping between [-1.0, 1.0]
        r.lf.setPower(lfp / powers[4]);
        r.lb.setPower(lbp / powers[4]);
        r.rf.setPower(rfp / powers[4]);
        r.rb.setPower(rbp / powers[4]);
    }

    public void turnTo(double angle) {
        // Convert angle to radians
        angle = angle * 180.0 / Math.PI;

        // Counter to keep track of consecutive times robot was within threshold
        int counter = 0;

        // Record PID output at beginning of every cycle
        double angleOut;

        // Begin the PID controller's timer and initiate the loop with the initial error
        angleError = getError(angle, currentAngle);
        turnPid.start();

        while (r.opMode.isStarted() && !r.opMode.isStopRequested()) {
            // Threshold check
            if (counter >= 2) break;

            // Update PID regardless of threshold status to prevent inaccuracies
            angleOut = turnPid.getOutput(angleError);

            // Increment the counter by one if the robot falls within the threshold, indicating a potential end of the motion
            if (Math.abs(angleError) < turnThreshold){
                counter++;
                setVelocity(0, 0, 0);
            }
            // Reset threshold counter (i.e. overshoot has occurred or the motion is still active)
            else{
                counter = 0;
                // Set motor velocities with PID controller output
                setVelocity(0, 0, angleOut);
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

        // Officially end the motion
        setVelocity(0, 0, 0);

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
        // TODO: make sure this doesnt cause issues, switch to auto mode if needed
        r.bulkRead();

        // Calculating angular change
        // TODO: make sure these are the right motors
        int d1 = r.rwheel.getCurrentPosition(); // x wheel 1
        int d2 = r.lwheel.getCurrentPosition(); // x wheel 2
        int d3 = r.carousel.getCurrentPosition(); // y wheel
        double deltaAngle = d1 - d2 / trackWidth;

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
    public void drive(double deltaX, double deltaY, boolean turn){
        // Only do if turning is desired
        if(turn) {
            // Get the angle to turn for aligning with the hypotenuse from inverse tan, in radians
            double targetAngle = Math.atan2(deltaY, deltaX);

            // Optimize the angle being passed into turnTo() (positive angle vs. negative counterpart, finds whichever is closest)
            if (targetAngle < 0) {
                if (Math.abs(targetAngle - currentAngle) > Math.abs((targetAngle + (2 * Math.PI)) - currentAngle))
                    targetAngle += (2 * Math.PI);
            } else {
                if (Math.abs(targetAngle - currentAngle) > Math.abs((targetAngle - (2 * Math.PI)) - currentAngle))
                    targetAngle -= (2 * Math.PI);
            }

            // Align with hypotenuse
            turnTo(targetAngle);
        }

        // set up target coordinates
        double targetX = currentX + deltaX;
        double targetY = currentY + deltaY;

        // Record PID output at beginning of every cycle
        double coordOut;

        // Counter to keep track of consecutive times robot was within threshold
        int counter = 0;

        // TODO: eventually add maintaining of angle (see PushbotAutoDriveByGyro)

        // Calculate the coordinate error as a resultant of the x and y error "vectors".
        // The corresponding output from the PID controller is also treated as a "scaled" resultant
        // and broken up into its x and y components as the lateral and forward velocities, respectively.
        // This works since we need to form the two error measures into one input for the PID controller,
        // and the angle to the target coordinates is a definite value that we calculate every cycle.
        // The only thing that then matters is the "speed" at which we approach the target coordinate,
        // the direction is taken care of by the calculated angle.
        coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
        double angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - currentAngle;

        // Begin the PID controller's timer and initiate the loop with the initial error
        posPid.start();

        while(r.opMode.isStarted() && !r.opMode.isStopRequested()){
            // Threshold check
            if(counter >= 2) break;

            // Update PID regardless of threshold status to prevent inaccuracies
            coordOut = posPid.getOutput(coordError);

            // Increment the counter by one if the robot falls within the threshold, indicating a potential end of the motion
            if(Math.abs(coordError) < coordThreshold){
                counter++;
                setVelocity(0, 0, 0);
            }
            // Reset threshold counter (i.e. overshoot has occurred or the motion is still active)
            else{
                counter = 0;
                // Set motor velocities with controller output
                setVelocity(coordOut * Math.sin(angleToTarget), coordOut * Math.cos(angleToTarget), 0);
            }

            // Update coordinates
            update();
            coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
            // Subtracts the current angle for field-centric
            angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - currentAngle;
        }

        // Officially end the motion
        setVelocity(0, 0, 0);

        posPid.reset();
    }

    // Overload of normal drive with default to no turning
    public void drive(double deltaX, double deltaY){
        drive(deltaX, deltaY, false);
    }

    public void drivePath(double[][] targetSet){
//        if(targetSet[0].length != 2) throw new InterruptedException("Invalid coordinate array");

        // Set first target to first coordinate
        double targetX = targetSet[0][0];
        double targetY = targetSet[0][1];
        double coordOut;
        // Start the threshold at a larger value so that we just generally follow the path
        double currentThreshhold = 50; // TODO: setup actual threshholds

        // The counter here is different, here it is counting which coordinate we are at
        int counter = 0;


        // Begin the PID controller's timer and initiate the loop with the initial error
        coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
        double angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - currentAngle;
        posPid.start();

        // This loop is now set up to end once the last coordinate is reaches
        while(r.opMode.isStarted() && !r.opMode.isStopRequested() && counter <= targetSet.length-1){
            // Update PID regardless of threshold status to prevent inaccuracies
            coordOut = posPid.getOutput(coordError);

            // Increment the counter by one if the robot falls within the threshold, indicating a potential end of the motion
            if(Math.abs(coordError) < currentThreshhold){
                if(++counter == targetSet.length-1){
                    currentThreshhold = coordThreshold;
                }

                if(counter != targetSet.length) {
                    targetX = targetSet[counter][0];
                    targetY = targetSet[counter][1];
                    posPid.reset();
                }
                else break;

            }
            // Reset threshold counter (i.e. overshoot has occurred or the motion is still active)
            else{
                // Set motor velocities with controller output
                setVelocity(coordOut * Math.sin(angleToTarget), coordOut * Math.cos(angleToTarget), 0);
            }

            // Update coordinates
            update();
            coordError = Math.hypot(getError(targetX, currentX), getError(targetY, currentY));
            // Subtracts the current angle for field-centric
            angleToTarget = Math.atan2(getError(targetY, currentY), getError(targetX, currentX)) - currentAngle;
        }

        // Officially end the motion
        setVelocity(0, 0, 0);

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

    public double[] getCoords(){
        double inchesPerTick = wheelDiameter * Math.PI / (ticksPerRev * gearRatio);
        return new double[]{currentX * inchesPerTick, currentY * inchesPerTick};
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

                // TODO: make sure this doesnt cause issues, switch to auto mode if needed
                r.bulkRead();

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



























/*
 _____  _____ _   _ ___________   _____   ___  ___  ___ _____ _____   _
/  ___||  _  | | | |_   _|  _  \ |  __ \ / _ \ |  \/  ||  ___/  ___| | |
\ `--. | | | | | | | | | | | | | | |  \// /_\ \| .  . || |__ \ `--.  | |
 `--. \| | | | | | | | | | | | | | | __ |  _  || |\/| ||  __| `--. \ | |
/\__/ /\ \/' / |_| |_| |_| |/ /  | |_\ \| | | || |  | || |___/\__/ / |_|
\____/  \_/\_\\___/ \___/|___/    \____/\_| |_/\_|  |_/\____/\____/  (_)

and they dropped the robot
 */