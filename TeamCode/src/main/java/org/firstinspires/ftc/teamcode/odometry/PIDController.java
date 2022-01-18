package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double startTime;
    private double dt;

    private double p;
    private double i;
    private double d;

    private double prevInput;

    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        p = 0;
        i = 0;
        d = 0;

        prevInput = 0;
    }

    public void start(){
        startTime = System.currentTimeMillis();
    }

    public void reset(){
        p = 0;
        i = 0;
        d = 0;
    }

    public double getOutput(double input){
        p = input;

        dt = System.currentTimeMillis() - startTime;
        startTime = System.currentTimeMillis();

        // This is technically an LRAM
        i += input * dt;

        try{
            d = (input - prevInput) / dt;
        } catch(ArithmeticException e){
            d = 0;
        }

        prevInput = input;

        return (kP * p) + (kI * i) + (kD * d);
    }
}