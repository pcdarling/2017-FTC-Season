package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shelby Goss on 9/28/2017.
 */

public class CompetitionHardware {

    public DcMotor FLmotor= null;
    public DcMotor FRmotor = null;
    public DcMotor RLmotor = null;
    public DcMotor RRmotor = null;
    public DcMotor Shoulder = null;
    //public DcMotor Extender = null;
    public DcMotor Elevator = null;
    public Servo Cleft = null;
    public Servo Cright = null;
    //public Servo Nabber = null;
    //public Servo Stick = null;
    //public Servo Flick = null;
    //public ColorSensor  Gem = null;

    HardwareMap hwMap = null;

    public CompetitionHardware() { // Constructor
    }

    public void init(HardwareMap ahwMap){

        //save reference to hwMap
        hwMap = ahwMap;
        FLmotor = hwMap.get(DcMotor.class, "FL");
        FRmotor = hwMap.get(DcMotor.class, "FR");
        RLmotor = hwMap.get(DcMotor.class, "RL");
        RRmotor = hwMap.get(DcMotor.class, "RR");
        Elevator = hwMap.get(DcMotor.class, "elevator");
        Shoulder = hwMap.get(DcMotor.class, "shoulder");
        //Extender = hwMap.get(DcMotor.class, "extender");
        //Nabber = hwMap.get(Servo.class, "nabber");
        Cleft = hwMap.get(Servo.class, "cleft");
        Cright = hwMap.get(Servo.class, "cright");
        //Stick = hwMap.get(Servo.class, "dat $tick");
        //Gem = hwMap.get(ColorSensor.class, "gem");
        //Flick = hwMap.get(Servo.class, "flick");

        FLmotor.setPower(0);
        FRmotor.setPower(0);
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        Elevator.setPower(0);
        Shoulder.setPower(0);
        //Extender.setPower(0);

        Shoulder.setDirection(DcMotor.Direction.FORWARD);
        //Extender.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.FORWARD);

        FLmotor.setDirection(DcMotor.Direction.FORWARD);
        FRmotor.setDirection(DcMotor.Direction.FORWARD);
        RLmotor.setDirection(DcMotor.Direction.FORWARD);
        RRmotor.setDirection(DcMotor.Direction.FORWARD);

        FLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
