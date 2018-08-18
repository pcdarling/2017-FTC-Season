package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Shelby Goss on 9/28/2017.
 */

public class CompetitionHardware2 {

    public DcMotor FLmotor= null;
    public DcMotor FRmotor = null;
    public DcMotor RLmotor = null;
    public DcMotor RRmotor = null;
    public DcMotor Shoulder = null;
    public DcMotor Elevator = null;
    public DcMotor Cleft = null;
    public DcMotor Cright = null;
    public Servo gemX = null;
    public Servo gemY = null;
    public ColorSensor  Gem = null;
    public TouchSensor tLeft = null;
    public TouchSensor tRight = null;
    public TouchSensor tBot = null;

    HardwareMap hwMap = null;

    // states for robot
    int elevatorMode = 0;
    int vuMarkID = 0;

    public CompetitionHardware2() { // Constructor
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
        Cright = hwMap.get(DcMotor.class, "cright");
        Cleft = hwMap.get(DcMotor.class, "cleft");
        gemX = hwMap.get(Servo.class, "gem x");
        gemY = hwMap.get(Servo.class, "gem y");
        Gem = hwMap.get(ColorSensor.class, "gem");
        tLeft = hwMap.get(TouchSensor.class,"tleft");
        tRight = hwMap.get(TouchSensor.class,"tright");
        tBot = hwMap.get(TouchSensor.class, "tbot");

        FLmotor.setPower(0);
        FRmotor.setPower(0);
        RLmotor.setPower(0);
        RRmotor.setPower(0);
        Elevator.setPower(0);
        Shoulder.setPower(0);

        Shoulder.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.FORWARD);
        Cright.setDirection(DcMotor.Direction.FORWARD);
        Cleft.setDirection(DcMotor.Direction.FORWARD);

        FLmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FRmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RLmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RRmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Cright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Cleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((SwitchableLight)Gem).enableLight(true);

        Gem.setI2cAddress(I2cAddr.create8bit(0x3a));

    }
}
