package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Shelby Goss on 9/28/2017.
 */

public class DrivetrainHardware {

    public DcMotor FLmotor = null;
    public DcMotor FRmotor = null;
    public DcMotor RLmotor = null;
    public DcMotor RRmotor = null;

    HardwareMap hwMap = null;

    public DrivetrainHardware() { // Constructor
    }

    public void init(HardwareMap ahwMap){

        //save reference to hwMap
        hwMap = ahwMap;
        FLmotor = hwMap.dcMotor.get("FL");
        FRmotor = hwMap.dcMotor.get("FR");
        RLmotor = hwMap.dcMotor.get("RL");
        RRmotor = hwMap.dcMotor.get("RR");

        FLmotor.setDirection(DcMotor.Direction.FORWARD);
        FRmotor.setDirection(DcMotor.Direction.FORWARD);
        RLmotor.setDirection(DcMotor.Direction.FORWARD);
        RRmotor.setDirection(DcMotor.Direction.FORWARD);

        FLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);






    }
}
