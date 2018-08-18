package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ElevatorHardware {

    public DcMotor Elevator = null;
    public Servo Cleft = null;
    public Servo Cright = null;

    HardwareMap hwMap = null;

    public ElevatorHardware() { // Constructor
    }

    public void init(HardwareMap ahwMap){
        //save reference to hwMap
        hwMap = ahwMap;

        Elevator = hwMap.dcMotor.get("elevator");
        Cleft = hwMap.servo.get("cleft");
        Cright = hwMap.servo.get("cright");

        Elevator.setDirection(DcMotor.Direction.FORWARD);

        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Elevator.setPower(0);
    }
}

