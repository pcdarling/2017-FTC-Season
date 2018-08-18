package org.firstinspires.ftc.teamcode;

/**
 * Created by Prestolie on 11/2/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp(name="Elevator", group="TeamBot")
public class Elevator extends LinearOpMode {

    public ElevatorHardware robot = new ElevatorHardware();

    // Preference Constants
    double STANDARD_ELEVATOR_SPEED = 0.15;
    double INC_SERVO = 0.1;
    int OPEN_CLEFT = 255;
    int OPEN_CRIGHT = 0;
    int CLOSE_CLEFT = 55; // Maybe 60?
    int CLOSE_CRIGHT = 150;
    int BASE_ELEVATOR = 0;
    int INC_ELEVATOR = 1;

    // Measured Constants
    int MAX_ELEVATOR = 2200;

    // Hardware Constants
    int DRIVE_GEAR_REDUCTION = 30;
    int PULSES_PER_REV = 28;
    double DEGREES_PER_COUNT = 0.7059;

    // Calculated Constants
    int COUNT_PER_REV = PULSES_PER_REV * DRIVE_GEAR_REDUCTION; // specifically for 40:1
    int COUNTS_TO_NEXT_MODE = (int)Math.ceil(MAX_ELEVATOR/3);


    // State of machine
    int elevatorMode = 0;
    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        setupRobot();

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a) {
                // Go Down
                raiseLower(1,STANDARD_ELEVATOR_SPEED);
            }
            if (gamepad1.y) {
                // Go Up
                raiseLower(0,STANDARD_ELEVATOR_SPEED);
            }
            if (gamepad1.b) {
                // Close
                openClose(2,INC_SERVO);
            }
            if (gamepad1.x) {
                // Open
                openClose(3,INC_SERVO);
            }
            // Wait for instructions
            idle();
        }

    }

    public void raiseLower(int decision, double speed) {
        int cur = robot.Elevator.getCurrentPosition();
        int NewPos;

        if (decision == 0 || decision == 1 || decision == 2 && elevatorMode < 0) {
            // Moving from manual to mode select (simple---Go Home first)
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            elevatorMode = 0;
            NewPos = 0;
        }
        else if (decision == 0 && elevatorMode < 2 && elevatorMode >= 0 ){
            // set the elevator to move upward
            robot.Elevator.setDirection(DcMotor.Direction.FORWARD);
            NewPos = cur+COUNTS_TO_NEXT_MODE;
            elevatorMode++;
        }
        else if (decision == 1 && elevatorMode  > 0){
            //sets the elevator to move downward
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            NewPos = cur+COUNTS_TO_NEXT_MODE;
            elevatorMode--;
        }
        else if (decision == 2){
            robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            NewPos = 0;
            elevatorMode = 0;
        }
        else{
            elevatorMode = -1;
            if(speed < 0){
                robot.Elevator.setDirection(DcMotor.Direction.REVERSE);
            }
            else {
                robot.Elevator.setDirection(DcMotor.Direction.FORWARD);
            }
            NewPos = robot.Elevator.getCurrentPosition() + INC_ELEVATOR;

        }

        robot.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        robot.Elevator.setPower(speed);

        while(opModeIsActive() && robot.Elevator.isBusy()){
            //busy waiting
            telemetry.addData("Elevator Status:", "Rotating");
        }

        robot.Elevator.setPower(0);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openClose(int decision, double speed) {
        if (decision == 0){
            //does incremental close
            double curLeft = robot.Cleft.getPosition();
            double curRight = robot.Cright.getPosition();
            robot.Cleft.setPosition(curLeft - INC_SERVO);
            robot.Cright.setPosition(curRight + INC_SERVO);
        }
        else if (decision == 1){
            //does incremental open
            double curLeft = robot.Cleft.getPosition();
            double curRight = robot.Cright.getPosition();
            robot.Cleft.setPosition(curLeft + INC_SERVO);
            robot.Cright.setPosition(curRight - INC_SERVO);
        }
        else if (decision == 2){
            //does closes
            robot.Cleft.setPosition(CLOSE_CLEFT);
            robot.Cright.setPosition(CLOSE_CRIGHT);
        }
        else {
            //does opens
            robot.Cleft.setPosition(OPEN_CLEFT);
            robot.Cright.setPosition(OPEN_CRIGHT);
        }
    }

    public void setupRobot() {
        robot.Cleft.setPosition(OPEN_CLEFT);
        robot.Cright.setPosition(OPEN_CRIGHT);
    }

    public void shutdownRobot() {
        // Open the claw
        robot.Cleft.setPosition(OPEN_CLEFT*DEGREES_PER_COUNT);
        robot.Cright.setPosition(CLOSE_CRIGHT*DEGREES_PER_COUNT);

        // Fully retract the Elevator
        raiseLower(2,STANDARD_ELEVATOR_SPEED);

    }

}
