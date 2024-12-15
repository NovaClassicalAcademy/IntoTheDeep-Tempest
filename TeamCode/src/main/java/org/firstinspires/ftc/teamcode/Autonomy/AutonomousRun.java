
package org.firstinspires.ftc.teamcode.Autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "ParkAutoTestRun", group = "Autonomous")


public class AutonomousRun extends LinearOpMode {

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void runOpMode() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initializing robot");
        Robot myRobot = new Robot(this);
        telemetry.addData("Status", "Robot initialized");

        waitForStart();
//
        telemetry.addData("Lift Position", myRobot.GetLiftCurrentPosition());
        telemetry.update();

        sleep();
//        //TODO: can't stop by itself. Maybe get new encoder to read left to right. might need to be timed.
//        myRobot.Strafe(15);
//        sleep(1000);
//        myRobot.Strafe(-15);
        myRobot.LiftLift(1250);
        sleep(5000);
        myRobot.LowerLift(-1250);
//        myRobot.Turn(5);
//        myRobot.Claw();
//        myRobot.Gripper();
//        myRobot.Dump();


//
//        //positive is going forward, negative is going back. might need to change powers. right is lighter than left side.
//        myRobot.Move(-20);

//        //first element deposit after start
//
//
//        //second element pick up and dump
//
//
//        //park in observation zone

    }
}