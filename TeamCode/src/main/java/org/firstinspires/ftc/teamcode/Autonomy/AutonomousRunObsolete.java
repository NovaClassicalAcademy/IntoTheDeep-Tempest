//
//package org.firstinspires.ftc.teamcode.Autonomy;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name = "ParkAutoTestRun", group = "Autonomous")
//
//public class AutonomousRunObsolete extends LinearOpMode {
////    private void TestLandD (Robot myRobot){
//        myRobot.MoveLift(2900, 3);
//        sleep(2000);
//        myRobot.Dump();
//        sleep(2000);
////        myRobot.RetractDump();
////        sleep(2000);
////        myRobot.MoveLift(10, 3);
////    }
//    private void HighBucket (Robot myRobot){ //fail
//        ///try this out. i think this is the plan that will score in high bucket.
//
////        //step 1: given high bucket
////        myRobot.Move(-1.5); //2 is too big--will go to wall
////        myRobot.Turn(45); //-x is turing left???
////        myRobot.Move(-20);
////        myRobot.Turn(-90);
//////        myRobot.ExtendLift(2900, 3);
////        myRobot.Move(-10);
////        myRobot.Dump();
//        sleep(1000);
////        myRobot.RetractDump();
//        myRobot.Move(10);
////        myRobot.LowerLift(1,3);
//
//        //step 2: furthest element from the wall on the ground score
//        myRobot.Turn(-90);
//        myRobot.Move(3);
//        myRobot.Turn(45);
//        myRobot.LowerHinge();
//        myRobot.Move(3);
//        myRobot.CloseClaw();
//        myRobot.LiftHinge();
//        myRobot.Move(-3);
//        myRobot.Turn(-45);
//        myRobot.Move(-3);
//        myRobot.Turn(90);
//        myRobot.OpenClaw();
//        sleep(1000);
////        myRobot.ExtendLift(2900, 3);
//        myRobot.Move(-10);
////        myRobot.Dump();
//        sleep(1000);
////        myRobot.RetractDump();
//        myRobot.Move(10);
////        myRobot.LowerLift(1,3);
//
//        //step 3: middle element on the ground score
//        myRobot.Turn(-90);
//        myRobot.Move(5);  //TODO: play around with this number
//        myRobot.Turn(45);
//        myRobot.LowerHinge();
//        myRobot.Move(5);
//        myRobot.CloseClaw();
//        myRobot.LiftHinge();
//        myRobot.Move(-3);
//        myRobot.Turn(-45);
//        myRobot.Move(-3);
//        myRobot.Turn(90);
//        myRobot.OpenClaw();
//        sleep(1000);
////        myRobot.ExtendLift(2900, 3);
//        myRobot.Move(-10);
////        myRobot.Dump();
//        sleep(1000);
////        myRobot.RetractDump();
//        myRobot.Move(10);
////        myRobot.LowerLift(1,3);
//
//    }
////    private void Specimen(Robot myRobot){
////
////        ///this is plan for specimen
////        myRobot.Turn(45);
////        myRobot.Move(30);
////        myRobot.Turn(90);
////        myRobot.Move(10);
////        myRobot.Turn(90);
////        myRobot.Move(30);
////        myRobot.Turn(90);
////
////    }
////
////    /* Code to run ONCE when the driver hits INIT */
////    @Override
////    public void runOpMode() {
////
////        // Tell the driver that initialization is complete.
////        telemetry.addData("Status", "Initializing robot");
////        Robot myRobot = new Robot(this);
////        telemetry.addData("Status", "Robot initialized");
////
////        waitForStart();
//////
//////        TestLandD(myRobot);
//////        myRobot.Turn(-45);
//////        sleep(2000);
//////        myRobot.Turn(45);
////        myRobot.LowerHinge();
////        myRobot.LiftHinge();
////        myRobot.OpenClaw();
////        myRobot.CloseClaw();
////    }
//}