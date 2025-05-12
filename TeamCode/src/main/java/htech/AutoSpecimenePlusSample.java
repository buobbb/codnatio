package htech;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import htech.subsystem.ExtendoSystem;
import htech.subsystem.IntakeSubsystem;
import htech.subsystem.LiftSystem;
import htech.subsystem.OuttakeSubsystem;
import htech.subsystem.RobotSystems;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "[AUTO] 5 + 1", group = "HTECH")
public class AutoSpecimenePlusSample extends LinearOpMode {

    //Mechanisms
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    LiftSystem lift;
    ExtendoSystem extendo;
    RobotSystems robotSystems;
    ElapsedTime timer;
    ElapsedTime matchTimer;


    //Pedro
    Follower follower;
    Path preload;
    PathChain collectSamples, failSafe1, failSafe;
    Path score1, score2, score3, score4;
    PathChain wall;
    Path park;
    Path collectSampleBasket, goToBasket;


    //States
    public enum STATES{
        IDLE,
        SPECIMEN, SCORING_SPECIMEN,
        COLLECTING_SAMPLES, COLLECTING_SPECIMEN,
        SCORE,
        WALL,
        PARK,
        PARKED,
        MOVING, WAITING, TRANSFER,
        CHECK_SPECIMEN, FAIL_SAFE,
        CHECKPOINT, WALL1,
        COLLECTING_SAMPLE,
        COLLECTING_SAMPLE1,
        COLLECTING_SAMPLE2, COLLECTING_SAMPLE3,
        SCORE_BASKET, SCORE_BASKET2, SCORE_BASKET3
    }
    public enum SCORING_STATES{
        IDLE,
        SCORE1,
        SCORE2,
        SCORE3,
        SCORE4
    }
    STATES CS = STATES.IDLE, NS = STATES.IDLE;
    SCORING_STATES SCORING_CS = SCORING_STATES.IDLE;


    //Coordinates
    public static double startX = 0, startY = 0, startH = 180;
    public static double preloadX = -30.5, preloadY = -5.5, preloadH = startH;

    public static double safe1Sample1X = 0, safe1Sample1Y = 32;
    public static double safe2Sample1X = -30, safe2Sample1Y = 15;
    public static double safe3Sample1X = -60, safe3Sample1Y = 37;
    public static double sample1X = -48, sample1Y = 37, sample1H = 180;
    public static double human1X = -23, human1Y = 37, human1H = 180;

    public static double safeSample2X = -48, safeSample2Y = 30;
    public static double sample2X = -47, sample2Y = 45, sample2H = 180;
    public static double human2X = -23, human2Y = 45, human2H = 180;

    public static double safeSample3X = -34, safeSample3Y = 34;
    public static double sample3X = -48, sample3Y = 51, sample3H = 180;
    public static double human3X = -3.5, human3Y = 50.5;
//    public static double

    public static double checkpointX = -23, checkpointY = 21, checkpointH = 180;

    public static double score1X = -31.5, score1Y = -2, scoreH = 180;
    public static double score2X = -31.5, score2Y = -4;
    public static double score3X = -31.5, score3Y = -7;
    public static double score4X = -32.1, score4Y = -9, score4H = 175;
    public static double safeScoreX = -14, safeScoreY = -7;

    public static double specimenX = -3.5, specimenY = 27.5, specimenH = 180;
    public static double safe1SpecimenX = -15, safe1SpecimenY = -5;
    public static double safe2SpecimenX = -15, safe2SpecimenY = 30;

    public static double parkX = -3, parkY = 30, parkH = 60;

    public static double sampleBasketX = -24, sampleBasketY = 20, sampleBasketH = 35;
    public static double basketX = -5, basketY = -56, basketH = 115;

    //Booleans
    boolean basket = false;
    boolean parking = false;
    boolean firstTime = true;

    //Timers
    public static double timeToCollect = 100;
    public static double matchTime = 30.6;
    public static double timeToGoDownSample = 200;
    public static double timeToCollectSample = 200;
    public double timeToWait = 0;


    //Speed
    public static double speed = 1;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        lift = new LiftSystem(hardwareMap);
        extendo = new ExtendoSystem(hardwareMap);
        robotSystems = new RobotSystems(extendo, lift, intakeSubsystem, outtakeSubsystem);
        timer = new ElapsedTime();
        matchTimer = new ElapsedTime();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startH)));

        intakeSubsystem.goToMoving();
        outtakeSubsystem.init();
        outtakeSubsystem.claw.close();
        extendo.pidEnabled = true;

        preload = new Path(
                new BezierLine(
                        new Point(startX, startY, Point.CARTESIAN),
                        new Point(preloadX, preloadY, Point.CARTESIAN)
                )
        );
        preload.setConstantHeadingInterpolation(Math.toRadians(preloadH));

        collectSamples = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(preloadX, preloadY, Point.CARTESIAN),
                                new Point(safe1Sample1X, safe1Sample1Y, Point.CARTESIAN),
                                new Point(safe2Sample1X, safe2Sample1Y, Point.CARTESIAN),
                                new Point(safe3Sample1X, safe3Sample1Y, Point.CARTESIAN),
                                new Point(sample1X, sample1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample1X, sample1Y, Point.CARTESIAN),
                                new Point(human1X, human1Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human1H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human1X, human1Y, Point.CARTESIAN),
                                new Point(safeSample2X, safeSample2Y, Point.CARTESIAN),
                                new Point(sample2X, sample2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierLine(
                                new Point(sample2X, sample2Y, Point.CARTESIAN),
                                new Point(human2X, human2Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(human2H))
                .setPathEndTimeoutConstraint(0)
                .addPath(
                        new BezierCurve(
                                new Point(human2X, human2Y, Point.CARTESIAN),
                                new Point(safeSample3X, safeSample3Y, Point.CARTESIAN),
                                new Point(sample3X, sample3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .addPath(
                        new BezierLine(
                                new Point(sample3X, sample3Y, Point.CARTESIAN),
                                new Point(human3X, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(sample3H))
                .build();


        wall = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(preloadX, preloadY, Point.CARTESIAN),
                                new Point(checkpointX, checkpointY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(checkpointX, checkpointY, Point.CARTESIAN),
                                new Point(specimenX, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();



        score1 = new Path(
                new BezierCurve(
                        new Point(human3X, human3Y, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score1X, score1Y, Point.CARTESIAN)
                )
        );
        score1.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score2 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score2X, score2Y, Point.CARTESIAN)
                )
        );
        score2.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score3 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score3X, score3Y, Point.CARTESIAN)
                )
        );
        score3.setConstantHeadingInterpolation(Math.toRadians(scoreH));

        score4 = new Path(
                new BezierCurve(
                        new Point(specimenX, specimenY, Point.CARTESIAN),
                        new Point(safeScoreX, safeScoreY, Point.CARTESIAN),
                        new Point(score4X, score4Y, Point.CARTESIAN)
                )
        );
        score4.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(score4H));


        failSafe1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(human3X, human3Y, Point.CARTESIAN),
                                new Point(human3X - 5, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(200)
                .addPath(
                        new BezierLine(
                                new Point(human3X - 5, human3Y, Point.CARTESIAN),
                                new Point(human3X + 0.5, human3Y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        failSafe = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(specimenX, specimenY, Point.CARTESIAN),
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(300)
                .addPath(
                        new BezierLine(
                                new Point(specimenX - 5, specimenY, Point.CARTESIAN),
                                new Point(specimenX + 0.5, specimenY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(preloadH)
                .setPathEndTimeoutConstraint(0)
                .build();

        park = new Path(
                new BezierCurve(
                        new Point(basketX, basketY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(parkX, parkY, Point.CARTESIAN)
                )
        );
        park.setLinearHeadingInterpolation(Math.toRadians(basketH), Math.toRadians(parkH));

        collectSampleBasket = new Path(
                new BezierCurve(
                        new Point(preloadX, preloadY, Point.CARTESIAN),
                        new Point(safe1SpecimenX, safe1SpecimenY, Point.CARTESIAN),
                        new Point(sampleBasketX, sampleBasketY, Point.CARTESIAN)
                )
        );
        collectSampleBasket.setLinearHeadingInterpolation(Math.toRadians(scoreH), Math.toRadians(sampleBasketH));

        goToBasket = new Path(
                new BezierLine(
                        new Point(sampleBasketX, sampleBasketY, Point.CARTESIAN),
                        new Point(basketX, basketY, Point.CARTESIAN)
                )
        );
        goToBasket.setLinearHeadingInterpolation(Math.toRadians(sampleBasketH), Math.toRadians(basketH));

        follower.setMaxPower(speed);
        follower.followPath(preload);

        waitForStart();

        matchTimer.reset();

        while(opModeIsActive() && matchTimer.seconds() < matchTime){

            switch (CS){

                case IDLE:
                    CS = STATES.SPECIMEN;
                    break;

                case SPECIMEN:
                    if(robotSystems.transferState == RobotSystems.TransferStates.IDLE || robotSystems.transferState == RobotSystems.TransferStates.GOING_TO_AFTER_TRANSFER) {

                        lift.goToHighChamber();
                        outtakeSubsystem.goToSpecimenPrescore();
                        outtakeSubsystem.funny.extend();

                        CS = STATES.MOVING;
                        timer.reset();
                        NS = STATES.SCORING_SPECIMEN;
                        firstTime = true;
                    }
                    break;

                case SCORING_SPECIMEN:
                    if(firstTime) {
                        robotSystems.scoreSpecimen();
                        firstTime = false;
                    }

                    timer.reset();
                    if(robotSystems.scoreSpecimenState == RobotSystems.scoreSpecimenStates.IDLE){
                        switch (SCORING_CS){
                            case IDLE:
                                CS = STATES.COLLECTING_SAMPLES;
                                break;
                            case SCORE1:
                                CS = STATES.WALL;
                                break;
                            case SCORE2:
                                CS = STATES.WALL;
                                break;
                            case SCORE3:
                                CS = STATES.WALL;
                                break;
                            case SCORE4:
                                CS = STATES.COLLECTING_SAMPLE;
                                break;
                        }
                    }
                    break;

                case MOVING:
                    if(basket){
                        if(robotSystems.transferState == RobotSystems.TransferStates.IDLE){
                            lift.goToHighBasket();
                            basket = false;
                        }
                    }
                    if(parking){
                        if(follower.getCurrentTValue() > 0.1){
                            lift.goToGround();
                            outtakeSubsystem.goToTransfer();
                            parking = false;
                        }
                    }
                    if(!follower.isBusy()){
                        CS = NS;
                        timer.reset();
                        firstTime = true;

                    }


                    if(NS == STATES.COLLECTING_SPECIMEN && SCORING_CS != SCORING_STATES.SCORE1 && follower.getCurrentTValue() > 0.85) {
                        lift.PIDON = false;
                        lift.setPower(-0.7);
                    } else if(NS == STATES.COLLECTING_SPECIMEN && SCORING_CS == SCORING_STATES.SCORE1 && timer.seconds() > 8) {
                        lift.PIDON = false;
                        lift.setPower(-0.7);
                    }

                    break;

                case WAITING:
                    if(timer.milliseconds() > timeToWait){
                        CS = NS;
                        timer.reset();
                        firstTime = true;
                    }
                    break;

                case COLLECTING_SAMPLES:
                    timer.reset();
                    follower.followPath(collectSamples);
                    lift.goToGround();
                    outtakeSubsystem.funny.maxRetract();
                    outtakeSubsystem.bar.goToSpecimenCollect();
                    outtakeSubsystem.claw.open();
                    intakeSubsystem.goToMoving();
                    CS = STATES.MOVING;
                    NS = STATES.COLLECTING_SPECIMEN;
                    SCORING_CS = SCORING_STATES.SCORE1;
                    break;

//                case CHECKPOINT:
//                    follower.followPath(checkpoint);
//                    CS = STATES.MOVING;
//                    NS = STATES.WALL1;
//                    break;
//
//                case WALL1:
//                    follower.followPath(wall1);
//                    CS = STATES.MOVING;
//                    NS = STATES.COLLECTING_SPECIMEN;
//                    break;

                case COLLECTING_SPECIMEN:
                    outtakeSubsystem.claw.close();
                    lift.reset();
                    timer.reset();
                    timeToWait = timeToCollect;
                    CS = STATES.WAITING;
                    NS = STATES.SCORE;
                    break;

//                case CHECK_SPECIMEN:
//                    if(timer.milliseconds() > timeToCheck) {
//                        if(intakeSubsystem.hasElement()){
//                            CS = STATES.TRANSFER;
//                            robotSystems.transfer();
//                        }
//                        else{
//                            intakeSubsystem.claw.open();
//                            timer.reset();
//                            timeToWait = 100;
//                            CS = STATES.WAITING;
//                            NS = STATES.FAIL_SAFE;
//                        }
//                    }
//
//                    break;
//
//                case FAIL_SAFE:
//                    if(SCORING_CS == SCORING_STATES.SCORE1){
//                        follower.followPath(failSafe1);
//                        CS = STATES.MOVING;
//                        NS = STATES.COLLECTING_SPECIMEN;
//                    }
//                    else{
//                        follower.followPath(failSafe);
//                        CS = STATES.MOVING;
//                        NS = STATES.COLLECTING_SPECIMEN;
//                    }
//                    break;

                case SCORE:
                    switch (SCORING_CS) {
                        case SCORE1:
                            follower.followPath(score1);
                            break;
                        case SCORE2:
                            follower.followPath(score2);
                            break;
                        case SCORE3:
                            follower.followPath(score3);
                            break;
                        case SCORE4:
                            follower.followPath(score4);
                            break;
                    }
                    CS = STATES.SPECIMEN;
                    break;

                case WALL:
                    follower.followPath(wall);
                    outtakeSubsystem.funny.maxRetract();
                    outtakeSubsystem.bar.goToSpecimenCollect();
                    outtakeSubsystem.claw.open();
                    lift.goToGround();
                    switch (SCORING_CS) {
                        case SCORE1:
                            SCORING_CS = SCORING_STATES.SCORE2;
                            break;
                        case SCORE2:
                            SCORING_CS = SCORING_STATES.SCORE3;
                            break;
                        case SCORE3:
                            SCORING_CS = SCORING_STATES.SCORE4;
                            break;
                    }
                    CS = STATES.MOVING;
                    timer.reset();
                    NS = STATES.COLLECTING_SPECIMEN;
                    break;

                case COLLECTING_SAMPLE:
                    follower.followPath(collectSampleBasket);
                    intakeSubsystem.goDown();
                    intakeSubsystem.claw.open();
                    lift.goToGround();
                    outtakeSubsystem.goToTransfer();
                    CS = STATES.MOVING;
                    timer.reset();
                    NS = STATES.COLLECTING_SAMPLE1;
                    break;

                case COLLECTING_SAMPLE1:
                    extendo.goToPos(380);
                    if(extendo.isAtPosition()) {
                        timeToWait = timeToCollectSample;
                        CS = STATES.WAITING;
                        NS = STATES.COLLECTING_SAMPLE2;
                        timer.reset();
                    }
                    break;
                case COLLECTING_SAMPLE2:
                    intakeSubsystem.bar.goToCollect();
                    if(timer.milliseconds() > timeToGoDownSample){
                        intakeSubsystem.claw.close();
                        CS = STATES.WAITING;
                        NS = STATES.COLLECTING_SAMPLE3;
                        timeToWait = timeToCollect;
                        timer.reset();
                    }
                    break;

                case COLLECTING_SAMPLE3:
                    robotSystems.transfer();
                    CS = STATES.SCORE_BASKET;
                    break;

                case SCORE_BASKET:
                    follower.followPath(goToBasket);
                    CS = STATES.MOVING;
                    timer.reset();
                    NS = STATES.SCORE_BASKET2;
                    basket = true;
                    break;

                case SCORE_BASKET2:
                    outtakeSubsystem.goToSampleScore();
                    CS = STATES.SCORE_BASKET3;
                    timer.reset();
                    break;

                case SCORE_BASKET3:
                    if(timer.milliseconds() > 100){
                        outtakeSubsystem.claw.open();
                        CS = STATES.PARK;
                        timer.reset();
                    }
                    break;

                case PARK:
                    if(timer.milliseconds() > 100){
                        outtakeSubsystem.goToTransfer();
                    }
                    follower.followPath(park);
                    parking = true;
                    CS = STATES.MOVING;
                    timer.reset();
                    NS = STATES.PARKED;
                    break;

                case PARKED:
                    break;
            }

            follower.update();
            robotSystems.update();
            telemetry.addData("STATE", CS);
            telemetry.addData("TIMER", timer.milliseconds());
            telemetry.addData("MATCH TIMER", matchTimer.seconds());
            telemetry.update();

        }

    }
}
