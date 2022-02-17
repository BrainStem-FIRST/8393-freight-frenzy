package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.MinMaxAverage;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

public class BrainSTEMRobot implements Component {
    //Various components of the  robot
    public CarouselSpin carouselSpin;
    public Collector collector;
    public DepositorLift depositorLift;
    public SampleMecanumDrive drive;
    public Turret turret;
    public PixyCam pixyCam;

    //Instance of linear opmode to use for hwMap
    private LinearOpMode opMode;

    //List of components to be initialized
    private ArrayList<Component> components;
    private List<LynxModule> allHubs;
    private MinMaxAverage mma = new MinMaxAverage();
    private ElapsedTime time = new ElapsedTime();

    /**
     * Instantiates a new robot.
     *
     * @param opMode the op mode
     */
    public BrainSTEMRobot(LinearOpMode opMode) {
        this.opMode = opMode;

        //Get instance of hardware map and telemetry
        HardwareMap map = opMode.hardwareMap;

        allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule m : allHubs) {
            if(!m.isParent()) {
                allHubs.remove(m);
                continue;
            }
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new SampleMecanumDrive(opMode.hardwareMap, this);

        components = new ArrayList<>();

        //Initialize robot components
        carouselSpin = new CarouselSpin(map);
        collector = new Collector(map);
        turret = new Turret(map, opMode.telemetry);
        depositorLift = new DepositorLift(map, turret);
        pixyCam = new PixyCam(map);

        //Add all components to an array list so they can be easily initialized
        components.add(carouselSpin);
        components.add(collector);
        components.add(depositorLift);
        components.add(turret);
    }

    @Override
    public void update() {
        for (LynxModule m : allHubs) {
            m.clearBulkCache();
            m.getBulkData();
        }

        for (Component component : components) {
            component.update();
        }

        double currentTime = time.milliseconds();
        time.reset();
        mma.update(currentTime);

//        opMode.telemetry.addLine("Update time: " + currentTime);
//        opMode.telemetry.addLine(mma.toString());
//        opMode.telemetry.update();
    }

    @Override
    public void reset() {
        for (Component component : components)
            component.reset();
    }

    public String test() {
        String failures = "";
        for (Component component : components)
            failures += component.test();
        return failures;
    }
}