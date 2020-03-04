package com.team1816.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;

import com.team1816.frc2020.Constants;
import com.team1816.lib.hardware.MotorUtil;
import com.team1816.lib.hardware.TalonSRXChecker;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Shooter extends Subsystem implements PidProvider {
    private static final String NAME = "shooter";
    private static Shooter INSTANCE;

    private LedManager ledManager = LedManager.getInstance();

    private NetworkTable networkTable;
    private double distance;

    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }

        return INSTANCE;
    }

    // Components
    private final IMotorControllerEnhanced shooterMain;
    private final IMotorControllerEnhanced shooterFollower;
    private final Camera camera = Camera.getInstance();

    // State
    private double shooterVelocity;
    private boolean outputsChanged;

    // Constants
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    public static final int MAX_VELOCITY = 11_800; // Far
    public static final int NEAR_VELOCITY = 11_100;  // Initiation line
    public static final int MID_VELOCITY = 9_900; // Trench this also worked from initiation
    public static final int MID_FAR_VELOCITY = 11_200;
    public static final int VELOCITY_THRESHOLD = (int) factory.getConstant(NAME, "velocityThreshold", 3000);

    private SendableChooser<Integer> velocityChooser = new SendableChooser<>();
    private VelocityManager velocityManager = new VelocityManager();

    private Shooter() {
        super(NAME);

        this.shooterMain = factory.getMotor(NAME, "shooterMain");
        this.shooterFollower = (IMotorControllerEnhanced) factory.getMotor(NAME, "shooterFollower", shooterMain);

        this.kP = factory.getConstant(NAME, "kP");
        this.kI = factory.getConstant(NAME, "kI");
        this.kD = factory.getConstant(NAME, "kD");
        this.kF = factory.getConstant(NAME, "kF");

        shooterMain.setNeutralMode(NeutralMode.Coast);
        shooterFollower.setNeutralMode(NeutralMode.Coast);

        configCurrentLimits(40 /* amps */);

        shooterMain.setInverted(false);
        shooterFollower.setInverted(true);

        shooterMain.configClosedloopRamp(0.5, Constants.kCANTimeoutMs);
        shooterMain.setSensorPhase(false);

        networkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

        networkTable.addEntryListener("distance", (table, key, entry, value, flags) -> {
            distance = value.getDouble();
        },EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    private void configCurrentLimits(int currentLimitAmps) {
        MotorUtil.configCurrentLimit(shooterMain, true, currentLimitAmps, 0, 0);
        MotorUtil.configCurrentLimit(shooterFollower, true, currentLimitAmps, 0, 0);
    }

    @Override
    public double getKP() {
        return kP;
    }

    @Override
    public double getKI() {
        return kI;
    }

    @Override
    public double getKD() {
        return kD;
    }

    @Override
    public double getKF() {
        return kF;
    }

    public void setVelocity(double velocity) {
        this.shooterVelocity = velocity;
        outputsChanged = true;
    }

    public void shootFromChooser(boolean shooting) {
        setVelocity(shooting ? velocityChooser.getSelected() : 0);
    }

    public void startShooter() {
        setVelocity(velocityManager.getShooterVelocity(camera.getDistance()));
    }

    public void stopShooter() {
        setVelocity(0);
    }

    public double getActualVelocity() {
        return shooterMain.getSelectedSensorVelocity(0);
    }

    public double getTargetVelocity() {
        return shooterVelocity;
    }

    public double getError() {
        return shooterMain.getClosedLoopError(0);
    }

    public boolean isVelocityNearTarget() {
        return Math.abs(this.getError()) < VELOCITY_THRESHOLD;
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsChanged) {
            if (shooterVelocity == 0) {
                this.shooterMain.set(ControlMode.PercentOutput, 0); // Inertia coast to 0
            } else {
                this.shooterMain.set(ControlMode.Velocity, shooterVelocity);
            }
            outputsChanged = false;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Shooter/IsAtSpeed", this::isVelocityNearTarget, null);
        builder.addDoubleProperty("Shooter/ShooterVelocity", this::getActualVelocity, this::setVelocity);

        velocityChooser.setDefaultOption("NEAR_VELOCITY", NEAR_VELOCITY);
        velocityChooser.addOption("MID_VELOCITY", MID_VELOCITY);
        velocityChooser.addOption("MID_FAR_VELOCITY", MID_FAR_VELOCITY);
        velocityChooser.addOption("MAX_VELOCITY", MAX_VELOCITY);

        SmartDashboard.putData(velocityChooser);
    }

    @Override
    public void stop() {

    }

    private TalonSRXChecker.CheckerConfig getTalonCheckerConfig(IMotorControllerEnhanced talon) {
        return TalonSRXChecker.CheckerConfig.getForSubsystemMotor(this, talon);
    }

    @Override
    public boolean checkSystem() {
        boolean checkShooter = TalonSRXChecker.checkMotors(this,
            new ArrayList<>() {
                {
                    add(new TalonSRXChecker.TalonSRXConfig("shooterMain", shooterMain));
                }
            }, getTalonCheckerConfig(shooterMain));

        System.out.println(checkShooter);
        if (checkShooter){
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        }
        else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return checkShooter;
    }
}
