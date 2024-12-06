package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystem.LauncherSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;


public class Robot extends TimedRobot {

    private final VictorSPX mL = new VictorSPX(Constants.MOTOR_LEFT);
    private final VictorSPX mL2 = new VictorSPX(Constants.MOTOR_LEFT2);
    private final VictorSPX mR = new VictorSPX(Constants.MOTOR_RIGHT);
    private final VictorSPX mR2 = new VictorSPX(Constants.MOTOR_RIGHT2);

    private NetworkTable table;
    private NetworkTableEntry tx, ty, area;

    private final Timer m_timer = new Timer();
    private boolean aligning = true;
    private boolean road = true;
    private boolean initSearch = false;
    private double lastSearchTime = 0;

    private final Timer launcherTimer = new Timer();
    private boolean launcherActive = false;

    private final PIDController alignPID = new PIDController(Constants.ALIGN_KP, Constants.ALIGN_KI, Constants.ALIGN_KD);
    private final PIDController distPID = new PIDController(Constants.DIST_KP, Constants.DIST_KI, Constants.DIST_KD);

    private LauncherSubsystem launcher;


    @Override
    public void robotInit() {
        initLime();
        initMotors();
        launcher = new LauncherSubsystem();
    }

    private void initMotors() {
        double deadband = 0.04;
        mL.setInverted(false);
        mL2.setInverted(false);
        mR.setInverted(true);
        mR2.setInverted(true);

        mL.setNeutralMode(NeutralMode.Brake);
        mL2.setNeutralMode(NeutralMode.Brake);
        mR.setNeutralMode(NeutralMode.Brake);
        mR2.setNeutralMode(NeutralMode.Brake);

        mL.configNeutralDeadband(deadband);
        mL2.configNeutralDeadband(deadband);
        mR.configNeutralDeadband(deadband);
        mR2.configNeutralDeadband(deadband);
    }

    private void initLime() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        area = table.getEntry("ta");
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return area.getDouble(0.0);
    }

    private void updateLimeDashboard() {
        SmartDashboard.putNumber("Eixo X", getX());
        SmartDashboard.putNumber("Eixo Y", getY());
        SmartDashboard.putNumber("Área", getArea());
    }

    @Override
    public void autonomousInit() {
        m_timer.reset();
        m_timer.start();
        aligning = true;
        road = true;
        initSearch = false;
        lastSearchTime = m_timer.get();
    }

    @Override
    public void autonomousPeriodic() {
        updateLimeDashboard();
        double taValue = getArea();

        if (taValue == 0.0) {
            searchForTag();
        } else if (taValue >= Constants.TA_DIST_MIN && taValue <= Constants.TA_DIST_MAX) {
            aligning = false;
            align();
            follower();
        } else if (aligning) {
            searchForTag();
        } else {
            stopMotors();
            if (!launcherActive) {
                startLauncher();
            } else if (launcherTimer.get() >= 4.0) { // Ajustado para 4 segundos
                stopLauncher();
            }
        }
    }

    public void startLauncher() {
        launcherTimer.reset();
        launcherTimer.start();
        launcherActive = true;
        launcher.firstStep();
        }

    public void stopLauncher() {
        launcher.stopAll();
        launcherActive = false;
        launcherTimer.stop();
    }
    public void searchForTag() {
        double goTime = m_timer.get();
        double taValue = getArea();

        if (taValue == 0.0 || taValue < Constants.TA_DIST_MIN) {
            if (!initSearch) {
                if (goTime <= 1.0) {
                    moveRight();
                }
                else if (goTime <= 2.0) {
                    moveLeft();
                }
                else {
                    initSearch = true;
                    m_timer.reset();
                }
            } else {
                if (road) {
                    moveRight();
                } else {
                    moveLeft();
                }
                if (goTime > 4.0) {
                    road = !road;
                    m_timer.reset();
                }
            }
        } else {
            aligning = false;
            align();
            follower();
        }
    }

    public void align() {
        double xOffset = getX();

        double pidOutput = alignPID.calculate(xOffset, 0);
        double setPoint = Math.max(-Constants.SPD_ADJUST, Math.min(pidOutput, Constants.SPD_ADJUST));

        setMotors(setPoint, -setPoint);
    }

    public void follower() {
        double xOffset = getX();
        double yOffset = getY();
        double area = getArea();

        double pidOutput = distPID.calculate(yOffset, Constants.TA_DIST_MAX);  // Ajusta a distância para o valor alvo
        double setPoint = Math.max(-Constants.SPD_ADJUST, Math.min(pidOutput, Constants.SPD_ADJUST));

        setMotors(-setPoint, -setPoint);

        if (Math.abs(xOffset) < Constants.ALIGN_TOLERANCE) {
            if (Math.abs(yOffset) > Constants.TA_DIST_MAX) {
                moveBackward();
            } else if (Math.abs(yOffset) < Constants.TA_DIST_MIN) {
                moveForward();
            } else {
                stopMotors();
            }
        }
    }

    public void setMotors(double lSpd, double rSpd) {
        mL.set(ControlMode.PercentOutput, lSpd);
        mL2.set(ControlMode.PercentOutput, lSpd);
        mR.set(ControlMode.PercentOutput, rSpd);
        mR2.set(ControlMode.PercentOutput, rSpd);
    }

    public void moveForward() {
        setMotors(Constants.SPD_ADJUST, Constants.SPD_ADJUST);
    }

    public void moveBackward() {
        setMotors(-Constants.SPD_ADJUST, -Constants.SPD_ADJUST);
    }

    public void moveLeft() {
        setMotors(-Constants.SPD_ADJUST , Constants.SPD_ADJUST );
    }

    public void moveRight() {
        setMotors(Constants.SPD_ADJUST, -Constants.SPD_ADJUST );
    }

    public void stopMotors() {
        setMotors(0, 0);
    }
}
