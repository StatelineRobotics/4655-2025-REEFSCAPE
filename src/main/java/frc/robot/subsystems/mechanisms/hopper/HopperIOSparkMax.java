package frc.robot.subsystems.mechanisms.hopper;

public class HopperIOSparkMax implements HopperIO {

    public SparkMax pivotMotor = new SparkMax(MechanismConstants.funnelId, MotorType.kBrushless);
    public ClosedLoopControler pivotController = pivotMotor.getClosedLoopController();
    public SparkAbsoluteEncoder pivotEncoder = povotMotor.getAbsoluteEncoder();

    public SparkMax beltMotor = new SparkMax(MechanismConstants.beltId, MotorType.kBrushless);
    public SparkEncoder beltEncoder = beltMotor.getEncoder();

    public HopperIOSparkMax() {
        public SparkMaxConfig wristConfig = new SparkMaxConfig();
        
        wristConfig.inverted(false).smartCurrentLimit(20);
        wristConfig.closedLoop.pid(
            FunnelConstants.kp,
            FunnelConstants.ki,
            FunnelConstants.kd
        ).setFeedback(kAbsolute);
        wristConfig.encoder.positionConversion(360).velocityConversion(360).centered(true);

        public SparkMaxConfig beltConfig = new SparkMaxConfig();
        beltConfig.inverted(false).smartCurrentLimit(10);
    }

    public void updateInputs(HopperIOInputs inputs) {
        inputs.pivotAngle = pivotEncoder.getPosition();
        pivotMotorVoltage = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
        pivotMotorCurrent = pivotMotor.getAppliedCurrent();

        beltMotorVoltage = beltMotor.getBusVoltage() * beltMotor.getAppliedOutput();
        beltMotorCurrent = beltMotor.getOutputCurrent();
    }

    public void requestBeltVoltage(double voltage) {
        beltMotor.setVoltage(voltage);
    }

    public void requestPivotAngle(double angle) {
        wristController.setReference(angle, ControlType.kPosition);
    }

    public void stopBelt() {
        beltMotor.stop();
    }

    public void stopWrist() {
        wristMotor.stop();
    }


}