package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

public class InverseKinematicArmSubsystem {
    //should be constants but examples rn
    double elevatorAngleToHorizon = Math.PI/4; //placeholder 45 deg for now IDK
    double armLength = 0.5; //EXAMPLE
    double elevatorMaxLength = 2; //EXAMPLE

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    Translation2d armSetPos;

    public InverseKinematicArmSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
    }

    public InverseKinematicArmSubsystem(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, Translation2d armStartPos) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        armSetPos = armStartPos;
    }

    public ArmElevatorData getArmElevatorPos() {
        ArmElevatorData m_armElevatorData = new ArmElevatorData();

        BiDouble armXValues = getArmXValues(armSetPos);

        BiDouble armDistances = new BiDouble();
        armDistances.a = armXValues.a / Math.cos(elevatorAngleToHorizon);
        armDistances.b = armXValues.b / Math.cos(elevatorAngleToHorizon);

        if (armDistances.a > elevatorMaxLength || armDistances.b < 0)
            armDistances.a = 0;
        if (armDistances.b > elevatorMaxLength || armDistances.b < 0)
            armDistances.b = 0;
        
        BiDouble armDistancesFromCurrent = new BiDouble();
        armDistancesFromCurrent.a = Math.abs(armDistances.a - elevatorSubsystem.getPosition()); //THE GET POSITION COULD BE WRONG IDK ILL CHECK LATER
        armDistancesFromCurrent.b = Math.abs(armDistances.b - elevatorSubsystem.getPosition());

        if (armDistancesFromCurrent.a > armDistancesFromCurrent.b) {
            if ((armDistancesFromCurrent.a < elevatorMaxLength) && (armDistancesFromCurrent.a > 0)) {
                m_armElevatorData.meters = armDistances.a;
            }
        } else if ((armDistancesFromCurrent.b < elevatorMaxLength) && (armDistancesFromCurrent.b > 0)) {
            m_armElevatorData.meters = armDistances.b;
        }

        return m_armElevatorData;
    }

    //heres the desmos graph i used to make this: https://www.desmos.com/calculator/ztbzmjo1e6
    public BiDouble getArmXValues(Translation2d setPos) {
        double tanOfElevatorAngle = Math.tan(elevatorAngleToHorizon);
        double theBitThatsUnderTheSqrtIForgotTheName = 
            Math.pow(-2 * setPos.getY() * tanOfElevatorAngle -2 * setPos.getX(), 2)
            -4 * (Math.pow(tanOfElevatorAngle, 2) + 1) *
            (Math.pow(setPos.getY(),2) + Math.pow(setPos.getX(),2) - armLength);
        double thePartBeforeTheSqrtIForgotTheName = 
            (2 * setPos.getY() * tanOfElevatorAngle + 2 * setPos.getX());
        double thePartThatDividesItAll =
            2 * (Math.pow(theBitThatsUnderTheSqrtIForgotTheName, 2) + 1);

        BiDouble rtrn = new BiDouble();

        rtrn.a = (thePartBeforeTheSqrtIForgotTheName + Math.sqrt(theBitThatsUnderTheSqrtIForgotTheName)) / thePartThatDividesItAll;
        rtrn.b = (thePartBeforeTheSqrtIForgotTheName - Math.sqrt(theBitThatsUnderTheSqrtIForgotTheName)) / thePartThatDividesItAll;
        return rtrn;
    }

    private class ArmElevatorData {
        public double radians = 0;
        public double meters = 0;
    }
    private class BiDouble{
        public double a = 0;
        public double b = 0;
    }
}
