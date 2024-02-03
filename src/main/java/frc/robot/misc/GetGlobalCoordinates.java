package frc.robot.misc;
public class GetGlobalCoordinates {
    //robotX and robotY is the global coordinates of the robot
    //rotationAngle is the rotation angle of the robot
    public  double pi = Math.PI;
    public  double robotX;
    public  double robotY;
    //assume the y axis is pointing forward and the x axis is pointing to the right
    //the rotationAngle of the robot
    public  double rotationAngle;
    //input x and input z are coordinates reletive to bot
    //x axis point to right, z axis point to font
    public  double[] pieceData;
    public  double inputX;
    public  double inputZ ;
    public  double xAxisDirection = rotationAngle -pi/2;
    //this is the radius of polar
    public  double targetDistanceToBot;
    public  double targetToRobotAngle;//target angle reletive to the x-axis of the robot
    public  double globalAngleOfTarget;
    //these are the global coordinates of gamepieces
    public double globalX;
    public double globalY;

    public GetGlobalCoordinates(double robotX, double robotY, double rotationAngle, double[] pieceData){
        this.robotX = robotX;
        this.robotY = robotY;
        this.rotationAngle = rotationAngle;
        this.pieceData = pieceData;
        this.inputX = pieceData[1];
        this.inputZ = pieceData[3];
        this.targetDistanceToBot = Math.pow(Math.pow(inputX,2)+Math.pow(inputZ,2),1/2);
        this.targetToRobotAngle = getTargetAngleToRobot(this.inputX,this.inputZ);
        this.globalAngleOfTarget = this.rotationAngle + this.targetToRobotAngle-pi/2;
        this.globalX = this.targetDistanceToBot * Math.cos(globalAngleOfTarget)+this.robotX;
        this.globalY = this.targetDistanceToBot * Math.sin(globalAngleOfTarget)+this.robotY;

    }
    //get the target angle reletive to x-axis of robot
    public double getTargetAngleToRobot(double xToRobot, double zToRobot){
        double targetAngle = 0;
        if (xToRobot >0 && zToRobot>0){
             targetAngle = Math.atan2(zToRobot, xToRobot);
        }
        else if (xToRobot <0 && zToRobot >0){
             targetAngle = Math.atan2(zToRobot, xToRobot);
        }
        else if (xToRobot == 0 && zToRobot >0){
            targetAngle = 0;
        }
        return targetAngle;
    }
    
   

}

  
    
