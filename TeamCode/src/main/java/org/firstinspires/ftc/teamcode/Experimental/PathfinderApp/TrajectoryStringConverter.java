package org.firstinspires.ftc.teamcode.Experimental.PathfinderApp;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryStringConverter {
    public static String generateStringFromTrajectory(ArrayList<Pose2d> p2dList, ArrayList<MoveOptions> options){
        String output = "Trajectory>>";
        for(int i = 0; i < p2dList.size(); i++){
            output += options.get(i) + ":{" + p2dList.get(i).getX() + "," + p2dList.get(i).getY() + "," +
                    p2dList.get(i).getHeading() + "},";
        }
        output = output.substring(0, output.length() - 1);
        return output + "||";
    }

    public static ArrayList<Pose2d> generateP2dFromString(String input) throws IOException{
        ArrayList<Pose2d> p2d = new ArrayList<>();
        if(input.substring(0, input.indexOf(">") + 2).equals("Trajectory>>")) {
            input = input.substring(input.indexOf(">") + 2);
            while (!input.equals("|")) {
                int[] comas = new int[3];
                int counter = 0;
                for (int i = input.indexOf("{"); i < input.indexOf("}"); i++) {
                    if (input.charAt(i) == ',') {
                        comas[counter] = i;
                        counter += 1;
                    }
                }
                try {
                    double x = Double.valueOf(input.substring(input.indexOf("{") + 1, comas[0]));
                    double y = Double.valueOf(input.substring(comas[0] + 1, comas[1]));
                    double heading = Double.valueOf(input.substring(comas[1] + 1, input.indexOf("}")));
                    p2d.add(new Pose2d(x, y, heading));
                } catch (Exception e) {
                    throw new IOException("Failed to convert input string into ArrayList<Pose2d>! \n Error occurred " +
                            "between " + input.indexOf("{") + "-" + input.indexOf("}") + "\n at '" +
                            input.substring(input.indexOf("{"), input.indexOf("}") + 1) + "'");
                }
                input = input.substring(input.indexOf("}") + 2);
            }
        }
        return p2d;
    }

    public static ArrayList<MoveOptions> generateMoveOptionsFromString(String input) throws IOException{
        ArrayList<MoveOptions> moveOptions = new ArrayList<>();
        if(input.substring(0, input.indexOf(">") + 2).equals("Trajectory>>")) {
            input = input.substring(input.indexOf(">") + 2);
            while (!input.equals("|")) {
                String movement = input.substring(0, input.indexOf(":"));
                if (movement.equals(MoveOptions.LineTo_Forward.toString()))
                    moveOptions.add(MoveOptions.LineTo_Forward);
                else if (movement.equals(MoveOptions.LineTo_Reversed.toString()))
                    moveOptions.add(MoveOptions.LineTo_Reversed);
                else if (movement.equals(MoveOptions.SplineTo_Forward.toString()))
                    moveOptions.add(MoveOptions.SplineTo_Forward);
                else if (movement.equals(MoveOptions.SplineTo_Reversed.toString()))
                    moveOptions.add(MoveOptions.SplineTo_Reversed);
                else if (movement.equals(MoveOptions.StrafeTo.toString()))
                    moveOptions.add(MoveOptions.StrafeTo);
                else
                    throw new IOException("Failed to convert input string into ArrayList<Pose2d>! \n Error occurred " +
                            "between " + 0 + "-" + input.indexOf(":") + "\n at '" +
                            input.substring(0, input.indexOf(":") + 1) + "'");
                input = input.substring(input.indexOf("}") + 2);
            }
        }
        return moveOptions;
    }
}