package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Detect {

    public static double tolerance = 0.9;

    public Detect() {
        TFODCalc.init();
        TFODCalc.setHardwareProperties(43.30, 3.67f);
    }

    // TODO : consider using enum, or a single int - there are only three possibilities!
    public static int[] getSkystonePositionsBlue(List<Recognition> updatedRecognitions, double imageWidthPx) {    //Stones left -> right
        // TODO : consider using an associative array of some sort here - this is not the ideal data structure

        if (updatedRecognitions != null) {
            ArrayList<Stone> skystoneIndex = new ArrayList<>();

            for (Recognition r : updatedRecognitions)
                skystoneIndex.add(new Stone(r.getLabel(), r.getLeft(), r.getTop(), r.getHeight(), r.getWidth()));

            if (!skystoneIndex.isEmpty() && skystoneIndex.size() >= 2)
                skystoneIndex = processData(skystoneIndex);

            switch (updatedRecognitions.size()) {
                case 1:
                    // if only one skystone is detected, segment image like |1|2|3-|
                    // if midpoint of skystone is in one of these "regions", assume the positions of remaining stones
                    if (skystoneIndex.get(0).getLabel().equalsIgnoreCase("skystone")) {
                        double horizontalMid = skystoneIndex.get(0).getCenter()[0];
                        double dividedImg = imageWidthPx / 3d;

                        if (horizontalMid <= dividedImg)
                            return new int[]{1, 4};
                        else if (horizontalMid > dividedImg && horizontalMid <= dividedImg * 2)
                            return new int[]{2, 5};
                        else
                            return new int[]{3, 6};
                    } else
                        return new int[]{1, 4};
                case 2:
                    // if only see two, and neither are skystones, assume that the skystone is out of view and predict position
                    // based on that

                    // if one is a skystone and one is not, predict position based on their relative positions
                    if (!containsLabel(skystoneIndex, "skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getLeft() >
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getLeft())
                            return new int[]{2, 5};
                        else if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getLeft() <=
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getLeft())
                            return new int[]{1, 4};
                    }
                    break;
            }

            if (containsLabel(skystoneIndex, "skystone")) {
                int stones = 0;
                if (updatedRecognitions.size() >= 3) {
                    double skystoneLeft = skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getLeft();;
                    for(int i = 0; i < skystoneIndex.size(); i++){
                        if(skystoneIndex.get(i).getLabel().equalsIgnoreCase("skystone") &&
                                skystoneLeft < skystoneIndex.get(i).getLeft())
                            skystoneLeft = skystoneIndex.get(i).getLeft();
                    }
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).getLabel().equalsIgnoreCase("stone")) {
                            if (skystoneLeft > skystoneIndex.get(x).getLeft())
                                stones += 1;
                        }
                    }

                    return new int[]{stones + 1, stones + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }

    public static int[] getSkystonePositionsRed(List<Recognition> updatedRecognitions, double imageWidthPx) {    //Stones left -> right
        // TODO : consider using an associative array of some sort here - this is not the ideal data structure

        if (updatedRecognitions != null) {
            ArrayList<Stone> skystoneIndex = new ArrayList<>();

            for (Recognition r : updatedRecognitions)
                skystoneIndex.add(new Stone(r.getLabel(), r.getLeft(), r.getTop(), r.getHeight(), r.getWidth()));

            if (!skystoneIndex.isEmpty() && skystoneIndex.size() >= 2)
                skystoneIndex = processData(skystoneIndex);

            switch (updatedRecognitions.size()) {
                case 1:
                    // if only one skystone is detected, segment image like |1|2|3-|
                    // if midpoint of skystone is in one of these "regions", assume the positions of remaining stones
                    if (skystoneIndex.get(0).getLabel().equalsIgnoreCase("skystone")) {
                        double horizontalMid = skystoneIndex.get(0).getCenter()[0];
                        double dividedImg = imageWidthPx / 3d;

                        if (horizontalMid >= dividedImg)
                            return new int[]{1, 4};
                        else if (horizontalMid < dividedImg && horizontalMid >= dividedImg * 2)
                            return new int[]{2, 5};
                        else
                            return new int[]{3, 6};
                    } else
                        return new int[]{1, 4};
                case 2:
                    // if only see two, and neither are skystones, assume that the skystone is out of view and predict position
                    // based on that

                    // if one is a skystone and one is not, predict position based on their relative positions
                    if (!containsLabel(skystoneIndex, "skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getRight() <
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getRight())
                            return new int[]{2, 5};
                        else if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getRight() >=
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getRight())
                            return new int[]{1, 4};
                    }
                    break;
            }

            if (containsLabel(skystoneIndex, "skystone")) {
                int stones = 0;
                if (updatedRecognitions.size() >= 3) {
                    double skystoneRight = skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getRight();
                    for(int i = 0; i < skystoneIndex.size(); i++){
                        if(skystoneIndex.get(i).getLabel().equalsIgnoreCase("skystone") &&
                                skystoneRight > skystoneIndex.get(i).getRight())
                            skystoneRight = skystoneIndex.get(i).getRight();
                    }
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).getLabel().equalsIgnoreCase("stone")) {
                            if (skystoneRight < skystoneIndex.get(x).getRight())
                                stones += 1;
                        }
                    }

                    return new int[]{stones + 1, stones + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }

    /*public int[] getSkystonePositionsRed(List<Recognition> updatedRecognitions, double imageWidthPx) {     //Stones right -> left
        if (updatedRecognitions != null) {
            int index = 0;
            double[] right = new double[updatedRecognitions.size() + 1];
            ArrayList<String> skystoneIndex = new ArrayList<>();

            for (Recognition recognition : updatedRecognitions) {
                right[index] = recognition.getRight();
                if (recognition.getLabel().equalsIgnoreCase("skystone"))
                    skystoneIndex.add("skystone");
                else
                    skystoneIndex.add("stone");
                index += 1;
            }

            switch (updatedRecognitions.size()) {
                case 1:
                    if (skystoneIndex.get(0).equalsIgnoreCase("skystone")) {
                        double horizontalMid = updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getWidth() / 2;
                        double dividedImg = imageWidthPx / 4;

                        if (horizontalMid >= dividedImg * 3)
                            return new int[]{1, 4};
                        else if (horizontalMid < dividedImg * 3 && horizontalMid >= dividedImg * 2)
                            return new int[]{2, 5};
                        else
                            return new int[]{3, 6};
                    } else
                        return new int[]{1, 4};
                case 2:
                    if (!skystoneIndex.contains("skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (right[skystoneIndex.indexOf("skystone")] > right[skystoneIndex.indexOf("stone")])
                            return new int[]{1, 4};
                        else if (right[skystoneIndex.indexOf("skystone")] <= right[skystoneIndex.indexOf("stone")])
                            return new int[]{2, 5};
                    }
                    break;
            }

            if (skystoneIndex.contains("skystone")) {
                double maxPos = -9999;
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 0; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).equalsIgnoreCase("skystone")) {
                            if (maxPos < right[x])
                                maxPos = right[x];
                        }
                    }

                    int idx = 0;
                    for (double r : right) {
                        if (r > maxPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }*/

    private static boolean containsLabel(ArrayList<Stone> stones, String label) {
        for (Stone s : stones)
            if (s.getLabel().equalsIgnoreCase(label))
                return true;
        return false;
    }

    private static int getIndex(ArrayList<Stone> stones, String label) {
        for (int i = 0; i < stones.size(); i++)
            if (stones.get(i).getLabel().equalsIgnoreCase(label))
                return i;
        return -1;
    }

    private static ArrayList<Stone> processData(ArrayList<Stone> stones) {
        float[] data = new float[stones.size()];

        for (int i = 0; i < stones.size(); i++)
            data[i] = stones.get(i).getTop();

        RobotLog.dd("Current List (Pre-Remove processing)", stones.toString());
        for (int i = 0; i < stones.size(); i++)
            if (stones.size() >= 3 && isOutlier(data, stones.get(i).getTop())) {
                RobotLog.dd(">>>", stones.get(i) + " is filtered out from the list!");
                stones.remove(i);
                i -= 1;
            }
        RobotLog.dd("Current List (Post-Remove processing)", stones.toString());

        /*if (stones.size() >= 2) {
            float[] centerDeltaX = new float[stones.size() - 1];
            float[] centerX = new float[stones.size()];

            for(int i = 0; i < centerX.length; i++)
                centerX[i] = stones.get(i).getCenter()[0];

            Arrays.sort(centerX);

            for (int i = 0; i < centerX.length - 1; i++)
                centerDeltaX[i] = Math.abs(centerX[i] - centerX[i + 1]);

            RobotLog.dd("Center Deltas", Arrays.toString(centerDeltaX));

            RobotLog.dd("Current List (Pre-Imply processing)", stones.toString());
            for (int i = 0; i < centerDeltaX.length; i++) {
                RobotLog.dd("Implying Loop Index", String.valueOf(i));
                RobotLog.dd("Value at Index", String.valueOf(centerDeltaX[i]));
                if (centerDeltaX[i] > 200) {
                    double impledCenterX = (stones.get(i).getCenter()[0] + stones.get(i + 1).getCenter()[0]) / 2d;
                    Stone impliedStone = generateImpliedStone(stones, impledCenterX);
                    RobotLog.dd(">>>", "Implied Stone CenterXs = [" + stones.get(i).getCenter()[0] + " - " + stones.get(i + 1).getCenter()[0] +
                            "], Index = " + i + ", Delta = " + centerDeltaX[i] + ", Implied Center = " + impledCenterX);
                    RobotLog.dd("Implied Stone", impliedStone.toString());
                    stones.add(impliedStone);
                }

            }
            RobotLog.dd("Current List (Post-Imply processing)", stones.toString());
        }*/

        return stones;
    }

    private static Stone generateImpliedStone(ArrayList<Stone> stones, double centerX) {
        double avgCenterY = 0;
        double avgHeight = 0;
        double avgWidth = 0;
        String label = containsLabel(stones, "skystone") ? "stone" : "skystone";

        for (Stone s : stones) {
            avgCenterY += s.getTop();
            avgHeight += s.getHeight();
            avgWidth += s.getWidth();
        }

        avgCenterY /= 4;
        avgHeight /= 4;
        avgWidth /= 4;

        return new Stone(label, Math.round(centerX - avgWidth / 2d), Math.round(avgCenterY - avgHeight / 2d),
                Math.round(avgHeight), Math.round(avgWidth));
    }

    private static boolean isOutlier(float[] data, float testCase) {
        float[] lowerQuartile = null;
        float[] upperQuartile = null;

        RobotLog.dd("DATA", Arrays.toString(data));

        Arrays.sort(data);

        RobotLog.dd("SORTED DATA", Arrays.toString(data));

        if (data.length % 2 == 0 && data.length > 3) {
            lowerQuartile = Arrays.copyOfRange(data, 0, data.length / 2 - 1);
            upperQuartile = Arrays.copyOfRange(data, data.length / 2, data.length - 1);
        } else {
            if (data.length > 3) {
                lowerQuartile = Arrays.copyOfRange(data, 0, data.length / 2 - 1);
                upperQuartile = Arrays.copyOfRange(data, data.length / 2 + 1, data.length - 1);
            } else if (data.length == 3) {
                lowerQuartile = new float[]{data[0]};
                upperQuartile = new float[]{data[2]};
            }
        }

        RobotLog.dd("LowerQuartile", Arrays.toString(lowerQuartile));
        RobotLog.dd("UpperQuartile", Arrays.toString(upperQuartile));

        double q1 = getMedian(lowerQuartile);
        double q3 = getMedian(upperQuartile);
        double iqr = q3 - q1;
        double lowerBounds = q1 - tolerance * iqr;
        double upperBounds = q3 + tolerance * iqr;

        RobotLog.dd(">>>", "Q1 = " + q1 + ", Q3 = " + q3 + ", IQR = " + iqr +
                ", {LowerBounds, UpperBounds} = {" + lowerBounds + ", " + upperBounds + "}, TestCase: " + testCase);

        return testCase <= lowerBounds || testCase >= upperBounds;
    }

    private static double getMedian(float[] data) {
        if (data.length % 2 == 0)
            return (data[data.length / 2] + data[data.length / 2 - 1]) / 2d;
        else
            return data[data.length / 2];
    }
}
