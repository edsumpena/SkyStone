package org.firstinspires.ftc.teamcode.Autonomous.Vision;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Tensorflow.TFODCalc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Detect {

    public Detect() {
        TFODCalc.init();
        TFODCalc.setHardwareProperties(43.30, 3.67f);
    }

    // TODO : consider using enum, or a single int - there are only three possibilities!
    public int[] getSkystonePositionsBlue(List<Recognition> updatedRecognitions, double imageWidthPx) {    //Stones left -> right
        // TODO : consider using an associative array of some sort here - this is not the ideal data structure

        if (updatedRecognitions != null) {
            ArrayList<Stone> skystoneIndex = new ArrayList<>();

            for (Recognition r : updatedRecognitions)
                skystoneIndex.add(new Stone(r.getLabel(), r.getLeft(), r.getTop(), r.getHeight(), r.getWidth()));

            if(!skystoneIndex.isEmpty() && skystoneIndex.size() >= 2)
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
                    if (containsLabel(skystoneIndex, "skystone")) {
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

                double minPos = skystoneIndex.get(0).getLeft();
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 1; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).getLabel().equalsIgnoreCase("skystone")) {
                            if (minPos > skystoneIndex.get(x).getLeft())
                                minPos = skystoneIndex.get(x).getLeft();
                        }
                    }

                    int idx = 0;
                    for (Stone s : skystoneIndex) {
                        if (s.getLeft() < minPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
                }
            }
        }
        return new int[]{-1, -1};
    }

    public int[] getSkystonePositionsRed(List<Recognition> updatedRecognitions, double imageWidthPx) {    //Stones left -> right
        // TODO : consider using an associative array of some sort here - this is not the ideal data structure

        if (updatedRecognitions != null) {
            ArrayList<Stone> skystoneIndex = new ArrayList<>();

            for (Recognition r : updatedRecognitions)
                skystoneIndex.add(new Stone(r.getLabel(), r.getLeft(), r.getTop(), r.getHeight(), r.getWidth()));

            if(!skystoneIndex.isEmpty() && skystoneIndex.size() >= 2)
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
                    if (containsLabel(skystoneIndex, "skystone")) {
                        return new int[]{3, 6};
                    } else {
                        if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getRight() >
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getRight())
                            return new int[]{2, 5};
                        else if (skystoneIndex.get(getIndex(skystoneIndex, "skystone")).getRight() <=
                                skystoneIndex.get(getIndex(skystoneIndex, "stone")).getRight())
                            return new int[]{1, 4};
                    }
                    break;
            }

            if (containsLabel(skystoneIndex, "skystone")) {

                double minPos = skystoneIndex.get(0).getRight();
                if (updatedRecognitions.size() >= 3) {
                    for (int x = 1; x < skystoneIndex.size(); x++) {
                        if (skystoneIndex.get(x).getLabel().equalsIgnoreCase("skystone")) {
                            if (minPos < skystoneIndex.get(x).getRight())
                                minPos = skystoneIndex.get(x).getRight();
                        }
                    }

                    int idx = 0;
                    for (Stone s : skystoneIndex) {
                        if (s.getRight() > minPos)
                            idx += 1;
                    }
                    return new int[]{idx + 1, idx + 4};
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

    private boolean containsLabel(ArrayList<Stone> stones, String label) {
        for (Stone s : stones)
            if (s.getLabel().equalsIgnoreCase(label))
                return true;
        return false;
    }

    private int getIndex(ArrayList<Stone> stones, String label) {
        for (int i = 0; i < stones.size(); i++)
            if (stones.get(i).getLabel().equalsIgnoreCase(label))
                return i;
        return -1;
    }

    private ArrayList<Stone> processData(ArrayList<Stone> stones) {
        float[] data = new float[stones.size()];

        for (int i = 0; i < stones.size(); i++)
            data[i] = stones.get(i).getTop();

        for (int i = 0; i < stones.size(); i++)
            if (isOutlier(data, stones.get(i).getTop())) {
                RobotLog.dd("NOTICE", stones.get(i).getTop() + ", Is Removed");
                stones.remove(i);
                i -= 1;
            }

        return stones;
    }

    private boolean isOutlier(float[] data, float testCase) {
        float[] vals = data;
        float[] lowerQuartile;
        float[] upperQuartile;

        Arrays.sort(vals);

        if (vals.length % 2 == 0) {
            lowerQuartile = Arrays.copyOfRange(vals, 0, vals.length / 2);
            upperQuartile = Arrays.copyOfRange(vals, vals.length / 2, vals.length);
        } else {
            lowerQuartile = Arrays.copyOfRange(vals, 0, vals.length / 2);
            upperQuartile = Arrays.copyOfRange(vals, vals.length / 2 + 1, vals.length);
        }

        float q1 = getMedian(lowerQuartile);
        float q3 = getMedian(upperQuartile);
        double iqr = q3 - q1;
        double lowerBounds = q1 - 1.5 * iqr;
        double upperBounds = q3 + 1.5 * iqr;

        return testCase <= lowerBounds || testCase >= upperBounds;
    }

    private static float getMedian(float[] data) {
        if (data.length % 2 == 0)
            return Math.round((data[data.length / 2] + data[data.length / 2 - 1]) / 2d);
        else
            return data[data.length / 2];
    }
}
