// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** Add your docs here. */
public class DistanceToEncoderInterpolator {
    private InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();

    public DistanceToEncoderInterpolator(){
        m_map.put(null, null);
    }
    public double getWristPosition(double distance){
        return m_map.get(distance);
    }
}
