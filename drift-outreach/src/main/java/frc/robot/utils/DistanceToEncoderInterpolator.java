// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DistanceToEncoderInterpolator {
    private InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();

    public DistanceToEncoderInterpolator(){
        m_map.put(37.00, -0.118);
        // m_map.put(45.9,-0.133);
        // m_map.put(54.5, -0.130);
        // m_map.put(63.0, -0.143);
        // m_map.put(70.1, -0.147);
        // m_map.put(74.8, -0.160);
        // m_map.put(80.8, -0.158);
        // m_map.put(88.5, -0.167);
        // m_map.put(94.7,-0.176);
        // m_map.put(97.9, -0.176);
        // m_map.put(104.7, -0.179);
        // m_map.put(108.5, -0.180);
        // m_map.put(114.2, -0.184);
        // m_map.put(120.5, -0.183);
        // m_map.put(122.5, -0.185);
        // m_map.put(127.4, -0.190);
        // m_map.put(131.9, -0.194);
        // m_map.put(134.3, -0.195);
        // m_map.put(140.1, -0.194);
        // m_map.put(146.4, -0.196);
        // m_map.put(149.5,-0.200);
        m_map.put(45.1, -0.140);
        m_map.put(55.0, -0.153);
        m_map.put(61.4, -0.158);
        m_map.put(72.7, -0.173);
        m_map.put(77.0, -0.175);
        m_map.put(82.0, -0.184);
        m_map.put(87.8, -0.188);
        m_map.put(96.2, -0.187);
        m_map.put(99.4, -0.189);
        m_map.put(103.8, -0.189);
        m_map.put(111.4, -0.193);
        m_map.put(118.0, -0.197);
        m_map.put(123.3, -0.203);
        m_map.put(125.4, -0.202);
        m_map.put(134.5, -0.204);
        m_map.put(140.3, -0.206);
        m_map.put(145.7, -0.207);
    }
    
    public double getWristPosition(double distance){
        SmartDashboard.putNumber("Vision/Encoder setpoint", m_map.get(distance));
        return m_map.get(distance);
    }
}
