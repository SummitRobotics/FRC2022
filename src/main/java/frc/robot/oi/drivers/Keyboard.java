// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi.drivers;

import frc.robot.oi.inputs.OIButton;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.function.BooleanSupplier;

public class Keyboard implements KeyListener {
    
    private boolean[] keys = new boolean[65536];
    public OIButton 
    W,
    A,
    S,
    D,
    Shift,
    E,
    Q;
    public static final int 
        f1 = KeyEvent.VK_F1, 
        w = KeyEvent.VK_W, 
        a = KeyEvent.VK_A, 
        s = KeyEvent.VK_S, 
        d = KeyEvent.VK_D, 
        shift = KeyEvent.VK_SHIFT, 
        e = KeyEvent.VK_E, 
        q = KeyEvent.VK_Q; 
    public Keyboard(){
        W = new OIButton(isKeyPressed(w));
        A = new OIButton(isKeyPressed(a));
        S = new OIButton(isKeyPressed(s));
        D = new OIButton(isKeyPressed(d));
        Shift = new OIButton(isKeyPressed(shift));
        E = new OIButton(isKeyPressed(e));
        Q = new OIButton(isKeyPressed(q));
       
    }
    public void keyPressed(KeyEvent e) {
        keys[e.getKeyCode()] = true;
    }
    
    public void keyReleased(KeyEvent e) {
        keys[e.getKeyCode()] = false;
    }
    
    public void keyTyped(KeyEvent e) {
    }
    
    public BooleanSupplier isKeyPressed(int key) {
        return () -> keys[key];
    }
}