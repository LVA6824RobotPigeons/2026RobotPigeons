package org.py.cmm5;

public class Sandbox {

    public static void sandboxStart() {

        Controls.controlsLogger(true);

        Controls.addControl("test", Controls.BinaryComponents.A,new int[] {1})
                .linkControl(Controls.BinaryComponents.B);

        Controls.addControl("test2",Controls.ThresholdComponents.AX,"GREATER_THAN:0.5");

    }

    public static boolean ready = false;
    public static void sandboxProcess() {

    }

}
