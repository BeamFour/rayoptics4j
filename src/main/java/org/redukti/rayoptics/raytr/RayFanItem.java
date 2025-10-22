// Copyright 2017-2025 Michael J. Hayford
// Original software https://github.com/mjhoptics/ray-optics
// Java version by Dibyendu Majumdar
package org.redukti.rayoptics.raytr;

import org.redukti.mathlib.Vector2;

public class RayFanItem {
    public Vector2 pupil;
    public RayPkg ray_pkg;

    public RayFanItem(Vector2 pupil, RayPkg ray_pkg) {
        this.pupil = pupil;
        this.ray_pkg = ray_pkg;
    }
}
