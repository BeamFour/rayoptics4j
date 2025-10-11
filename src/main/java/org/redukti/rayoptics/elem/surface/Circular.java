// Copyright 2017-2015 Michael J. Hayford
// Original software https://github.com/mjhoptics/ray-optics
// Java version by Dibyendu Majumdar
package org.redukti.rayoptics.elem.surface;

import org.redukti.mathlib.Vector2;

public class Circular extends Aperture {
    public double radius = 1.0;

    public Circular(double x_offset, double y_offset, double rotation, double radius) {
        super(x_offset, y_offset, rotation);
        this.radius = radius;
    }

    @Override
    public Vector2 dimension() {
        return new Vector2(radius, radius);
    }

    @Override
    public void set_dimension(double x, double y) {
        radius = x;
    }

    @Override
    public double max_dimension() {
        return radius;
    }

    @Override
    public boolean point_inside(double x, double y,double fuzz) {
        Vector2 v = tform(x, y);
        return Math.sqrt(v.x*v.x + v.y*v.y) <= radius + fuzz;
    }

    /**
     * Get a target for ray aiming to aperture boundaries.
     */
    @Override
    public Vector2 edge_pt_target(Vector2 rel_dir) {
        return new Vector2(radius*rel_dir.x, radius*rel_dir.y);
    }

    @Override
    public void apply_scale_factor(double scale_factor) {
        super.apply_scale_factor(scale_factor);
        radius *= scale_factor;
    }
}
