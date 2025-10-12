// Copyright 2017-2015 Michael J. Hayford
// Original software https://github.com/mjhoptics/ray-optics
// Java version by Dibyendu Majumdar
package org.redukti.rayoptics.specs;

public class SystemSpec {

    public String title;
    public String initials;
    public String dimensions;
    public double temperature;
    public double pressure;

    public SystemSpec() {
        title = "";
        initials = "";
        dimensions = "mm";
        temperature = 20.0;
        pressure = 760.0;
    }

    /**
     * convert nm to system units
     * <p>
     * Args:
     * nm (float): value in nm
     * <p>
     * Returns:
     * float: value converted to system units
     *
     * @param nm
     * @return
     */
    public double nm_to_sys_units(double nm) {
        if ("m".equals(dimensions))
            return 1e-9 * nm;
        else if ("cm".equals(dimensions))
            return 1e-7 * nm;
        else if ("mm".equals(dimensions))
            return 1e-6 * nm;
        else if ("in".equals(dimensions))
            return 1e-6 * nm / 25.4;
        else if ("ft".equals(dimensions))
            return 1e-6 * nm / 304.8;
        else
            return nm;
    }
}