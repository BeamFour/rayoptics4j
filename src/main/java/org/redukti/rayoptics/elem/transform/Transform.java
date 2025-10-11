// Copyright 2017-2015 Michael J. Hayford
// Original software https://github.com/mjhoptics/ray-optics
// Java version by Dibyendu Majumdar
package org.redukti.rayoptics.elem.transform;

import org.redukti.mathlib.Matrix3;
import org.redukti.mathlib.Vector3;
import org.redukti.rayoptics.math.Tfm3d;
import org.redukti.rayoptics.raytr.RayData;
import org.redukti.rayoptics.seq.Gap;
import org.redukti.rayoptics.seq.Interface;
import org.redukti.rayoptics.seq.PathSeg;
import org.redukti.rayoptics.seq.SequentialModel;
import org.redukti.rayoptics.util.Lists;
import org.redukti.rayoptics.util.Pair;
import org.redukti.rayoptics.util.ZDir;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public class Transform {

    interface TransformCalc {
        Tfm3d transform(Interface s1, double zdist, Interface s2);
    }

    private static void accumulate_transforms(Iterator<PathSeg> seq,
                                              PathSeg b4_seg,
                                              TransformCalc transform_calc,
                                              Tfm3d tfrm_prev,
                                              int tfrm_dir,
                                              List<Tfm3d> tfrms) {
        var b4_ifc = b4_seg.ifc;
        var b4_gap = b4_seg.gap;
        var r_prev = tfrm_prev.rt;
        var t_prev = tfrm_prev.t;
        while (seq.hasNext()) {
            PathSeg seg = seq.next();
            var ifc = seg.ifc;
            var gap = seg.gap;
            var zdist = tfrm_dir * b4_gap.thi;
            var tf = transform_calc.transform(b4_ifc, zdist, ifc);
            var r = tf.rt;
            var t = tf.t;
            var t_new = r_prev.multiply(t).add(t_prev);
            var r_new = r_prev.multiply(r);
            tfrms.add(new Tfm3d(r_new, t_new));
            r_prev = r_new;
            t_prev = t_new;
            b4_ifc = ifc;
            b4_gap = gap;
        }
    }

    /**
     * Return global surface coordinates (rot, t) wrt surface `glo`.

     *  Args:
        seq_model: sequential model
        glo: global reference surface index
        origin: None | (r_origin, t_origin)

    The tuple (r_origin, t_origin) is the transform from the desired
    global origin to the global surface `glo`.
     */
    public static List<Tfm3d> compute_global_coords(SequentialModel seq_model, Integer glo, Tfm3d origin) {
        if (glo == null)
            glo = 1;
        // Initialize origin of global coordinate system.
        List<Tfm3d> tfrms = new ArrayList<>();
        if (origin == null)
            origin = new Tfm3d(Matrix3.IDENTITY,Vector3.ZERO);
        var tfrm_origin = origin;
        tfrms.add(tfrm_origin);

        List<PathSeg> seq;
        PathSeg b4_seg;

        // Compute transforms from global surface to object surface
        if (glo > 0) {
            // iterate in reverse over the segments before the
            // global reference surface
            int step = -1;
            seq = zip_longest(
                    Lists.slice(seq_model.ifcs, glo, null, step),
                    Lists.slice(seq_model.gaps, glo - 1, null, step),
                    Lists.slice(seq_model.z_dir, glo - 1, null, step));
            var iter = seq.iterator();
            b4_seg = iter.next();
            // loop of remaining surfaces in path
            accumulate_transforms(iter,b4_seg,Transform::reverse_transform,tfrm_origin,-1,tfrms);
            tfrms = Lists.slice(tfrms, null, null, -1); // reverse
        }
        // Compute transforms from global surface to image surface
        seq = zip_longest(Lists.from(seq_model.ifcs, glo),
                            Lists.from(seq_model.gaps, glo),
                            Lists.from(seq_model.z_dir,glo));
        var iter = seq.iterator();
        b4_seg = iter.next();
        accumulate_transforms(iter,b4_seg,Transform::forward_transform,tfrm_origin,1,tfrms);
        return tfrms;
    }

    private static void local_transform(Iterator<Pair<Interface, Gap>> seq,
                                   TransformCalc transform_calc,
                                   int tfrm_dir,
                                   List<Tfm3d> tfrms) {
        var b4seg = seq.next();
        var b4_ifc = b4seg.first;
        var b4_gap = b4seg.second;
        while (seq.hasNext()) {
            var seg = seq.next();
            var ifc = seg.first;
            var gap = seg.second;
            var zdist = tfrm_dir * b4_gap.thi;
            var tf = transform_calc.transform(b4_ifc, zdist, ifc);
            var r = tf.rt;
            var t = tf.t;
            var rt = r.transpose();
            tfrms.add(new Tfm3d(rt, t));
            b4_ifc = ifc;
            b4_gap = gap;
        }
    }

    /**
     * Return forward surface coordinates (r.T, t) for each interface.
     * @param seq
     * @param step
     * @return
     */
    public static List<Tfm3d> compute_local_transforms(SequentialModel seq_model, List<Pair<Interface, Gap>> seq, int step) {
        var num_ifcs = seq_model.get_num_surfaces();
        List<Tfm3d> tfrms = new ArrayList<>();
        if (step == -1) {
            if (seq == null) {
                seq = zip_longest(
                    Lists.slice(seq_model.ifcs, num_ifcs, null, step),
                    Lists.slice(seq_model.gaps, num_ifcs-1, null, step));
            }
            local_transform(seq.iterator(),Transform::reverse_transform,-1,tfrms);
        }
        else if (step == 1) {
            if (seq == null) {
                seq = zip_longest(
                    Lists.step(seq_model.ifcs, step),
                    Lists.step(seq_model.gaps, step));
            }
            local_transform(seq.iterator(),Transform::forward_transform,1,tfrms);
        }
        tfrms.add(new Tfm3d(Matrix3.IDENTITY,Vector3.ZERO));
        return tfrms;
    }

    /**
     * generate transform rotation and translation from
     *         s1 coords to s2 coords
     *
     * @param s1
     * @param zdist
     * @param s2
     * @return
     */
    public static Tfm3d forward_transform(Interface s1, double zdist, Interface s2) {
        // calculate origin of s2 wrt to s1
        Vector3 t_orig = new Vector3(0., 0., zdist);
        Matrix3 r_after_s1 = null,
                r_before_s2 = null;
        if (s1.decenter != null) {
            // get transformation info after s1
            Pair<Matrix3, Vector3> after = s1.decenter.tform_after_surf();
            r_after_s1 = after.first;
            Vector3 t_after_s1 = after.second;
            t_orig = t_orig.add(t_after_s1);
        }
        if (s2.decenter != null) {
            // get transformation info before s2
            Pair<Matrix3, Vector3> before = s2.decenter.tform_before_surf();
            r_before_s2 = before.first;
            Vector3 t_before_s2 = before.second;
            t_orig = t_orig.add(t_before_s2);
        }
        Matrix3 r_cascade = Matrix3.IDENTITY;
        if (r_after_s1 != null) {
            // rotate the origin of s2 around s1 "after" transformation
            t_orig = r_after_s1.multiply(t_orig);
            r_cascade = r_after_s1;
            if (r_before_s2 != null) {
                r_cascade = r_after_s1.multiply(r_before_s2);
            }
        }
        else if (r_before_s2 != null) {
            r_cascade = r_before_s2;
        }
        return new Tfm3d(r_cascade, t_orig);
    }

    /**
     * generate transform rotation and translation from
     *         s2 coords to s1 coords
     *
     * @param s1
     * @param zdist
     * @param s2
     * @return
     */
    public static Tfm3d reverse_transform(Interface s1, double zdist, Interface s2) {
        // calculate origin of s2 wrt to s1
        Vector3 t_orig = new Vector3(0., 0., zdist);
        Matrix3 r_after_s1 = null,
                r_before_s2 = null;
        if (s1.decenter != null) {
            // get transformation info after s1
            Pair<Matrix3, Vector3> after = s1.decenter.tform_after_surf();
            r_after_s1 = after.first;
            Vector3 t_after_s1 = after.second;
            t_orig = t_orig.add(t_after_s1);
        }
        if (s2.decenter != null) {
            // get transformation info before s2
            Pair<Matrix3, Vector3> before = s2.decenter.tform_before_surf();
            r_before_s2 = before.first;
            Vector3 t_before_s2 = before.second;
            t_orig = t_orig.add(t_before_s2);
        }
        // going in reverse direction so negate translation
        t_orig = t_orig.negate();
        Matrix3 r_cascade = Matrix3.IDENTITY;
        if (r_before_s2 != null) {
            // rotate the origin of s1 around s2 "before" transformation
            r_cascade = r_before_s2.transpose();
            t_orig = r_cascade.multiply(t_orig); // TODO check what dot() does
            if (r_after_s1 != null) {
                r_cascade = r_cascade.multiply(r_after_s1.transpose()); // TODO check what dot does
            }
        }
        else if (r_after_s1 != null) {
            r_cascade = r_after_s1.transpose();
        }
        return new Tfm3d(r_cascade, t_orig);
    }

    /**
     * Transform ray_seg from interface to following seg.
     *
     *     Args:
     *         interface: the :class:'~seq.interface.Interface' for the path sequence
     *         ray_seg: ray segment exiting from **interface**
     *
     *     Returns:
     *         (**b4_pt**, **b4_dir**)
     *
     *         - **b4_pt** - ray intersection pt wrt following seg
     *         - **b4_dir** - ray direction cosine wrt following seg
     * @param ifc
     * @param ray_seg
     */
    public static RayData transform_after_surface(Interface ifc, RayData ray_seg) {
        Vector3 b4_pt;
        Vector3 b4_dir;
        if (ifc.decenter != null) {
            // get transformation info after surf
            Pair<Matrix3, Vector3> xform = ifc.decenter.tform_after_surf();
            Matrix3 r = xform.first;
            Vector3 t = xform.second;
            if (r == null) {
                b4_pt = ray_seg.pt.minus(t);
                b4_dir = ray_seg.dir;
            }
            else {
                Matrix3 rt = r.transpose();
                b4_pt = rt.multiply(ray_seg.pt.minus((t)));
                b4_dir = rt.multiply(ray_seg.dir);
            }
        }
        else {
            b4_pt = ray_seg.pt;
            b4_dir = ray_seg.dir;
        }
        return new RayData(b4_pt, b4_dir);
    }

    public static List<Pair<Interface, Gap>> zip_longest(List<Interface> ifcs, List<Gap> gaps) {
        List<Pair<Interface, Gap>> list = new ArrayList<>();
        for (int i = 0; i < Math.max(ifcs.size(), gaps.size()); i++) {
            Interface ifc = i < ifcs.size() ? ifcs.get(i) : null;
            Gap g = i < gaps.size() ? gaps.get(i) : null;
            list.add(new Pair<>(ifc, g));
        }
        return list;
    }

    public static List<PathSeg> zip_longest(List<Interface> ifcs,
                                                List<Gap> gaps,
                                                List<ZDir> z_dir) {
        List<PathSeg> list = new ArrayList<>();
        List<Integer> sizes = List.of(ifcs.size(), gaps.size(), z_dir.size());
        int maxSize = sizes.stream().max(Comparator.naturalOrder()).orElse(0);
        for (int i = 0; i < maxSize; i++) {
            Interface ifc = i < ifcs.size() ? ifcs.get(i) : null;
            Gap gap = i < gaps.size() ? gaps.get(i) : null;
            ZDir dir = i < z_dir.size() ? z_dir.get(i) : null;
            list.add(new PathSeg(ifc, gap, null, null, dir));
        }
        return list;
    }

}
