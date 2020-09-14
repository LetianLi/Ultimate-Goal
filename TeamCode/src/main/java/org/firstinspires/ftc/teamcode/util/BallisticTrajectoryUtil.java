package org.firstinspires.ftc.teamcode.util;

// LICENSE
//
//   This software is dual-licensed to the public domain and under the following
//   license: you are granted a perpetual, irrevocable license to copy, modify,
//   publish, and distribute this file as you see fit.
//
// VERSION
//   0.1.0  (2016-06-01)  Initial release
//
// AUTHOR
//   Forrest Smith
//
// TRANSPOSED FROM C# TO JAVA BY
//   Letian Li - FTC Team 12611 TechNova
//
// ADDITIONAL READING
//   https://medium.com/@ForrestTheWoods/solving-ballistic-trajectories-b0165523348c
//   https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
//
// API
//    int solve_ballistic_arc(Vector3 proj_pos, float proj_speed, Vector3 target, float gravity, out Vector3 low, out Vector3 high);
//    int solve_ballistic_arc(Vector3 proj_pos, float proj_speed, Vector3 target, Vector3 target_velocity, float gravity, out Vector3 s0, out Vector3 s1, out Vector3 s2, out Vector3 s3);
//    bool solve_ballistic_arc_lateral(Vector3 proj_pos, float lateral_speed, Vector3 target, float max_height, out float vertical_speed, out float gravity);
//    bool solve_ballistic_arc_lateral(Vector3 proj_pos, float lateral_speed, Vector3 target, Vector3 target_velocity, float max_height_offset, out Vector3 fire_velocity, out float gravity, out Vector3 impact_point);
//
//    float ballistic_range(float speed, float gravity, float initial_height);
//
//    bool IsZero(double d);
//    int SolveQuadric(double c0, double c1, double c2, out double s0, out double s1);
//    int SolveCubic(double c0, double c1, double c2, double c3, out double s0, out double s1, out double s2);
//    int SolveQuartic(double c0, double c1, double c2, double c3, double c4, out double s0, out double s1, out double s2, out double s3);
//
// SOURCE
//    https://github.com/forrestthewoods/lib_fts/blob/master/code/fts_ballistic_trajectory.cs
//
//     SolveQuadric, SolveCubic, and SolveQuartic were ported from C as written for Graphics Gems I
//     Original Author: Jochen Schwarze (schwarze@isa.de)
//     https://github.com/erich666/GraphicsGems/blob/240a34f2ad3fa577ef57be74920db6c4b00605e4/gems/Roots3And4.c
//

import android.util.Log;

import java.util.Arrays;
import java.util.Comparator;

public class BallisticTrajectoryUtil {
    private final static String tag = "fts";

    /**
     * Utility function used by SolveQuadratic, SolveCubic, and SolveQuartic
     */
    public static boolean IsZero(double d) {
        final double eps = 1e-9;
        return d > -eps && d < eps;
    }

    /**
     * Class to replace (out double) of C#
     */
    public static class Number {
        private double number;

        public Number(double number) {
            this.number = number;
        }

        public void setNaN() {
            this.number = Double.NaN;
        }

        public void plus(double other) {
            this.number += other;
        }

        public void minus(double other) {
            this.number -= other;
        }

        public void times(double scalar) {
            this.number *= scalar;
        }

        public void div(double scalar) {
            this.number /= scalar;
        }

        public void set(double newNumber) {
            this.number = newNumber;
        }

        public double get() {
            return this.number;
        }

        public static class sorter implements Comparator<Number> {
            public int compare(Number a, Number b) {
                return Double.compare(a.get(), b.get());
            }
        }
    }

    /**
     * Class to replace Unity3d's Vector3
     * @link https://docs.unity3d.com/ScriptReference/Vector3.html
     * (x, y, z) = (left-right, down-up, back-forward)
     */
    public static class Vector3 {
        public double x;
        public double y;
        public double z;
        public static final Vector3 back    = new Vector3(0 , 0 , -1);
        public static final Vector3 down    = new Vector3(0 , -1, 0 );
        public static final Vector3 forward = new Vector3(0 , 0 , 1 );
        public static final Vector3 left    = new Vector3(-1, 0 , 0 );
        public static final Vector3 one     = new Vector3(1 , 1 , 1 );
        public static final Vector3 right   = new Vector3(1 , 0 , 0 );
        public static final Vector3 up      = new Vector3(0 , 1 , 0 );
        public static final Vector3 zero    = new Vector3(0 , 0 , 0 );


        public Vector3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void set(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void set(Vector3 newVector3) {
            set(newVector3.x, newVector3.y, newVector3.z);
        }

        public Vector3 plus(Vector3 other) {
            return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
        }

        public Vector3 minus(Vector3 other) {
            return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
        }

        public Vector3 times(double scalar) {
            return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
        }

        public Vector3 div(double scalar) {
            return new Vector3(this.x/scalar, this.y/scalar, this.z/scalar);
        }

        public static double Dot(Vector3 lhs, Vector3 rhs) {
            return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
        }

        /**
         * @return Length of the vector
         */
        public double getMagnitude() {
            return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
        }

        /**
         * @return Vector when length is 1
         */
        public Vector3 getNormalized() {
            double magnitude = getMagnitude();
            return this.div(magnitude);
        }

        public boolean equals(Vector3 comparison) {
            return this.x == comparison.x && this.y == comparison.y && this.z == comparison.z;
        }
    }

    /**
     * Method to replace Debug.assert()
     * @param ifFalse when this is false
     * @param debugText Log error with this text
     */
    public static void assertion(boolean ifFalse, String debugText) {
        // Handling these cases is up to your project's coding standards
        if (!ifFalse) {
            Log.e(tag, debugText);
        }
    }

    /**
     * Solve quadratic equation: c0*x^2 + c1*x + c2.
     * @param c0 leading coefficient c0*x^2
     * @param c1 second coefficient c1*x^1
     * @param c2 constant c2*x^0
     *
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @return number of solutions.
     */
    public static int SolveQuadric(double c0, double c1, double c2, Number s0, Number s1) {
        s0.setNaN();
        s1.setNaN();

        double p, q, D;

        /* normal form: x^2 + px + q = 0 */
        p = c1 / (2 * c0);
        q = c2 / c0;

        D = p * p - q;

        if (IsZero(D)) {
            s0.set(-p);
            return 1;
        }
        else if (D < 0) {
            return 0;
        }
        else /* if (D > 0) */ {
            double sqrt_D = Math.sqrt(D);

            s0.set(sqrt_D - p);
            s1.set(-sqrt_D - p);
            return 2;
        }
    }

    /**
     * Solve cubic equation: c0*x^3 + c1*x^2 + c2*x + c3.
     * @param c0 leading coefficient c0*x^3
     * @param c1 second coefficient c1*x^2
     * @param c2 third coefficient c2*x^1
     * @param c3 constant c3*x^0
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @param s2 solution 3 reference
     * @return number of solutions.
     */
    public static int SolveCubic(double c0, double c1, double c2, double c3, Number s0, Number s1, Number s2)
    {
        s0.setNaN();
        s1.setNaN();
        s2.setNaN();

        int     num;
        double  sub;
        double  A, B, C;
        double  sq_A, p, q;
        double  cb_p, D;

        /* normal form: x^3 + Ax^2 + Bx + C = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;

        /*  substitute x = y - A/3 to eliminate quadric term:  x^3 +px + q = 0 */
        sq_A = A * A;
        p = 1.0/3 * (-1.0/3 * sq_A + B);
        q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);

        /* use Cardano's formula */
        cb_p = p * p * p;
        D = q * q + cb_p;

        if (IsZero(D)) {
            if (IsZero(q)) /* one triple solution */ {
                s0.set(0);
                num = 1;
            }
            else /* one single and one double solution */ {
                double u = Math.pow(-q, 1.0/3.0);
                s0.set(2 * u);
                s1.set(-u);
                num = 2;
            }
        }
        else if (D < 0) /* Casus irreducibilis: three real solutions */ {
            double phi = 1.0/3 * Math.acos(-q / Math.sqrt(-cb_p));
            double t = 2 * Math.sqrt(-p);

            s0.set( t * Math.cos(phi));
            s1.set(-t * Math.cos(phi + Math.PI / 3));
            s2.set(-t * Math.cos(phi - Math.PI / 3));
            num = 3;
        }
        else /* one real solution */ {
            double sqrt_D = Math.sqrt(D);
            double u =  Math.pow(sqrt_D - q, 1.0/3.0);
            double v = -Math.pow(sqrt_D + q, 1.0/3.0);

            s0.set(u + v);
            num = 1;
        }

        /* resubstitute */
        sub = 1.0/3 * A;

        if (num > 0)    s0.minus(sub);
        if (num > 1)    s1.minus(sub);
        if (num > 2)    s2.minus(sub);

        return num;
    }

    /**
     * Solve quartic function: c0*x^4 + c1*x^3 + c2*x^2 + c3*x + c4.
     * @param c0 leading coefficient c0*x^4
     * @param c1 second coefficient c1*x^3
     * @param c2 third coefficient c2*x^2
     * @param c3 forth coefficient c3*x^1
     * @param c4 constant c4*x^0
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @param s2 solution 3 reference
     * @param s3 solution 4 reference
     * @return number of solutions.
     */
    public static int SolveQuartic(double c0, double c1, double c2, double c3, double c4, Number s0, Number s1, Number s2, Number s3) {
        s0.setNaN();
        s1.setNaN();
        s2.setNaN();
        s3.setNaN();

        double[]  coeffs = new double[4];
        double  z, u, v, sub;
        double  A, B, C, D;
        double  sq_A, p, q, r;
        int     num;

        /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;
        D = c4 / c0;

        /*  substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0 */
        sq_A = A * A;
        p = -3.0/8 * sq_A + B;
        q = 1.0/8 * sq_A * A - 1.0/2 * A * B + C;
        r = -3.0/256*sq_A*sq_A + 1.0/16*sq_A*B - 1.0/4*A*C + D;

        if (IsZero(r)) {
            /* no absolute term: y(y^3 + py + q) = 0 */

            coeffs[ 3 ] = q;
            coeffs[ 2 ] = p;
            coeffs[ 1 ] = 0;
            coeffs[ 0 ] = 1;

            num = SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);
        }
        else {
            /* solve the resolvent cubic ... */
            coeffs[ 3 ] = 1.0/2 * r * p - 1.0/8 * q * q;
            coeffs[ 2 ] = -r;
            coeffs[ 1 ] = -1.0/2 * p;
            coeffs[ 0 ] = 1;

            SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);

            /* ... and take the one real solution ... */
            z = s0.get();

            /* ... to build two quadric equations */
            u = z * z - r;
            v = 2 * z - p;

            if (IsZero(u))
                u = 0;
            else if (u > 0)
                u = Math.sqrt(u);
            else
                return 0;

            if (IsZero(v))
                v = 0;
            else if (v > 0)
                v = Math.sqrt(v);
            else
                return 0;

            coeffs[ 2 ] = z - u;
            coeffs[ 1 ] = q < 0 ? -v : v;
            coeffs[ 0 ] = 1;

            num = SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);

            coeffs[ 2 ]= z + u;
            coeffs[ 1 ] = q < 0 ? v : -v;
            coeffs[ 0 ] = 1;

            if (num == 0) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);
            if (num == 1) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s1, s2);
            if (num == 2) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s2, s3);
        }

        /* resubstitute */
        sub = 1.0/4 * A;

        if (num > 0)    s0.minus(sub);
        if (num > 1)    s1.minus(sub);
        if (num > 2)    s2.minus(sub);
        if (num > 3)    s3.minus(sub);

        return num;
    }

    /**
     * Calculate the maximum range that a ballistic projectile can be fired on given speed and gravity.
     * @param speed projectile velocity
     * @param gravity force of gravity, positive is down
     * @param initial_height distance above flat terrain
     * @return maximum range
     */
    public static double ballistic_range(float speed, float gravity, float initial_height) {

        assertion(speed > 0 && gravity > 0 && initial_height >= 0, "fts.ballistic_range called with invalid data");

        // Derivation
        //   (1) x = speed * time * cos O
        //   (2) y = initial_height + (speed * time * sin O) - (.5 * gravity*time*time)
        //   (3) via quadratic: t = (speed*sin O)/gravity + sqrt(speed*speed*sin O + 2*gravity*initial_height)/gravity    [ignore smaller root]
        //   (4) solution: range = x = (speed*cos O)/gravity * sqrt(speed*speed*sin O + 2*gravity*initial_height)    [plug t back into x=speed*time*cos O]
        double angle = Math.toRadians(45); // no air resistance, so 45 degrees provides maximum range
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);

        double range = (speed*cos/gravity) * (speed*sin + Math.sqrt(speed*speed*sin*sin + 2*gravity*initial_height));
        return range;
    }

    /**
     * Solve firing angles for a ballistic projectile with speed and gravity to hit a fixed position.
     * @param proj_pos point projectile will fire from
     * @param proj_speed scalar speed of projectile
     * @param target point projectile is trying to hit
     * @param gravity force of gravity, positive down
     * @param s0 firing solution (low angle) reference
     * @param s1 firing solution (high angle) reference
     * @return number of unique solutions found: 0, 1, or 2.
     */
    public static int solve_ballistic_arc(Vector3 proj_pos, float proj_speed, Vector3 target, float gravity, Vector3 s0, Vector3 s1) {

        assertion(!proj_pos.equals(target) && proj_speed > 0 && gravity > 0, "fts.solve_ballistic_arc called with invalid data");

        // C# requires out variables be set
        s0.set(Vector3.zero);
        s1.set(Vector3.zero);

        // Derivation
        //   (1) x = v*t*cos O
        //   (2) y = v*t*sin O - .5*g*t^2
        //
        //   (3) t = x/(cos O*v)                                        [solve t from (1)]
        //   (4) y = v*x*sin O/(cos O * v) - .5*g*x^2/(cos^2 O*v^2)     [plug t into y=...]
        //   (5) y = x*tan O - g*x^2/(2*v^2*cos^2 O)                    [reduce; cos/sin = tan]
        //   (6) y = x*tan O - (g*x^2/(2*v^2))*(1+tan^2 O)              [reduce; 1+tan O = 1/cos^2 O]
        //   (7) 0 = ((-g*x^2)/(2*v^2))*tan^2 O + x*tan O - (g*x^2)/(2*v^2) - y    [re-arrange]
        //   Quadratic! a*p^2 + b*p + c where p = tan O
        //
        //   (8) let gxv = -g*x*x/(2*v*v)
        //   (9) p = (-x +- sqrt(x*x - 4gxv*(gxv - y)))/2*gxv           [quadratic formula]
        //   (10) p = (v^2 +- sqrt(v^4 - g(g*x^2 + 2*y*v^2)))/gx        [multiply top/bottom by -2*v*v/x; move 4*v^4/x^2 into root]
        //   (11) O = atan(p)

        Vector3 diff = target.minus(proj_pos);
        Vector3 diffXZ = new Vector3(diff.x, 0f, diff.z);
        double groundDist = diffXZ.getMagnitude();

        double speed2 = proj_speed*proj_speed;
        double speed4 = proj_speed*proj_speed*proj_speed*proj_speed;
        double y = diff.y;
        double x = groundDist;
        double gx = gravity*x;

        double root = speed4 - gravity*(gravity*x*x + 2*y*speed2);

        // No solution
        if (root < 0)
            return 0;

        root = Math.sqrt(root);

        double lowAng = Math.atan2(speed2 - root, gx);
        double highAng = Math.atan2(speed2 + root, gx);
        int numSolutions = lowAng != highAng ? 2 : 1;

        Vector3 groundDir = diffXZ.getNormalized();
        s0.set(groundDir.times(Math.cos(lowAng)*proj_speed).plus(Vector3.up.times(Math.sin(lowAng)*proj_speed)));
        if (numSolutions > 1)
            s1.set(groundDir.times(Math.cos(highAng)*proj_speed).plus(Vector3.up.times(Math.sin(highAng)*proj_speed)));

        return numSolutions;
    }

    /**
     * Solve firing angles for a ballistic projectile with speed and gravity to hit a target moving with constant, linear velocity.
     * @param proj_pos point projectile will fire from
     * @param proj_speed scalar speed of projectile
     * @param target_pos point projectile is trying to hit
     * @param target_velocity velocity of target
     * @param gravity force of gravity, positive down
     * @param s0 firing solution (fastest time impact) reference
     * @param s1 firing solution (next impact) reference
     * -param s2 firing solution (next impact) reference?
     * -param s3 firing solution (next impact) reference?
     * @return number of unique solutions found: 0, 1, 2, 3, or 4.
     */
    public static int solve_ballistic_arc(Vector3 proj_pos, double proj_speed, Vector3 target_pos, Vector3 target_velocity, double gravity, Vector3 s0, Vector3 s1) {

        // Initialize output parameters
        s0.set(Vector3.zero);
        s1.set(Vector3.zero);

        // Derivation
        //
        //  For full derivation see: blog.forrestthewoods.com
        //  Here is an abbreviated version.
        //
        //  Four equations, four unknowns (solution.x, solution.y, solution.z, time):
        //
        //  (1) proj_pos.x + solution.x*time = target_pos.x + target_vel.x*time
        //  (2) proj_pos.y + solution.y*time + .5*G*t = target_pos.y + target_vel.y*time
        //  (3) proj_pos.z + solution.z*time = target_pos.z + target_vel.z*time
        //  (4) proj_speed^2 = solution.x^2 + solution.y^2 + solution.z^2
        //
        //  (5) Solve for solution.x and solution.z in equations (1) and (3)
        //  (6) Square solution.x and solution.z from (5)
        //  (7) Solve solution.y^2 by plugging (6) into (4)
        //  (8) Solve solution.y by rearranging (2)
        //  (9) Square (8)
        //  (10) Set (8) = (7). All solution.xyz terms should be gone. Only time remains.
        //  (11) Rearrange 10. It will be of the form a*^4 + b*t^3 + c*t^2 + d*t * e. This is a quartic.
        //  (12) Solve the quartic using SolveQuartic.
        //  (13) If there are no positive, real roots there is no solution.
        //  (14) Each positive, real root is one valid solution
        //  (15) Plug each time value into (1) (2) and (3) to calculate solution.xyz
        //  (16) The end.

        double G = gravity;

        double A = proj_pos.x;
        double B = proj_pos.y;
        double C = proj_pos.z;
        double M = target_pos.x;
        double N = target_pos.y;
        double O = target_pos.z;
        double P = target_velocity.x;
        double Q = target_velocity.y;
        double R = target_velocity.z;
        double S = proj_speed;

        double H = M - A;
        double J = O - C;
        double K = N - B;
        double L = -.5f * G;

        // Quartic Coeffecients
        double c0 = L*L;
        double c1 = 2*Q*L;
        double c2 = Q*Q + 2*K*L - S*S + P*P + R*R;
        double c3 = 2*K*Q + 2*H*P + 2*J*R;
        double c4 = K*K + H*H + J*J;

        // Solve quartic
        Number[] times = new Number[4];
        int numTimes = SolveQuartic(c0, c1, c2, c3, c4, times[0], times[1], times[2], times[3]);

        // Sort so faster collision is found first
        Arrays.sort(times, new Number.sorter());

        // Plug quartic solutions into base equations
        // There should never be more than 2 positive, real roots.
        Vector3[] solutions = new Vector3[2];
        int numSolutions = 0;

        for (int i = 0; i < numTimes && numSolutions < 2; i++) {
            double t = times[i].get();
            if (t <= 0)
                continue;

            solutions[numSolutions].set(
                    (float)((H+P*t)/t),
                    (float)((K+Q*t-L*t*t)/ t),
                    (float)((J+R*t)/t));
            numSolutions++;
        }

        // Write out solutions
        if (numSolutions > 0)   s0.set(solutions[0]);
        if (numSolutions > 1)   s1.set(solutions[1]);

        return numSolutions;
    }



    /**
     * Solve the firing arc with a fixed lateral speed. Vertical speed and gravity varies.
     * This enables a visually pleasing arc.
     * @param proj_pos point projectile will fire from
     * @param lateral_speed scalar speed of projectile along XZ plane
     * @param target_pos point projectile is trying to hit
     * @param max_height height above Max(proj_pos, impact_pos) for projectile to peak at
     * @param fire_velocity firing velocity reference
     * @param gravity gravity necessary to projectile to hit precisely max_height reference
     * @return true if a valid solution was found
     */
    public static boolean solve_ballistic_arc_lateral(Vector3 proj_pos, double lateral_speed, Vector3 target_pos, double max_height, Vector3 fire_velocity, Number gravity) {

        assertion(!proj_pos.equals(target_pos) && lateral_speed > 0 && max_height > proj_pos.y, "fts.solve_ballistic_arc called with invalid data");

        fire_velocity.set(Vector3.zero);
        gravity.setNaN();

        Vector3 diff = target_pos.minus(proj_pos);
        Vector3 diffXZ = new Vector3(diff.x, 0f, diff.z);
        double lateralDist = diffXZ.getMagnitude();

        if (lateralDist == 0)
            return false;

        double time = lateralDist / lateral_speed;

        fire_velocity.set(diffXZ.getNormalized().times(lateral_speed));

        // System of equations. Hit max_height at t=.5*time. Hit target at t=time.
        //
        // peak = y0 + vertical_speed*halfTime + .5*gravity*halfTime^2
        // end = y0 + vertical_speed*time + .5*gravity*time^s
        // Wolfram Alpha: solve b = a + .5*v*t + .5*g*(.5*t)^2, c = a + vt + .5*g*t^2 for g, v
        double a = proj_pos.y;       // initial
        double b = max_height;       // peak
        double c = target_pos.y;     // final

        gravity.set(-4*(a - 2*b + c) / (time* time));
        fire_velocity.y = -(3*a - 4*b + c) / time;

        return true;
    }

    /**
     * Solve the firing arc with a fixed lateral speed. Vertical speed and gravity varies.
     * This enables a visually pleasing arc.
     * @param proj_pos point projectile will fire from
     * @param lateral_speed scalar speed of projectile along XZ plane
     * @param target point projectile is trying to hit
     * @param target_velocity height above Max(proj_pos, impact_pos) for projectile to peak at
     * @param max_height_offset firing velocity
     * @param fire_velocity gravity necessary to projectile to hit precisely max_height reference
     * @param gravity point where moving target will be hit reference
     * @param impact_point point where moving target will be hit reference
     * @return true if a valid solution was found
     */
    public static boolean solve_ballistic_arc_lateral(Vector3 proj_pos, float lateral_speed, Vector3 target, Vector3 target_velocity, float max_height_offset, Vector3 fire_velocity, Number gravity, Vector3 impact_point) {

        assertion(!proj_pos.equals(target) && lateral_speed > 0, "fts.solve_ballistic_arc_lateral called with invalid data");

        // Initialize output variables
        fire_velocity.set(Vector3.zero);
        gravity.set(0f);
        impact_point.set(Vector3.zero);

        // Ground plane terms
        Vector3 targetVelXZ = new Vector3(target_velocity.x, 0f, target_velocity.z);
        Vector3 diffXZ = target.minus(proj_pos);
        diffXZ.y = 0;

        // Derivation
        //   (1) Base formula: |P + V*t| = S*t
        //   (2) Substitute variables: |diffXZ + targetVelXZ*t| = S*t
        //   (3) Square both sides: Dot(diffXZ,diffXZ) + 2*Dot(diffXZ, targetVelXZ)*t + Dot(targetVelXZ, targetVelXZ)*t^2 = S^2 * t^2
        //   (4) Quadratic: (Dot(targetVelXZ,targetVelXZ) - S^2)t^2 + (2*Dot(diffXZ, targetVelXZ))*t + Dot(diffXZ, diffXZ) = 0
        double c0 = Vector3.Dot(targetVelXZ, targetVelXZ) - lateral_speed*lateral_speed;
        double c1 = 2f * Vector3.Dot(diffXZ, targetVelXZ);
        double c2 = Vector3.Dot(diffXZ, diffXZ);
        Number t0 = new Number(0);
        Number t1 = new Number(0);
        int n = SolveQuadric(c0, c1, c2, t0, t1);

        // pick smallest, positive time
        boolean valid0 = n > 0 && t0.get() > 0;
        boolean valid1 = n > 1 && t1.get() > 0;

        double t;
        if (!valid0 && !valid1)
            return false;
        else if (valid0 && valid1)
            t = Math.min(t0.get(), t1.get());
        else
            t = valid0 ? t0.get() : t1.get();

        // Calculate impact point
        impact_point = target.plus(target_velocity.times(t));

        // Calculate fire velocity along XZ plane
        Vector3 dir = impact_point.minus(proj_pos);
        fire_velocity = new Vector3(dir.x, 0f, dir.z).getNormalized().times(lateral_speed);

        // Solve system of equations. Hit max_height at t=.5*time. Hit target at t=time.
        //
        // peak = y0 + vertical_speed*halfTime + .5*gravity*halfTime^2
        // end = y0 + vertical_speed*time + .5*gravity*time^s
        // Wolfram Alpha: solve b = a + .5*v*t + .5*g*(.5*t)^2, c = a + vt + .5*g*t^2 for g, v
        double a = proj_pos.y;       // initial
        double b = Math.max(proj_pos.y, impact_point.y) + max_height_offset;  // peak
        double c = impact_point.y;   // final

        gravity.set(-4*(a - 2*b + c) / (t* t));
        fire_velocity.y = -(3*a - 4*b + c) / t;

        return true;
    }
}