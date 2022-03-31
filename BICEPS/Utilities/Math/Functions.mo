within BICEPS.Utilities.Math;
package Functions "Package with mathematical functions"
  extends Modelica.Icons.VariantsPackage;

  function inverseMonotonicCubicHermite
    "Calculates the analytic inverse of a strictly monotonic cubic hermite function"
    extends Modelica.Icons.Function;
    input Real y "Ordinate value";
    input Real x1 "Lower abscissa value";
    input Real x2 "Upper abscissa value";
    input Real y1 "Lower ordinate value";
    input Real y2 "Upper ordinate value";
    input Real y1d "Lower gradient";
    input Real y2d "Upper gradient";
    output Real x "Interpolated abscissa value";
  protected
    Real h "Distance between x1 and x2";
    Real t[3,2] "abscissa scaled with h, i.e., t=[0..1] within x=[x1..x2]";
    Real p1 "Polynomial coefficient 1";
    Real p2 "Polynomial coefficient 2";
    Real p3 "Polynomial coefficient 3";
    Real p4 "Polynomial coefficient 4";
  algorithm
    if (y > y1 and y < y2 and abs(y2d-y1d)>0) then
      h := x2 - x1;
      if abs(h)>0 then
        p1 := -2*(y2 - y1) + h*(y2d + y1d);
        p2 := 3*(y2 - y1) - h*(y2d + 2*y1d);
        p3 := h*y1d;
        p4 := y1 - y;
        t := Modelica.Math.Vectors.Utilities.roots({p1,p2,p3,p4});
        x := h*t[3,1] + x1;
      else
      // Degenerate case, x1 and x2 are identical, return step function
      x := (x1 + x2)/2;
      end if;
    elseif y <= y1 then
      // linear extrapolation
      x:=(y - y1)/y1d + x1;
    else
      x:=(y - y2)/y2d + x2;
    end if;
  end inverseMonotonicCubicHermite;

  function standardDeviationPopulation "The population standard deviation, 
  used if the inputs represent the entire population of the data set"

  algorithm

  end standardDeviationPopulation;

  function standardDeviationSample "The sample standard deviation, 
  used if the inputs represent a sample of the entire population"

  algorithm

  end standardDeviationSample;

  function variance "The variance of the input data"

  algorithm

  end variance;
end Functions;
