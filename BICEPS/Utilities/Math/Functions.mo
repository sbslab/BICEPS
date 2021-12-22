within BICEPS.Utilities.Math;
package Functions "Package with mathematical functions"
  extends Modelica.Icons.VariantsPackage;

  function inverseMonotonicCubicHermite
    "Calculates the analytic inverse of a monotonic cubic hermite function"
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
    Real t[:,2] "abscissa scaled with h, i.e., t=[0..1] within x=[x1..x2]";
    Real p1 "Polynomial coefficient 1";
    Real p2 "Polynomial coefficient 2";
    Real p3 "Polynomial coefficient 3";
    Real p4 "Polynomial coefficient 4";
  algorithm
  /*  h := x2 - x1;
  if abs(h)>0 then
    // Regular case
    t := (x - x1)/h;
    aux3 :=t^3;
    aux2 :=t^2;
    h00 := 2*aux3 - 3*aux2 + 1;
    h10 := aux3 - 2*aux2 + t;
    h01 := -2*aux3 + 3*aux2;
    h11 := aux3 - aux2;
    y := y1*h00 + h*y1d*h10 + y2*h01 + h*y2d*h11;
  else
    // Degenerate case, x1 and x2 are identical, return step function
    y := (y1 + y2)/2; 
  end if;*/

    if (y > y1 and y < y2) then
      h := x2 - x1;
      if y == 0 and y1 == 0 then
        x := x1;
      elseif y == 0 and y2 == 0 then
        x := x2;
      else
        if abs(h)>0 then
          p1 := -2*(y2 - y1) + (x2 - x1)*(y2d + y1d);
          p2 := 3*(y2 - y1) - (x2 - x1)*(y2d + 2*y1d);
          p3 := (x2 - x1)*y1d;
          p4 := y1 - y;
          t := Modelica.Math.Vectors.Utilities.roots({p1,p2,p3,p4});
          x := h*t[1,1] + x1;
        // Regular case
        // New roots function
        else
        // Degenerate case, x1 and x2 are identical, return step function
        x := (x1 + x2)/2;
        end if;
      end if;
    elseif y <= y1 then
      // linear extrapolation
      x:=y/y1d - y1 + x1;
    else
      x:=y/y2d - y2 + x2;
    end if;
  end inverseMonotonicCubicHermite;
end Functions;
