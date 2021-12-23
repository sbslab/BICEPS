within BICEPS.Utilities.Math;
model CubicHermiteInverse
  "Block for the inverse function of a strictly monotonic cubic hermite spline"
  extends Modelica.Blocks.Icons.Block;
  parameter Real xMin=-1 "Minimimum desired threshold for independent variable";
  parameter Real xMax=1 "Maximum desired threshold for independent variable";
  parameter Real x0=0 "Nominal value for independent variable";
  final parameter Boolean ensureMonotonicity=true "Set to true if spline monoticity is ensured";
  parameter Boolean reverseActing=false "Set to true if xMin=-1 corresponds to yMax";
  final parameter Real[3] xd={xMin,x0,xMax} "Support points";
  final parameter Real[3] yd={-1,0,1}   "Support points";
  final parameter Real[3] d(each fixed=false)
    "Derivatives at the support points";
  Integer i "Integer to select data interval";
  Modelica.Blocks.Interfaces.RealInput y "Independent variable"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}}),
      iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput x "Dependent variable" annotation (
      Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(
          extent={{100,-10},{120,10}})));
initial algorithm
  // Get the derivative values at the support points
  d := Buildings.Utilities.Math.Functions.splineDerivatives(
    x=xd,
    y=yd,
    ensureMonotonicity=ensureMonotonicity);
algorithm
  i := 1;
  for j in 1:size(yd, 1) - 1 loop
    if y > yd[j] then
      i := j;
    end if;
  end for;
  // Extrapolate or interpolate the data
  x := BICEPS.Utilities.Math.Functions.inverseMonotonicCubicHermite(
    y=y,
    x1=xd[i],
    x2=xd[i + 1],
    y1=yd[i],
    y2=yd[i + 1],
    y1d=d[i],
    y2d=d[i + 1]);
  if reverseActing then
    x := 2*x0 - x;
  else
    x := x;
  end if;
  annotation (
    defaultComponentName="spl",
    Icon(coordinateSystem(preserveAspectRatio=false),
    graphics={
    Line(points={{40,-76},{40,58}}, color={192,192,192}),
    Line(points={{0,-76},{0,78}}, color={192,192,192}),
    Text(
      extent={{-35,94},{-6,76}},
      lineColor={160,160,164},
      textString="y"),
    Polygon(
      points={{0,88},{-6,72},{6,72},{0,88}},
      lineColor={192,192,192},
      fillColor={192,192,192},
      fillPattern=FillPattern.Solid),
    Text(
      extent={{26,-76},{52,-92}},
      lineColor={160,160,164},
      textString="xMax"),
    Text(
      extent={{-54,-78},{-28,-94}},
      lineColor={160,160,164},
      textString="xMin"),
    Line(points={{-40,-76},{-40,58}}, color={192,192,192}),
    Text(
      extent={{-12,-78},{14,-94}},
      lineColor={160,160,164},
      textString="x0"),
    Line(points={{-80,-2},{80,-2}}, color={192,192,192}),
    Line(
      points={{-50,-60},{-40,-38},{-28,-10},{0,-2},{28,6},{40,40},{48,62}},
      color={0,0,0},
      smooth=Smooth.Bezier),
    Line(points={{40,40},{0,40}}, color={192,192,192}),
    Line(points={{0,-38},{-40,-38}}, color={192,192,192}),
    Text(
      extent={{-20,48},{6,32}},
      lineColor={160,160,164},
      textString="1"),
    Text(
      extent={{-4,-30},{22,-46}},
      lineColor={160,160,164},
      textString="-1"),
        Ellipse(
          extent={{-4,2},{4,-6}},
          lineColor={238,46,47},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
    Text(
      extent={{-87,38},{-6,-16}},
      lineColor={0,0,0},
          textString="p1*x^3+...+p4=0")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)));
end CubicHermiteInverse;
