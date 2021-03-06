within BICEPS.Utilities.Math.Examples;
model CubicHermite "Test problem for cubic hermite splines"
  extends Modelica.Icons.Example;
  BICEPS.Utilities.Math.CubicHermite spl(
    xMin=273.15 + 17,
    xMax=273.15 + 24,
    x0=273.15 + 20) "Spline function"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Modelica.Blocks.Sources.Sine TMea(
    amplitude=6,
    freqHz(displayUnit="Hz") = 1,
    offset=273.15 + 20) "Measured temperature"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
equation
  connect(TMea.y, spl.x)
    annotation (Line(points={{-19,10},{-2,10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CubicHermite;
