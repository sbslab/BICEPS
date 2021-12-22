within BICEPS.Utilities.Math.Examples;
model CubicHermiteInverse "Test problem for cubic hermite splines"
  extends Modelica.Icons.Example;
  Modelica.Blocks.Sources.Sine y(amplitude=2, freqHz(displayUnit="Hz") = 1)
    "Control signal"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  BICEPS.Utilities.Math.CubicHermiteInverse2 spl(
    xMin=13,
    xMax=25,
    x0=20) annotation (Placement(transformation(extent={{0,0},{20,20}})));
  BICEPS.Utilities.Math.CubicHermiteInverse2 splRev(
    xMin=13,
    xMax=25,
    x0=20,
    reverseActing=true)
    annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
equation
  connect(y.y, spl.y)
    annotation (Line(points={{-19,10},{-2,10}}, color={0,0,127}));
  connect(y.y, splRev.y) annotation (Line(points={{-19,10},{-10,10},{-10,-30},{
          -2,-30}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(Tolerance=1e-06, __Dymola_Algorithm="Dassl"));
end CubicHermiteInverse;
