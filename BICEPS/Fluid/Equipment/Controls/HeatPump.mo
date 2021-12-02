within BICEPS.Fluid.Equipment.Controls;
model HeatPump "Heat pump control"
  extends Modelica.Blocks.Icons.Block;
  parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Real T0=273.15+20 "Nominal value for independent variable";
  parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
  parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
  Modelica.Blocks.Interfaces.RealOutput TSet "Setpoint temperature"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput TConEnt
    "Measured entering condenser water temperature"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Buildings.Controls.OBC.CDL.Logical.Switch enaHeaPum(u2(start=false))
    "Enable heat pump by switching to actual set point"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Blocks.Interfaces.BooleanInput uEna "Enable signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Utilities.Math.CubicHermiteInverse spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0,
    final ensureMonotonicity=true)
    annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter lim(final uMax=TMax, final uMin=
       TMin)
    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
  Modelica.Blocks.Interfaces.RealInput y "Control signal"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
equation
  connect(spl.x,lim. u)
    annotation (Line(points={{-59,60},{-42,60}},
                                             color={0,0,127}));
  connect(uEna, enaHeaPum.u2)
    annotation (Line(points={{-120,0},{38,0}}, color={255,0,255}));
  connect(TConEnt, enaHeaPum.u3) annotation (Line(points={{-120,-60},{0,-60},{0,
          -8},{38,-8}}, color={0,0,127}));
  connect(lim.y, enaHeaPum.u1) annotation (Line(points={{-18,60},{0,60},{0,8},{
          38,8}},                 color={0,0,127}));
  connect(enaHeaPum.y, TSet)
    annotation (Line(points={{62,0},{110,0}}, color={0,0,127}));
  connect(y, spl.y)
    annotation (Line(points={{-120,60},{-82,60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPump;
