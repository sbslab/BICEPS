within BICEPS.Experimental.Examples.Controls;
model HeatPump "Heat pump controller"
 extends Modelica.Blocks.Icons.Block;
  parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Real T0=273.15+20 "Nominal value for independent variable";
 parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
 parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
  Modelica.Blocks.Interfaces.RealInput yEle
    "Electrical subsystem relative exergy potential"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealOutput TSet(
    final quantity="ThermodynamicTemperature",
    final unit = "K",
    min=0,
    displayUnit = "degC") "Temperature setpoint"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput yHeaPum
    "Heat pump relative exergy potential"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Utilities.Math.CubicHermiteInverse spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0,
    final ensureMonotonicity=true)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Blocks.Math.Add add(k1=a, k2=b)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Blocks.Math.Gain nor(k=1/(a + b)) "Normalized"
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter lim(
    final uMax=TMax,
    final uMin=TMin)
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  connect(yEle, add.u1) annotation (Line(points={{-120,60},{-80,60},{-80,6},{-62,
          6}}, color={0,0,127}));
  connect(yHeaPum, add.u2) annotation (Line(points={{-120,-60},{-80,-60},{-80,-6},
          {-62,-6}}, color={0,0,127}));
  connect(add.y, nor.u)
    annotation (Line(points={{-39,0},{-22,0}}, color={0,0,127}));
  connect(nor.y, spl.y)
    annotation (Line(points={{1,0},{18,0}}, color={0,0,127}));
  connect(spl.x, lim.u)
    annotation (Line(points={{41,0},{58,0}}, color={0,0,127}));
  connect(lim.y, TSet)
    annotation (Line(points={{82,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPump;
