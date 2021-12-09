within BICEPS.Fluid.Equipment.Controls;
model Pump2 "Pump control"
  extends Modelica.Blocks.Icons.Block;
  parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Real T0=273.15+20 "Nominal value for independent variable";
  parameter Real dT=0.5 "Temperature deadband for complete linear transition";
  parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
  parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
  Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Utilities.Math.CubicHermiteInverse spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0,
    final ensureMonotonicity=true)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(final uMax=TMax, final
      uMin=TMin)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Blocks.Interfaces.RealInput TMea "Measured temperature"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealInput y "Control signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Buildings.Controls.OBC.CDL.Continuous.Line lin
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Blocks.Sources.Constant off(k=0.01) "Off signal"
    annotation (Placement(transformation(extent={{0,-50},{20,-30}})));
  Modelica.Blocks.Sources.Constant on(k=1) "On signal"
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
  Modelica.Blocks.Sources.Constant dTSet(k=dT*2) "Transition zone"
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
  Modelica.Blocks.Math.Add x1(k1=-1) "First transition point"
    annotation (Placement(transformation(extent={{0,10},{20,30}})));
equation
  connect(spl.x, TSet.u)
    annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
  connect(y, spl.y)
    annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
  connect(TMea, lin.u) annotation (Line(points={{-120,-60},{50,-60},{50,0},{58,0}},
        color={0,0,127}));
  connect(off.y, lin.f2) annotation (Line(points={{21,-40},{40,-40},{40,-8},{58,
          -8}},
        color={0,0,127}));
  connect(TSet.y, x1.u2) annotation (Line(points={{-18,0},{-10,0},{-10,14},{-2,14}},
                color={0,0,127}));
  connect(dTSet.y, x1.u1) annotation (Line(points={{-19,40},{-14,40},{-14,26},{-2,
          26}}, color={0,0,127}));
  connect(x1.y, lin.x1)
    annotation (Line(points={{21,20},{30,20},{30,8},{58,8}},color={0,0,127}));
  connect(on.y, lin.f1)
    annotation (Line(points={{21,60},{40,60},{40,4},{58,4}},color={0,0,127}));
  connect(TSet.y, lin.x2) annotation (Line(points={{-18,0},{30,0},{30,-4},{58,-4}},
        color={0,0,127}));
  connect(lin.y, yOut)
    annotation (Line(points={{82,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Pump2;
