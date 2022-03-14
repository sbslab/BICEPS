within BICEPS.Fluid.Equipment.Controls;
model Pump "Pump control"
  extends Modelica.Blocks.Icons.Block;
  parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Real T0=273.15+20 "Nominal value for independent variable";
  parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
  parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
  Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Utilities.Math.CubicHermiteInverse spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0)
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(final uMax=TMax, final
      uMin=TMin)
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Blocks.Interfaces.RealInput TMea "Measured temperature"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealInput y "Control signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Buildings.Controls.Continuous.LimPID conPID(k=0.1, Ti=60,
    yMin=1e-3)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation
  connect(spl.x, TSet.u)
    annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
  connect(y, spl.y)
    annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
  connect(TSet.y, conPID.u_s)
    annotation (Line(points={{-18,0},{38,0}}, color={0,0,127}));
  connect(TMea, conPID.u_m)
    annotation (Line(points={{-120,-60},{50,-60},{50,-12}}, color={0,0,127}));
  connect(conPID.y, yOut)
    annotation (Line(points={{61,0},{80,0},{80,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Pump;
