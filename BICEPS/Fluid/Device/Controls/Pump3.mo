within BICEPS.Fluid.Device.Controls;
model Pump3 "Pump control - on/off"
  extends Modelica.Blocks.Icons.Block;
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Modelica.SIunits.Temperature TMin=288.15
    "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=298.15
    "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=293.15
    "Nominal value for independent variable";
  parameter Modelica.SIunits.TemperatureDifference dT=1
    "Temperature deadband for complete linear transition";
  parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
  parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
  Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Utilities.Math.CubicHermiteInverse spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0) if biomimeticControl
    "Spline to inversely calculate pulsing setpoint from control signal"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(
    final uMax=TMax,
    final uMin=TMin) if biomimeticControl
    "Temperature setpoint if biomimetic control"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Blocks.Interfaces.RealInput TSetSta if not biomimeticControl
    "Static temperature setpoint"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput TMea
    "Measured temperature"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
    "Control signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

  Buildings.Controls.OBC.CDL.Logical.OnOffController onOffCon(bandwidth=dT)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Blocks.Math.BooleanToReal booToRea(realFalse=0.02)
    "Convert boolean to real"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  connect(spl.x, TSet.u)
    annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
  connect(y, spl.y)
    annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
  connect(TMea, onOffCon.u) annotation (Line(points={{-120,-60},{0,-60},{0,-6},
          {18,-6}}, color={0,0,127}));
  connect(TSet.y, onOffCon.reference)
    annotation (Line(points={{-18,0},{0,0},{0,6},{18,6}}, color={0,0,127}));
  connect(TSetSta, onOffCon.reference)
    annotation (Line(points={{-120,60},{0,60},{0,6},{18,6}}, color={0,0,127}));
  connect(onOffCon.y, booToRea.u)
    annotation (Line(points={{42,0},{58,0}}, color={255,0,255}));
  connect(booToRea.y, yOut)
    annotation (Line(points={{81,0},{92,0},{92,0},{110,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Pump3;
