within BICEPS.Experimental.Examples.Sensors;
model RelativeInternalExergyPotential
  "Sensor for relative internal exergy"
  extends Modelica.Icons.RotationalSensor;
  extends Buildings.Fluid.Interfaces.PartialTwoPort;
  parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
  parameter Real k(min=Modelica.Constants.small)=10
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)
    "Nominal mass flow rate, used for regularization near zero flow"
    annotation(Dialog(group = "Nominal condition"));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Utilities.Math.CubicHermite spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0)
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
  Modelica.Blocks.Interfaces.RealOutput y "Control signal"
    annotation (Placement(transformation(extent={{0,100},{20,120}}),
        iconTransformation(
        extent={{100,70},{120,90}})));
equation
  connect(port_a, senTem.port_a)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,127,255}));
  connect(senTem.port_b, port_b)
    annotation (Line(points={{-40,0},{100,0}}, color={0,127,255}));
  connect(senTem.T, spl.x)
    annotation (Line(points={{-50,11},{-50,70},{-22,70}}, color={0,0,127}));
  connect(spl.y, y)
    annotation (Line(points={{1,70},{6,70},{6,110},{10,110}},
                                               color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-100,0},{-70,0}}, color={28,108,200}),
        Line(points={{70,0},{100,0}}, color={28,108,200}),
        Text(
          extent={{60,-18},{-60,-68}},
          lineColor={0,0,0},
          textString="T"),
        Line(points={{18,58},{24,80}}, color={0,0,0}),
                           Line(points={{100,80},{24,80}},
                                                         color={0,0,0})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RelativeInternalExergyPotential;
