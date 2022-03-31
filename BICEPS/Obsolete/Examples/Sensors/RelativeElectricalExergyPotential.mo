within BICEPS.Obsolete.Examples.Sensors;
model RelativeElectricalExergyPotential
  "Sensor for relative electric potential exergy"
  extends Modelica.Icons.ObsoleteModel;
  extends Modelica.Icons.RotationalSensor;
  extends Buildings.BaseClasses.BaseIcon;
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Modelica.SIunits.Voltage v0=208 "Nominal value for independent variable";
  parameter Real k(min=Modelica.Constants.small)=10
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  final parameter Modelica.SIunits.Voltage vMin=v0*(1-tol) "Minimimum desired threshold for independent variable";
  final parameter Modelica.SIunits.Voltage vMax=v0*(1+tol) "Maximum desired threshold for independent variable";
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-100}),     iconTransformation(extent={{-10,-110},{10,-90}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sensors.Probe sen(
    final V_nominal=v0,
    final perUnit=false)
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  Modelica.Blocks.Interfaces.RealOutput y "Control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Utilities.Math.CubicHermite spl(
    final xMin=vMin,
    final xMax=vMax,
    final x0=v0)
    annotation (Placement(transformation(extent={{60,70},{80,90}})));
equation
  connect(terminal, terminal)
    annotation (Line(points={{0,-100},{0,-100}},
                                               color={0,120,120}));
  connect(terminal, sen.term)
    annotation (Line(points={{0,-100},{0,21}},          color={0,120,120}));
  connect(sen.V, spl.x) annotation (Line(points={{7,33},{50,33},{50,80},{58,80}},
        color={0,0,127}));
  connect(spl.y, y)
    annotation (Line(points={{81,80},{110,80}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{60,-16},{-60,-66}},
          lineColor={0,0,0},
          textString="V"),
        Line(points={{18,58},{26,80}}, color={0,0,0}),
                           Line(points={{100,80},{26,80}},
                                                         color={0,0,0}),
        Line(points={{0,-70},{0,-90}}, color={0,0,0})}),         Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RelativeElectricalExergyPotential;
