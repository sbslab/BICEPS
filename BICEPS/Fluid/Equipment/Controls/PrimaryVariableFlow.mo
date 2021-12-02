within BICEPS.Fluid.Equipment.Controls;
block PrimaryVariableFlow
  "Ideal control of condenser or evaporator variable flow rate"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
    "Heat flow rate at nominal conditions (>0 for condenser)";
  parameter Modelica.SIunits.TemperatureDifference dT_nominal(
    min=if Q_flow_nominal>0 then Modelica.Constants.eps else -100,
    max=if Q_flow_nominal<0 then -Modelica.Constants.eps else 100)
    "DeltaT at nominal conditions (>0 for condenser)";
  parameter Real ratFloMin(
    final unit="1",
    final min=0,
    final max=1)=0.3
    "Minimum mass flow rate (ratio to nominal)";
  constant Modelica.SIunits.SpecificHeatCapacity cp=
    Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
    "Specific heat capacity";
  final parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)=
    Q_flow_nominal/cp/dT_nominal
    "Mass flow rate at nominal conditions";
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput m_flow(final unit="kg/s")
    "Mass flow rate"
    annotation (Placement(transformation(extent={{100,-20},{140,20}}),
      iconTransformation(extent={{100,-20},{140,20}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant masFloMin(
    final k=ratFloMin*m_flow_nominal)
    "Minimum mass flow rate"
    annotation (Placement(transformation(extent={{-30,30},{-10,50}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain masFlo_dT(
    final k=1/cp/dT_nominal)
    "Mass flow rate for constant DeltaT"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Max masFlo "Mass flow rate"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Buildings.Controls.OBC.CDL.Continuous.Abs abs1 "Absolute value"
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
    annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
  Buildings.Controls.OBC.CDL.Continuous.Product flo
    "Zero flow rate if not enabled"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Blocks.Interfaces.BooleanInput u "Enable signal"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Interfaces.RealInput loa "Load"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
equation
  connect(masFloMin.y, masFlo.u1) annotation (Line(points={{-8,40},{0,40},{0,6},
          {18,6}},    color={0,0,127}));
  connect(masFlo_dT.y, abs1.u)
    annotation (Line(points={{-58,0},{-32,0}}, color={0,0,127}));
  connect(abs1.y, masFlo.u2)
    annotation (Line(points={{-8,0},{0,0},{0,-6},{18,-6}},   color={0,0,127}));
  connect(masFlo.y, flo.u2)
    annotation (Line(points={{42,0},{50,0},{50,-6},{58,-6}}, color={0,0,127}));
  connect(booToRea.y, flo.u1) annotation (Line(points={{-58,80},{50,80},{50,6},{
          58,6}}, color={0,0,127}));
  connect(flo.y, m_flow)
    annotation (Line(points={{82,0},{120,0}}, color={0,0,127}));
  connect(booToRea.u, u)
    annotation (Line(points={{-82,80},{-120,80}}, color={255,0,255}));
  connect(masFlo_dT.u, loa)
    annotation (Line(points={{-82,0},{-120,0}}, color={0,0,127}));
  annotation (
    defaultComponentName="conFlo",
    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This block implements an ideal control of the evaporator (or condenser) water
mass flow rate.
The control intent aims to maintain a constant water temperature difference
<code>dT_nominal</code> across the heat exchanger, within the limit of a
minimum mass flow rate ratio <code>ratFloMin</code>.
For computational performance and to avoid the use of a PI controller,
the required mass flow rate is computed based on a signal representative of
the load.
</p>
</html>", revisions="<html>
<ul>
<li>
February 23, 2021, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"));
end PrimaryVariableFlow;
