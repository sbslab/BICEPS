within BICEPS.Electrical.Device.Controls;
model Battery2 "Control for the battery energy storage system"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Energy EMax(min=0, displayUnit="kWh")
    "Maximum available charge";
  parameter Modelica.SIunits.Power P_nominal(min=0)
    "Nominal power charge/discharge rate";
  parameter Modelica.SIunits.Power PMax(min=0)=10000
    "Maximum power charge/discharge rate";
  parameter Modelica.SIunits.Power PMin(min=0)=100
    "Minimum power charge/discharge rate";
  parameter Modelica.SIunits.Time riseTime=60
    "Rise time of the filter (time to reach 99.6 % of the transition speed)"
    annotation(Dialog(tab="Dynamics", group="Filtered transition speed"));
  Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput soc "State of charge"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealOutput P(
    start = 0,
    final quantity="Power",
    final unit="W",
    displayUnit="KW") "Battery power (negative for discharge)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Continuous.Filter fil(
    analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
    filterType=Modelica.Blocks.Types.FilterType.LowPass,
    order=2,
    f_cut=5/(2*Modelica.Constants.pi*riseTime),
    init=Modelica.Blocks.Types.Init.InitialOutput)
    "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  BatteryStage sta(P_nominal=P_nominal) "Stage"
    annotation (Placement(transformation(extent={{-60,-16},{-40,4}})));
  Modelica.Blocks.Math.IntegerToReal intToRea "Integer to real"
    annotation (Placement(transformation(extent={{-20,-16},{0,4}})));
  Modelica.Blocks.Math.Product pro "Product"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Blocks.Math.Abs abs "Absolute value"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter lim(final uMax=PMax, final uMin=
       PMin)
    annotation (Placement(transformation(extent={{-20,50},{0,70}})));
equation
  connect(fil.y, P) annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
  connect(sta.sta, intToRea.u)
    annotation (Line(points={{-39,-6},{-22,-6}}, color={255,127,0}));
  connect(PNetIn, sta.PNetIn) annotation (Line(points={{-120,60},{-80,60},{-80,0},
          {-62,0}}, color={0,0,127}));
  connect(soc, sta.soc) annotation (Line(points={{-120,-40},{-80,-40},{-80,-10},
          {-62,-10}}, color={0,0,127}));
  connect(intToRea.y, pro.u2)
    annotation (Line(points={{1,-6},{18,-6}}, color={0,0,127}));
  connect(pro.y, fil.u)
    annotation (Line(points={{41,0},{68,0}}, color={0,0,127}));
  connect(PNetIn, abs.u)
    annotation (Line(points={{-120,60},{-62,60}}, color={0,0,127}));
  connect(abs.y, lim.u)
    annotation (Line(points={{-39,60},{-22,60}}, color={0,0,127}));
  connect(lim.y, pro.u1)
    annotation (Line(points={{2,60},{10,60},{10,6},{18,6}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery2;
