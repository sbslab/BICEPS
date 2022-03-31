within BICEPS.Electrical.Device.Controls;
model Battery "Control for the battery energy storage system"
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
  Modelica.Blocks.Sources.Constant off(k=0)
    "Battery state of charge is at its limit and cannot charge/discharge further"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold belCap(t=0.95, h=0.04)
    "Below SOC capacity. Hysteresis set for 10s cycles"
    annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  Buildings.Controls.OBC.CDL.Logical.Switch swi
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  Buildings.Controls.OBC.CDL.Logical.And cha "Charge"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Buildings.Controls.OBC.CDL.Logical.And dis "Discharge"
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  Buildings.Controls.OBC.CDL.Logical.Or chaOrDis "Charge or discharge"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
  Modelica.Blocks.Continuous.Filter fil(
    analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
    filterType=Modelica.Blocks.Types.FilterType.LowPass,
    order=2,
    f_cut=5/(2*Modelica.Constants.pi*riseTime),
    init=Modelica.Blocks.Types.Init.InitialOutput)
    "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold notEmp(t=0.05, h=0.04)
    "Not empty."
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  Modelica.Blocks.Math.RealToBoolean grePowNomPos(threshold=P_nominal*1.05)
    "Greater than nominal power (positive)"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Modelica.Blocks.Math.RealToBoolean grePowNomNeg(threshold=-P_nominal*1.05)
    "Greater than nominal power (negative)"
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  Buildings.Controls.OBC.CDL.Logical.Not lesPowNomNeg
    "Less than nominal power (negative)"
    annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
equation
  connect(soc, belCap.u)
    annotation (Line(points={{-120,-40},{-82,-40}}, color={0,0,127}));
  connect(cha.u2, belCap.y) annotation (Line(points={{-2,22},{-50,22},{-50,-40},
          {-58,-40}},color={255,0,255}));
  connect(cha.y, chaOrDis.u1)
    annotation (Line(points={{22,30},{28,30}}, color={255,0,255}));
  connect(dis.y, chaOrDis.u2) annotation (Line(points={{22,-10},{24,-10},{24,22},
          {28,22}},                 color={255,0,255}));
  connect(chaOrDis.y, swi.u2)
    annotation (Line(points={{52,30},{68,30}}, color={255,0,255}));
  connect(off.y, swi.u3) annotation (Line(points={{51,-50},{60,-50},{60,22},{68,
          22}}, color={0,0,127}));
  connect(swi.y, fil.u) annotation (Line(points={{92,30},{94,30},{94,16},{64,16},
          {64,0},{68,0}}, color={0,0,127}));
  connect(fil.y, P) annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
  connect(soc, notEmp.u) annotation (Line(points={{-120,-40},{-90,-40},{-90,-70},
          {-82,-70}}, color={0,0,127}));
  connect(notEmp.y, dis.u2) annotation (Line(points={{-58,-70},{-10,-70},{-10,-18},
          {-2,-18}}, color={255,0,255}));
  connect(PNetIn, grePowNomPos.u) annotation (Line(points={{-120,60},{-90,60},{-90,
          30},{-82,30}}, color={0,0,127}));
  connect(grePowNomNeg.y, lesPowNomNeg.u)
    annotation (Line(points={{-59,-10},{-42,-10}}, color={255,0,255}));
  connect(grePowNomPos.y, cha.u1)
    annotation (Line(points={{-59,30},{-2,30}}, color={255,0,255}));
  connect(PNetIn, grePowNomNeg.u) annotation (Line(points={{-120,60},{-90,60},{-90,
          -10},{-82,-10}}, color={0,0,127}));
  connect(dis.u1, lesPowNomNeg.y)
    annotation (Line(points={{-2,-10},{-18,-10}}, color={255,0,255}));
  connect(PNetIn, swi.u1) annotation (Line(points={{-120,60},{60,60},{60,38},{
          68,38}},                  color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery;
