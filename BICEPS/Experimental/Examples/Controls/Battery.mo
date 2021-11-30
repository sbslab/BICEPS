within BICEPS.Experimental.Examples.Controls;
model Battery "Control for the battery energy storage system"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Energy EMax(min=0, displayUnit="kWh")
    "Maximum available charge";
  parameter Modelica.SIunits.Power P_nominal(min=0)
    "Nominal power charge/discharge rate";
  parameter Modelica.SIunits.Time riseTime=1
    "Rise time of the filter (time to reach 99.6 % of the transition speed)"
    annotation(Dialog(tab="Dynamics", group="Filtered transition speed"));
  Modelica.Blocks.Interfaces.RealInput yEle
    "Electrical subsystem relative exergy potential"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput soc "State of charge"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealOutput P(
    start = 0,
    final quantity="Power",
    final unit="W",
    displayUnit="KW") "Battery power (negative for discharge)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Math.Gain staPow(k=P_nominal) "Power charge/discharge state"
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
  Modelica.Blocks.Sources.Constant off(k=0)
    "Battery state of charge is at its limit and cannot charge/discharge further"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold belCap(t=0.95,
                                                                  h=0.05)
    "Below SOC capacity. Hysteresis set for 10s cycles"
    annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
  Buildings.Controls.OBC.CDL.Logical.Switch swi
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  Buildings.Controls.OBC.CDL.Logical.And cha "Charge"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Buildings.Controls.OBC.CDL.Logical.Not callDis "Call for discharge"
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  Buildings.Controls.OBC.CDL.Logical.And dis "Discharge"
    annotation (Placement(transformation(extent={{30,-20},{50,0}})));
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
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold notEmp(t=0.05, h=0.025)
    "Not empty."
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  Buildings.Controls.OBC.CDL.Continuous.Hysteresis callCha(uLow=-0.5, uHigh=0.5)
    "Call to charge with switching charge <> discharge controlled with a hysteresis to avoid chattering"
    annotation (Placement(transformation(extent={{-90,50},{-70,70}})));
  Modelica.Blocks.Math.BooleanToReal sta(realTrue=1, realFalse=-1) "State"
    annotation (Placement(transformation(extent={{-30,50},{-10,70}})));
  Buildings.Controls.OBC.CDL.Logical.TrueFalseHold truFalHol(trueHoldDuration=
        10*60) "Hold to avoid short cycling"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
equation
  connect(soc, belCap.u)
    annotation (Line(points={{-120,-60},{-82,-60}}, color={0,0,127}));
  connect(cha.u2, belCap.y) annotation (Line(points={{-2,22},{-6,22},{-6,-60},{-58,
          -60}},     color={255,0,255}));
  connect(staPow.y, swi.u1) annotation (Line(points={{21,60},{60,60},{60,38},{
          68,38}}, color={0,0,127}));
  connect(callDis.y, dis.u1)
    annotation (Line(points={{22,-10},{28,-10}}, color={255,0,255}));
  connect(cha.y, chaOrDis.u1)
    annotation (Line(points={{22,30},{28,30}}, color={255,0,255}));
  connect(dis.y, chaOrDis.u2) annotation (Line(points={{52,-10},{54,-10},{54,10},
          {24,10},{24,22},{28,22}}, color={255,0,255}));
  connect(chaOrDis.y, swi.u2)
    annotation (Line(points={{52,30},{68,30}}, color={255,0,255}));
  connect(off.y, swi.u3) annotation (Line(points={{51,-50},{60,-50},{60,22},{68,
          22}}, color={0,0,127}));
  connect(swi.y, fil.u) annotation (Line(points={{92,30},{94,30},{94,16},{64,16},
          {64,0},{68,0}}, color={0,0,127}));
  connect(fil.y, P) annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
  connect(soc, notEmp.u) annotation (Line(points={{-120,-60},{-90,-60},{-90,-30},
          {-82,-30}}, color={0,0,127}));
  connect(notEmp.y, dis.u2) annotation (Line(points={{-58,-30},{24,-30},{24,-18},
          {28,-18}}, color={255,0,255}));
  connect(yEle, callCha.u)
    annotation (Line(points={{-120,60},{-92,60}}, color={0,0,127}));
  connect(sta.y, staPow.u)
    annotation (Line(points={{-9,60},{-2,60}}, color={0,0,127}));
  connect(callCha.y, truFalHol.u)
    annotation (Line(points={{-68,60},{-62,60}}, color={255,0,255}));
  connect(truFalHol.y, sta.u)
    annotation (Line(points={{-38,60},{-32,60}}, color={255,0,255}));
  connect(truFalHol.y, cha.u1) annotation (Line(points={{-38,60},{-36,60},{-36,
          30},{-2,30}}, color={255,0,255}));
  connect(callDis.u, truFalHol.y) annotation (Line(points={{-2,-10},{-36,-10},{
          -36,60},{-38,60}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery;
