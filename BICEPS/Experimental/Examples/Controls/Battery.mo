within BICEPS.Experimental.Examples.Controls;
model Battery "Control for the battery energy storage system"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Energy EMax(min=0, displayUnit="kWh")
    "Maximum available charge";
  parameter Modelica.SIunits.Power P_nominal(min=0)
    "Nominal power charge/discharge rate";
  Modelica.Blocks.Interfaces.RealInput yEle
    "Electrical subsystem relative exergy potential"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput soc "State of charge"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealOutput P(
    final quantity="Power",
    final unit="W",
    displayUnit="KW") "Battery power (negative for discharge)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Math.Sign sta
    "State of charge or discharge (positive = charge)"
    annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
  Modelica.Blocks.Math.Gain staPow(k=P_nominal) "Power charge/discharge state"
    annotation (Placement(transformation(extent={{0,50},{20,70}})));
  Modelica.Blocks.Sources.Constant off(k=0)
    "Battery state of charge is at its limit and cannot charge/discharge further"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
  Buildings.Controls.OBC.CDL.Continuous.LessThreshold belCap(t=EMax, h=
        P_nominal*10) "Below SOC capacity. Hysteresis set for 10s cycles"
    annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  Buildings.Controls.OBC.CDL.Logical.Switch swi
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold callCha(h=P_nominal*10)
    "Call for charge"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Buildings.Controls.OBC.CDL.Logical.And cha "Charge"
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold emp(h=P_nominal*10)
    "Empty battery"
    annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
  Buildings.Controls.OBC.CDL.Logical.Not callDis "Call for discharge"
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  Buildings.Controls.OBC.CDL.Logical.And dis "Discharge"
    annotation (Placement(transformation(extent={{30,-20},{50,0}})));
  Buildings.Controls.OBC.CDL.Logical.Or chaOrDis "Charge or discharge"
    annotation (Placement(transformation(extent={{30,20},{50,40}})));
equation
  connect(yEle, sta.u)
    annotation (Line(points={{-120,60},{-82,60}}, color={0,0,127}));
  connect(sta.y, staPow.u)
    annotation (Line(points={{-59,60},{-2,60}}, color={0,0,127}));
  connect(soc, belCap.u)
    annotation (Line(points={{-120,-60},{-62,-60}}, color={0,0,127}));
  connect(sta.y, callCha.u) annotation (Line(points={{-59,60},{-50,60},{-50,30},
          {-42,30}}, color={0,0,127}));
  connect(callCha.y, cha.u1)
    annotation (Line(points={{-18,30},{-2,30}}, color={255,0,255}));
  connect(cha.u2, belCap.y) annotation (Line(points={{-2,22},{-6,22},{-6,-60},{
          -38,-60}}, color={255,0,255}));
  connect(staPow.y, swi.u1) annotation (Line(points={{21,60},{60,60},{60,38},{
          68,38}}, color={0,0,127}));
  connect(soc, emp.u) annotation (Line(points={{-120,-60},{-80,-60},{-80,-30},{
          -62,-30}}, color={0,0,127}));
  connect(callCha.y, callDis.u) annotation (Line(points={{-18,30},{-12,30},{-12,
          -10},{-2,-10}}, color={255,0,255}));
  connect(callDis.y, dis.u1)
    annotation (Line(points={{22,-10},{28,-10}}, color={255,0,255}));
  connect(emp.y, dis.u2) annotation (Line(points={{-38,-30},{24,-30},{24,-18},{
          28,-18}}, color={255,0,255}));
  connect(cha.y, chaOrDis.u1)
    annotation (Line(points={{22,30},{28,30}}, color={255,0,255}));
  connect(dis.y, chaOrDis.u2) annotation (Line(points={{52,-10},{54,-10},{54,10},
          {24,10},{24,22},{28,22}}, color={255,0,255}));
  connect(chaOrDis.y, swi.u2)
    annotation (Line(points={{52,30},{68,30}}, color={255,0,255}));
  connect(off.y, swi.u3) annotation (Line(points={{51,-50},{60,-50},{60,22},{68,
          22}}, color={0,0,127}));
  connect(swi.y, P) annotation (Line(points={{92,30},{94,30},{94,0},{110,0}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Battery;
