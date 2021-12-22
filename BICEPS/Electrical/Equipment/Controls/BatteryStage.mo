within BICEPS.Electrical.Equipment.Controls;
model BatteryStage "Staging control for battery"
  extends Modelica.Blocks.Icons.Block;
  parameter Real tWai = 60 "Waiting time (s)";
  parameter Modelica.SIunits.Power P_nominal(min=0)
    "Nominal power charge/discharge rate";
  Modelica.Blocks.Interfaces.RealInput soc "State of charge"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.IntegerOutput sta "State"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.StateGraph.Transition chaToOff(condition=PNetIn < P_nominal or soc >=
        0.95)
    "Charge to off" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,40})));
  Modelica.StateGraph.Transition offToDis(
    condition=PNetIn <= P_nominal and soc > 0.05,
                                          enableTimer=true, waitTime=tWai)
    "Off to discharge" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,-20})));
  Modelica.StateGraph.Transition disToOff(condition=PNetIn > P_nominal or soc <=
        0.05)
    "Discharge to off" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-70,-20})));
  Modelica.StateGraph.Transition offToCha(
    condition=PNetIn >= P_nominal and soc < 0.95,
                                          enableTimer=true, waitTime=tWai)
    "Off to charge" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-70,40})));
  Modelica.StateGraph.StepWithSignal cha "Charge"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Modelica.StateGraph.StepWithSignal dis "Discharge"
    annotation (Placement(transformation(extent={{-20,-60},{-40,-40}})));
  Modelica.StateGraph.InitialStep off(nIn=2, nOut=2)
                                      "Off state"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Modelica.Blocks.MathInteger.MultiSwitch swi(
    expr={1,-1},
    y_default=0,
    use_pre_as_default=false,
    nu=2) annotation (Placement(transformation(extent={{40,-10},{80,10}})));
  inner Modelica.StateGraph.StateGraphRoot stateGraphRoot
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
equation
  connect(cha.outPort[1], chaToOff.inPort)
    annotation (Line(points={{-19.5,70},{10,70},{10,44}}, color={0,0,0}));
  connect(off.outPort[1], offToDis.inPort)
    annotation (Line(points={{-19.5,10.25},{10,10.25},{10,-16}},
                                                           color={0,0,0}));
  connect(offToDis.outPort, dis.inPort[1])
    annotation (Line(points={{10,-21.5},{10,-50},{-19,-50}}, color={0,0,0}));
  connect(dis.outPort[1], disToOff.inPort)
    annotation (Line(points={{-40.5,-50},{-70,-50},{-70,-24}}, color={0,0,0}));
  connect(offToCha.outPort, cha.inPort[1])
    annotation (Line(points={{-70,41.5},{-70,70},{-41,70}}, color={0,0,0}));
  connect(disToOff.outPort, off.inPort[1])
    annotation (Line(points={{-70,-18.5},{-70,10.5},{-41,10.5}},
                                                             color={0,0,0}));
  connect(cha.active, swi.u[1]) annotation (Line(points={{-30,59},{-30,52},{30,52},
          {30,1.5},{40,1.5}}, color={255,0,255}));
  connect(dis.active, swi.u[2]) annotation (Line(points={{-30,-61},{-30,-70},{30,
          -70},{30,-1.5},{40,-1.5}}, color={255,0,255}));
  connect(swi.y, sta)
    annotation (Line(points={{81,0},{110,0}}, color={255,127,0}));
  connect(chaToOff.outPort, off.inPort[2]) annotation (Line(points={{10,38.5},{
          10,26},{-50,26},{-50,9.5},{-41,9.5}}, color={0,0,0}));
  connect(offToCha.inPort, off.outPort[2]) annotation (Line(points={{-70,36},{
          -70,30},{-10,30},{-10,9.75},{-19.5,9.75}}, color={0,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BatteryStage;
