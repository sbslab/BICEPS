within BICEPS.Electrical.Device;
model Panel "Generic model for an electrical panel"
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Integer nSto=1 "Number of storage connections";
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terGri
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,110}), iconTransformation(extent={{-10,100},{10,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terPro
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-60,-110}), iconTransformation(extent={{-50,-120},{-30,-100}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terSto[nSto]
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={0,-110}), iconTransformation(extent={{-10,-120},{10,-100}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terCon
    "Connector for consumers" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-110}), iconTransformation(extent={{30,-120},{50,-100}})));
  Modelica.Blocks.Interfaces.RealInput yCon if biomimeticControl
    "Consumer control signal(s)"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
    "Output control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Buildings.Electrical.AC.OnePhase.Sensors.GeneralizedSensor met "Main meter"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={0,40})));
  Controls.Panel con(n=nSto + 2) if
                                biomimeticControl "Controller"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Modelica.Blocks.Interfaces.RealInput ySto[nSto] if biomimeticControl
    "Storage control signal(s)"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealInput yPro if biomimeticControl
    "Producer control signal(s)"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput PNetOut "Net power output"
    annotation (Placement(transformation(extent={{100,10},{120,30}})));
  BICEPS.Electrical.Sensors.TotalPower senPro "Sensor on producers" annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,-50})));
  BICEPS.Electrical.Sensors.TotalPower senCon "Sensor on consumers" annotation (
     Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={60,-50})));
  Modelica.Blocks.Math.Add PNet(k1=+1, k2=-1)
    "Net power supply/demand only"
    annotation (Placement(transformation(extent={{40,10},{60,30}})));
equation
  connect(terGri, met.terminal_n)
    annotation (Line(points={{0,110},{0,50},{1.77636e-15,50}},
                                              color={0,120,120}));
  connect(senPro.terminal_p, met.terminal_p)
    annotation (Line(points={{-60,-40},{-60,-30},{0,-30},{0,30}},
      color={0,120,120}));
  connect(yPro, con.yIn[1])
    annotation (Line(
     points={{-120,0},{-70,0},{-70,60},{-59.8,60}},
     color={0,0,127}));
  for i in 1:nSto loop
    connect(met.terminal_p, terSto[i])
      annotation (Line(points={{-1.77636e-15,30},{-1.77636e-15,-110},{0,-110}},
                                                          color={0,120,120}));
    connect(ySto[i], con.yIn[i+2])
      annotation (Line(points={{-120,40},{-70,40},{-70,60},{-59.8,60}},
         color={0,0,127}));
  end for;
  connect(senCon.terminal_n, met.terminal_p)
    annotation (Line(points={{60,-40},{60,-30},{0,-30},{0,30},{-1.77636e-15,
          30}},color={0,120,120}));
  connect(yCon, con.yIn[2])
    annotation (Line(points={{-120,80},{-70,80},{-70,60},{-59.8,60}}, color={0,0,127}));
  connect(con.yOut, yOut)
    annotation (Line(points={{-39,60},{110,60}}, color={0,0,127}));
  connect(terPro, senPro.terminal_n)
    annotation (Line(points={{-60,-110},{-60,-60}}, color={0,120,120}));
  connect(terCon, senCon.terminal_p)
    annotation (Line(points={{60,-110},{60,-60}}, color={0,120,120}));
  connect(PNet.y, PNetOut)
    annotation (Line(points={{61,20},{110,20}}, color={0,0,127}));
  connect(senPro.P, PNet.u1) annotation (Line(points={{-51,-50},{-20,-50},{-20,26},
          {38,26}}, color={0,0,127}));
  connect(senCon.P, PNet.u2) annotation (Line(points={{51,-50},{20,-50},{20,14},
          {38,14}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-24},{0,-32}}, color={0,0,0}),
          Rectangle(
          extent={{-60,80},{60,-80}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Rectangle(
          extent={{-26,24},{24,-24}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-22,20},{20,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="M"),
        Ellipse(extent={{-22,-20},{20,20}}, lineColor={0,0,0}),
        Line(points={{0,-36},{16,-64}}, color={0,0,0}),
        Ellipse(
          extent={{-4,-32},{4,-40}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,-60},{4,-68}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{0,24},{0,100}}, color={0,0,0}),
        Rectangle(
          extent={{-52,70},{50,30}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),     Text(
          extent={{-56,80},{56,28}},
          lineColor={0,0,255},
          textString="%name"),
        Line(points={{0,-68},{0,-100}}, color={0,0,0}),
        Line(points={{40,-100},{0,-80},{-40,-100}}, color={0,0,0})}),
                                 Diagram(coordinateSystem(preserveAspectRatio=false)));
end Panel;
