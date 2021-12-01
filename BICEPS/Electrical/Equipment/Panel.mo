within BICEPS.Electrical.Equipment;
model Panel "Generic model for an electrical panel"
  parameter Integer nPro=1 "Number of producer connections";
  parameter Integer nCon=1 "Number of consumer connections";
  parameter Integer nSto=1 "Number of storage connections";
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terGri
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,110}), iconTransformation(extent={{-10,100},{10,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terPro[nPro]
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-60,-110}), iconTransformation(extent={{-50,-120},{-30,-100}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terSto[nSto]
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={0,-110}), iconTransformation(extent={{-10,-120},{10,-100}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terCon[nCon]
    "Connector for consumers" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-110}), iconTransformation(extent={{30,-120},{50,-100}})));
  Modelica.Blocks.Interfaces.RealInput yCon[nCon] "Consumer control signal(s)"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Buildings.Electrical.AC.OnePhase.Sensors.GeneralizedSensor met "Main meter"
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=-90,
        origin={0,40})));
  Controls.Panel con(n=nPro + nSto + nCon) "Controller"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Modelica.Blocks.Interfaces.RealInput ySto[nSto] "Storage control signal(s)"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealInput yPro[nPro] "Producer control signal(s)"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput yNetPow
    "Storage signal based on net supply/demand power"
    annotation (Placement(transformation(extent={{100,10},{120,30}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sensors.GeneralizedSensor senPro[
    nPro] "Sensor on producers" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,-50})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sensors.GeneralizedSensor senCon[
    nCon] "Sensor on consumers" annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={60,-50})));
  Modelica.Blocks.Math.Add PNet(k1=+1, k2=-1)
                                "Net power supply/demand only"
    annotation (Placement(transformation(extent={{40,10},{60,30}})));
  Modelica.Blocks.Math.Sum PPro "Producer power"
    annotation (Placement(transformation(extent={{-40,-22},{-20,0}})));
  Modelica.Blocks.Math.Sum PCon "Consumer power"
    annotation (Placement(transformation(extent={{40,-22},{20,0}})));
  Buildings.Controls.OBC.CDL.Continuous.Limiter lim(uMax=1, uMin=-1)
    annotation (Placement(transformation(extent={{70,10},{90,30}})));
equation
  connect(terGri, met.terminal_n)
    annotation (Line(points={{0,110},{0,50},{1.77636e-15,50}},
                                              color={0,120,120}));
  for i in 1:nPro loop
    connect(senPro[i].terminal_p, met.terminal_p)
      annotation (Line(points={{-60,-40},{-60,-30},{0,-30},{0,30}},
        color={0,120,120}));
    connect(yPro[i], con.yIn[i])
      annotation (Line(
       points={{-120,0},{-70,0},{-70,60},{-59.8,60}},
       color={0,0,127}));
    connect(senPro[i].S[1], PPro.u[i])
      annotation (Line(points={{-51,-56},{-48,-56},
          {-48,-11},{-42,-11}}, color={0,0,127}));
  end for;
  for i in 1:nSto loop
    connect(met.terminal_p, terSto[i])
      annotation (Line(points={{-1.77636e-15,30},{-1.77636e-15,-110},{0,-110}},
                                                          color={0,120,120}));
     connect(ySto[i], con.yIn[i+nPro])
       annotation (Line(points={{-120,40},{-70,40},{-70,60},{-59.8,60}},
         color={0,0,127}));
  end for;
  for i in 1:nCon loop
    connect(senCon[i].terminal_n, met.terminal_p)
      annotation (Line(points={{60,-40},{60,-30},{0,-30},{0,30},{-1.77636e-15,
            30}},                                                               color={0,120,120}));
    connect(yCon[i], con.yIn[i+nPro+nSto])
      annotation (Line(points={{-120,80},{-70,80},{-70,60},{-59.8,60}}, color={0,0,127}));
    connect(senCon[i].S[1], PCon.u[i])
      annotation (Line(points={{51,-44},{48,-44},
          {48,-11},{42,-11}}, color={0,0,127}));
  end for;
  connect(con.yOut, yOut)
    annotation (Line(points={{-39,60},{110,60}}, color={0,0,127}));

  connect(terPro, senPro.terminal_n)
    annotation (Line(points={{-60,-110},{-60,-60}}, color={0,120,120}));
  connect(terCon, senCon.terminal_p)
    annotation (Line(points={{60,-110},{60,-60}}, color={0,120,120}));




  connect(PCon.y, PNet.u2) annotation (Line(points={{19,-11},{14,-11},{14,14},{
          38,14}}, color={0,0,127}));
  connect(PPro.y, PNet.u1) annotation (Line(points={{-19,-11},{12,-11},{12,26},
          {38,26}}, color={0,0,127}));
  connect(PNet.y, lim.u)
    annotation (Line(points={{61,20},{68,20}}, color={0,0,127}));
  connect(lim.y, yNetPow)
    annotation (Line(points={{92,20},{110,20}}, color={0,0,127}));
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
