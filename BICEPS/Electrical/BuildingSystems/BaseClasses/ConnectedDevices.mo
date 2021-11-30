within BICEPS.Electrical.BuildingSystems.BaseClasses;
model ConnectedDevices
  "Model of distributed electrically connected devices 
  including producers, consumers, and storages."
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Integer nPro=1 "Number of producer connections";
  parameter Integer nCon=1 "Number of consumer connections";
  parameter Integer nSto=1 "Number of storage connections";
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terPro[nPro]
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,110}),   iconTransformation(extent={{30,100},{50,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terSto[nSto]
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,110}),  iconTransformation(extent={{-10,100},{10,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terCon[nCon]
    "Connector for consumers" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-40,110}), iconTransformation(extent={{-50,100},{-30,120}})));
  Modelica.Blocks.Interfaces.RealOutput yCon[nCon] "Consumer control signal(s)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput yIn "Input control signal"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput PCon[nCon] "Power of consumers"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Equipment.ConsumerThreePhaseBalanced con[nCon] "Consumers"
    annotation (Placement(transformation(extent={{-50,-50},{-30,-70}})));
  Modelica.Blocks.Interfaces.RealOutput ySto[nSto] "Storage control signal(s)"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.RealOutput yPro[nPro] "Producer control signal(s)"
    annotation (Placement(transformation(extent={{98,70},{118,90}})));
  Equipment.ProducerPhotovoltaicArray producerPhotovoltaicArray
    annotation (Placement(transformation(extent={{50,40},{70,60}})));
  Equipment.StorageBattery bat[nSto]
    annotation (Placement(transformation(extent={{-10,20},{10,0}})));
equation
  connect(PCon, con.P) annotation (Line(points={{-120,-60},{-52,-60},{-52,-60}},
        color={0,0,127}));
  connect(terCon, con.terminal)
    annotation (Line(points={{-40,110},{-40,-49.2}}, color={0,120,120}));
  connect(con.y, yCon) annotation (Line(points={{-29,-66},{80,-66},{80,0},{110,0}},
        color={0,0,127}));
  for i in 1:nSto loop
    connect(yIn, bat[i].yIn) annotation (Line(points={{-120,60},{-80,60},{-80,4},{
          -12,4}}, color={0,0,127}));
  end for;
  connect(bat.terminal, terSto)
    annotation (Line(points={{0,20.8},{0,110},{0,110}}, color={0,120,120}));
  connect(bat.yOut, ySto) annotation (Line(points={{11,4},{80,4},{80,40},{110,40}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Polygon(points={{-56,40},{-24,40},{-12,16},{-24,-10},{-56,-10},{-68,16},
              {-56,40}},  lineColor={0,140,72}),
        Ellipse(extent={{26,-72},{-26,-22}},
                                          lineColor={0,140,72}),
        Polygon(
          points={{-20,-30},{0,0},{20,-30},{-20,-30}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-20,-30},{0,0}}, color={0,140,72}),
        Line(points={{0,0},{20,-30}},  color={0,140,72}),
        Ellipse(
          extent={{78,6},{44,40}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{62,6},{34,40}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{62,40},{14,40},{14,6},{62,6}},       color={0,140,72}),
        Line(points={{-40,40},{-40,100}}, color={0,0,0}),
        Line(points={{0,0},{0,100}},   color={0,0,0}),
        Line(points={{40,40},{40,100}}, color={0,0,0}),
        Polygon(
          points={{-40,-10},{-46,-20},{-34,-20},{-40,-10}},
          lineColor={0,0,0},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Line(points={{-100,-60},{-40,-60},{-40,-20}}, color={0,0,127})}),
                                                          Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ConnectedDevices;
