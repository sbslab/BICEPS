within BICEPS.Electrical.BuildingSystems;
model Electrical "Model of a building's electrical system"
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Integer nPro=1 "Number of producer connections";
  parameter Integer nCon=1 "Number of consumer connections";
  parameter Integer nSto=1 "Number of storage connections";
  parameter Modelica.SIunits.Power PCon_nominal[nCon] = fill(0,nCon)
    "Nominal power for consumer loads";
  parameter Modelica.SIunits.Power PPro_nominal[nPro] = fill(0,nPro)
    "Nominal power for producer loads";
  parameter Modelica.SIunits.Power PSto_nominal[nSto] = fill(0,nSto)
    "Nominal power for storage loads";
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Modelica.SIunits.Length LGri=1500 "Length of the grid line";
  parameter Modelica.SIunits.Length LCon[nCon]=fill(50,nCon)
    "Length of the consumer lines";
  parameter Modelica.SIunits.Length LPro[nPro]=fill(100,nPro)
    "Length of the producer lines";
  parameter Modelica.SIunits.Length LSto[nSto]=fill(20,nSto)
    "Length of the storage lines";
  Equipment.Panel P1(
    nPro=nPro,
    nCon=nCon,
    nSto=nSto)
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  BaseClasses.ConnectedDevices dev(
    nPro=nPro,
    nCon=nCon,
    nSto=nSto)
    annotation (Placement(transformation(extent={{10,-40},{-10,-20}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linGri(
    l=LGri,
    P_nominal=sum(PCon_nominal)+sum(PPro_nominal)+sum(PSto_nominal),
    each final V_nominal=V_nominal) "Grid power line"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,60})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linCon[nCon](
    l=LCon,
    P_nominal=PCon_nominal,
    each final V_nominal=V_nominal) "Consumer power lines" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,0})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linSto[nSto](
    l=LSto,
    final P_nominal=PSto_nominal,
    each final V_nominal=V_nominal) "Storage  power lines" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={0,0})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linPro[nPro](
    l=LPro,
    P_nominal=PPro_nominal,
    each final V_nominal=V_nominal) "Producer power lines" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-10,0})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,110}),      iconTransformation(extent={{-120,60},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput PCon[nCon] "Power of consumers"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
equation
  connect(P1.terCon, linCon.terminal_n) annotation (Line(points={{4,19},{4,14},{
          10,14},{10,10}}, color={0,120,120}));
  connect(P1.terSto, linSto.terminal_p)
    annotation (Line(points={{0,19},{0,10}}, color={0,120,120}));
  connect(P1.terPro, linPro.terminal_p) annotation (Line(points={{-4,19},{-4,14},
          {-10,14},{-10,10}}, color={0,120,120}));
  connect(linCon.terminal_p, dev.terCon) annotation (Line(points={{10,-10},{10,-14},
          {4,-14},{4,-19}}, color={0,120,120}));
  connect(linSto.terminal_n, dev.terSto)
    annotation (Line(points={{0,-10},{0,-19}}, color={0,120,120}));
  connect(linPro.terminal_n, dev.terPro) annotation (Line(points={{-10,-10},{-10,
          -14},{-4,-14},{-4,-19}}, color={0,120,120}));
  connect(linGri.terminal_p, P1.terGri)
    annotation (Line(points={{0,50},{0,41}}, color={0,120,120}));
  connect(terminal, linGri.terminal_n)
    annotation (Line(points={{0,110},{0,70}}, color={0,120,120}));
  connect(PCon, dev.PCon) annotation (Line(points={{-120,-60},{30,-60},{30,-36},
          {12,-36}}, color={0,0,127}));
  connect(P1.yOut, dev.yIn) annotation (Line(points={{11,36},{30,36},{30,-24},{12,
          -24}}, color={0,0,127}));
  connect(dev.yPro, P1.yPro) annotation (Line(points={{-10.8,-22},{-22,-22},{
          -22,30},{-12,30}}, color={0,0,127}));
  connect(dev.ySto, P1.ySto) annotation (Line(points={{-11,-26},{-26,-26},{-26,
          34},{-12,34}}, color={0,0,127}));
  connect(dev.yCon, P1.yCon) annotation (Line(points={{-11,-30},{-30,-30},{-30,
          38},{-12,38}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
                             Polygon(
        points={{-50,-76},{-34,-26},{0,-58},{-50,-76}},
        lineColor={0,0,0},
        smooth=Smooth.None,
        fillPattern=FillPattern.Solid,
        fillColor={0,0,0}),      Line(
        points={{40,78},{-28,10},{32,10},{-50,-76},{-50,-76}},
        color={0,0,0},
        smooth=Smooth.None)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Electrical;
