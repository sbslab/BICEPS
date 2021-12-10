within BICEPS.Electrical.BuildingSystems;
model Electrical "Model of a building's electrical system"
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Integer nCon=1 "Number of consumer connections";
  parameter Integer nSto=1 "Number of storage connections";
  final parameter Integer nPro=if have_pv and have_wind then 2 elseif
     have_pv or have_wind then 1 else 0 "Number of producer connections";
  parameter Boolean have_pv=true "True if the building has a PV system";
  parameter Boolean have_wind=true "True if the building has a wind system";
  parameter Real tol=0.05
    "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100 "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.Angle lat "Latitude";
  parameter Modelica.SIunits.Power PCon_nominal[nCon]
    "Nominal power for consumer loads";
  parameter Modelica.SIunits.Power PPro_nominal=
    if (have_pv and have_wind) then PSun+PWin elseif
      have_pv then PSun elseif
      have_wind then PWin else
      0
    "Nominal power for producer loads";
  parameter Modelica.SIunits.Power PSun "Nominal power of the PV";
  parameter Modelica.SIunits.Power PWin "Nominal power of the wind turbine";
  parameter Modelica.SIunits.Power PSto_nominal
    "Nominal power for storage loads";
  parameter Modelica.SIunits.Energy EBatMax=180000000
    "Maximum energy capacity of the battery";
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Modelica.SIunits.Length LGri=1500 "Length of the grid line";
  parameter Modelica.SIunits.Length LCon[nCon]=fill(10,nCon)
    "Length of the consumer lines";
  parameter Modelica.SIunits.Length LPro[nPro]=fill(25,nPro)
    "Length of the producer lines";
  parameter Modelica.SIunits.Length LSto[nSto]=fill(5,nSto)
    "Length of the storage lines";
  Equipment.Panel P1(
    biomimeticControl=biomimeticControl,
    nPro=nPro,
    nCon=nCon,
    nSto=nSto)
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  BaseClasses.ConnectedDevices dev(
    biomimeticControl=biomimeticControl,
    nCon=nCon,
    nSto=nSto,
    have_pv=have_pv,
    have_wind=have_wind,
    lat=lat,
    V_nominal=V_nominal,
    tol=tol,
    k=k,
    PSun=PSun,
    PWin=PWin,
    PBat=PSto_nominal,
    EBatMax=EBatMax)
    annotation (Placement(transformation(extent={{10,-40},{-10,-20}})));
  Buildings.Electrical.AC.OnePhase.Lines.Line linGri(
    l=LGri,
    P_nominal=sum(PCon_nominal)+sum(PPro_nominal)+sum(PSto_nominal),
    each final V_nominal=V_nominal) "Grid power line"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,60})));
  Buildings.Electrical.AC.OnePhase.Lines.Line linCon[nCon](
    l=LCon,
    P_nominal=PCon_nominal,
    each final V_nominal=V_nominal) "Consumer power lines" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,0})));
  Buildings.Electrical.AC.OnePhase.Lines.Line linSto[nSto](
    l=LSto,
    final P_nominal=fill(PSto_nominal, nSto),
    each final V_nominal=V_nominal) "Storage  power lines" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={0,0})));
  Buildings.Electrical.AC.OnePhase.Lines.Line linPro[nPro](
    l=LPro,
    P_nominal=fill(PPro_nominal, nPro),
    each final V_nominal=V_nominal) "Producer power lines" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-10,0})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,80}),    iconTransformation(extent={{-120,60},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput PCon[nCon] "Power of consumers"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus
    "Weather data bus"
    annotation (Placement(transformation(extent={{-20,82},{20,122}}),
      iconTransformation(extent={{-10,90},{10,110}})));

  Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
    "Output control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
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
    annotation (Line(points={{-110,80},{0,80},{0,70}},
                                              color={0,120,120}));
  connect(PCon, dev.PCon) annotation (Line(points={{-120,-60},{30,-60},{30,-36},
          {12,-36}}, color={0,0,127}));
  connect(dev.yPro, P1.yPro) annotation (Line(points={{-10.8,-22},{-22,-22},{-22,
          30},{-12,30}}, color={0,0,127}));
  connect(dev.ySto, P1.ySto) annotation (Line(points={{-11,-26},{-26,-26},{-26,34},
          {-12,34}}, color={0,0,127}));
  connect(dev.yCon, P1.yCon) annotation (Line(points={{-11,-30},{-30,-30},{-30,38},
          {-12,38}}, color={0,0,127}));
  connect(weaBus, dev.weaBus) annotation (Line(
      points={{0,102},{0,88},{40,88},{40,-20},{9,-20}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(P1.yOut, yOut) annotation (Line(points={{11,36},{30,36},{30,80},{110,
          80}},
        color={0,0,127}));
  connect(P1.yNetPow, dev.yNetPow) annotation (Line(points={{11,32},{30,32},{30,
          -24},{12,-24}}, color={0,0,127}));
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
