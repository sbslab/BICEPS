within BICEPS.Combined.ElectricalAndFluid.Examples;
model SingleFamilyResidentialBuilding
  "Example model for the electrical and mechanical (thermofluid) coupled system"
  extends Modelica.Icons.Example;
  package Medium=Buildings.Media.Water
    "Medium in the building distribution system";
  parameter String filNam=
    "modelica://BICEPS/Resources/Data/Combined/ElectricalAndFluid/SingleFamilyBuildingElectricity.mos"
    "Library path of the files with other electrical loads as time series";
  BICEPS.Combined.ElectricalAndFluid.SingleFamilyResidentialBuilding bld(
    redeclare package MediumWat = Medium,
    show_T=true,
    lat=weaDat.lat,
    POth_nominal=9653,
    EBatMax=540000000,
    filNam=filNam,
    TMin=bld.T0 - 1.5,
    TMax=bld.T0 + 2.5)
    "Building"
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
  Buildings.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    T=285.15,
    nPorts=1) "Sink"
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  Buildings.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    T=293.15,
    nPorts=1) "Source"
    annotation (Placement(transformation(extent={{60,-90},{40,-70}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    computeWetBulbTemperature=false, filNam=
        ModelicaServices.ExternalReferences.loadResource(
        "modelica://BICEPS/Resources/weatherdata/DEU_BW_Mannheim_107290_TRY2010_12_Jahr_BBSR.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(
    f=60,
    V=208,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
  Modelica.Blocks.Continuous.Integrator EGri "Grid energy"
    annotation (Placement(transformation(extent={{60,70},{80,90}})));
  Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
    "Real grid power"
    annotation (Placement(transformation(extent={{20,70},{40,90}})));
  Modelica.Blocks.Continuous.Integrator Epv if bld.have_pv "PV energy"
    annotation (Placement(transformation(extent={{60,40},{80,60}})));
  Modelica.Blocks.Continuous.Integrator EHeaPum "Heat pump energy"
    annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
  Modelica.Blocks.Continuous.Integrator EBat "Battery energy"
    annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Modelica.Blocks.Sources.RealExpression Ppv(y=bld.ele.dev.pv.pv.P) if bld.have_pv
    "PV power"
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Modelica.Blocks.Sources.RealExpression PHeaPum(y=bld.ele.dev.con[1].loa.P)
    "Heat pump power"
    annotation (Placement(transformation(extent={{20,-50},{40,-30}})));
  Modelica.Blocks.Sources.RealExpression PBat(y=-1*bld.ele.dev.bat[1].bat.P)
    "Battery power"
    annotation (Placement(transformation(extent={{20,-20},{40,0}})));
  Modelica.Blocks.Continuous.Integrator EPum "Pump energy"
    annotation (Placement(transformation(extent={{-20,70},{0,90}})));
  Modelica.Blocks.Sources.RealExpression PPum(y=bld.ele.dev.con[2].loa.P)
    "Pump power"
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Modelica.Blocks.Continuous.Integrator EOth "Other loads"
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Modelica.Blocks.Sources.RealExpression POth(y=bld.ele.dev.con[3].loa.P)
    "Other loads power"
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Modelica.Blocks.Continuous.Integrator EWin if bld.have_wind "Wind energy"
    annotation (Placement(transformation(extent={{60,10},{80,30}})));
  Modelica.Blocks.Sources.RealExpression PWin(y=bld.ele.dev.win.winTur.P) if
       bld.have_wind
    "Wind power"
    annotation (Placement(transformation(extent={{20,10},{40,30}})));
equation
  connect(gri.terminal, bld.terminal) annotation (Line(points={{-70,-30},{-70,
          -53},{-21,-53}},
                      color={0,120,120}));
  connect(sin.ports[1], bld.port_b) annotation (Line(points={{-60,-80},{-40,-80},
          {-40,-66},{-20,-66}}, color={0,127,255}));
  connect(bld.port_a, sou.ports[1]) annotation (Line(points={{0,-66},{20,-66},{
          20,-80},{40,-80}},
                          color={0,127,255}));
  connect(weaDat.weaBus, bld.weaBus) annotation (Line(
      points={{-20,0},{-10,0},{-10,-50}},
      color={255,204,51},
      thickness=0.5));
  connect(PGriRea.y,EGri. u)
    annotation (Line(points={{41,80},{58,80}}, color={0,0,127}));
  connect(Ppv.y,Epv. u)
    annotation (Line(points={{41,50},{58,50}}, color={0,0,127}));
  connect(PHeaPum.y, EHeaPum.u)
    annotation (Line(points={{41,-40},{58,-40}}, color={0,0,127}));
  connect(PBat.y,EBat. u)
    annotation (Line(points={{41,-10},{58,-10}}, color={0,0,127}));
  connect(PPum.y, EPum.u)
    annotation (Line(points={{-39,80},{-22,80}}, color={0,0,127}));
  connect(POth.y, EOth.u)
    annotation (Line(points={{-39,50},{-22,50}}, color={0,0,127}));
  connect(PWin.y, EWin.u)
    annotation (Line(points={{41,20},{58,20}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Combined/ElectricalAndFluid/Examples/SingleFamilyResidentialBuilding.mos"
      "Simulate and plot"),
    experiment(
      StartTime=86400,
      StopTime=172800,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end SingleFamilyResidentialBuilding;
