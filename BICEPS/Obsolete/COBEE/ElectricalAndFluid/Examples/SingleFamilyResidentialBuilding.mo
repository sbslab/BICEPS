within BICEPS.Obsolete.COBEE.ElectricalAndFluid.Examples;
model SingleFamilyResidentialBuilding
  "Example model for the electrical and mechanical (thermofluid) coupled system"
  extends Modelica.Icons.Example;
  package Medium=Buildings.Media.Water
    "Medium in the building distribution system";
  parameter String filNam=
    "modelica://BICEPS/Resources/Data/Experimental/COBEE/ElectricalAndFluid/SingleFamilyBuildingElectricity.mos"
    "Library path of the files with other electrical loads as time series";
  BICEPS.Experimental.COBEE.ElectricalAndFluid.SingleFamilyResidentialBuilding bld(
    redeclare package MediumWat = Medium,
    biomimeticControl=true,
    show_T=true,
    lat=weaDat.lat,
    PHeaPum_nominal(displayUnit="kW") = 4000,
    POth_nominal=9653,
    PPV_nominal(displayUnit="kW") = 1000*(4*3.36),
    PBat_nominal=500,
    PBatMin=500,
    filNam=filNam,
    TMin=bld.T0 - 2,
    TMax=bld.T0 + 2) "Building"
    annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
  Buildings.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    T=293.15,
    nPorts=1) "Sink"
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  Buildings.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    T=285.15,
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
    annotation (Placement(transformation(extent={{60,100},{80,120}})));
  Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
    "Real grid power"
    annotation (Placement(transformation(extent={{20,100},{40,120}})));
  Modelica.Blocks.Continuous.Integrator Epv "PV energy"
    annotation (Placement(transformation(extent={{60,70},{80,90}})));
  Modelica.Blocks.Continuous.Integrator EHeaPum "Heat pump energy"
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Modelica.Blocks.Continuous.Integrator EBat "Battery energy"
    annotation (Placement(transformation(extent={{60,10},{80,30}})));
  Modelica.Blocks.Sources.RealExpression Ppv(y=bld.ele.dev.pv.pv.P)
    "PV power"
    annotation (Placement(transformation(extent={{20,70},{40,90}})));
  Modelica.Blocks.Sources.RealExpression PHeaPum(y=bld.mec.PHeaPum)
    "Heat pump power"
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Modelica.Blocks.Sources.RealExpression PBat(y=-1*bld.ele.dev.bat[1].bat.P)
    "Battery power"
    annotation (Placement(transformation(extent={{20,10},{40,30}})));
  Modelica.Blocks.Continuous.Integrator EPum "Pump energy"
    annotation (Placement(transformation(extent={{-20,100},{0,120}})));
  Modelica.Blocks.Sources.RealExpression PPum(y=bld.mec.PPum)
    "Pump power"
    annotation (Placement(transformation(extent={{-60,100},{-40,120}})));
  Modelica.Blocks.Continuous.Integrator EOth "Other loads"
    annotation (Placement(transformation(extent={{-20,70},{0,90}})));
  Modelica.Blocks.Sources.RealExpression POth(y=bld.gain.y)
    "Other loads power"
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Modelica.Blocks.Logical.GreaterThreshold dis "Discharging"
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Blocks.Logical.LessThreshold cha "Charging"
    annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Modelica.Blocks.Math.BooleanToReal cha1(realTrue=-1)
    annotation (Placement(transformation(extent={{92,-20},{112,0}})));
  Modelica.Blocks.Math.BooleanToReal dis1 "Discharge 1"
    annotation (Placement(transformation(extent={{92,-60},{112,-40}})));
  Modelica.Blocks.Continuous.Integrator EBatCha "Battery energy, charging"
    annotation (Placement(transformation(extent={{148,-14},{168,6}})));
  Modelica.Blocks.Continuous.Integrator EBatDis "Battery energy, discharging"
    annotation (Placement(transformation(extent={{150,-54},{170,-34}})));
  Modelica.Blocks.Math.Product PBatCha
    annotation (Placement(transformation(extent={{120,-14},{140,6}})));
  Modelica.Blocks.Math.Product PBatDis
    annotation (Placement(transformation(extent={{120,-54},{140,-34}})));
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
    annotation (Line(points={{41,110},{58,110}},
                                               color={0,0,127}));
  connect(Ppv.y,Epv. u)
    annotation (Line(points={{41,80},{58,80}}, color={0,0,127}));
  connect(PHeaPum.y, EHeaPum.u)
    annotation (Line(points={{-39,50},{-22,50}}, color={0,0,127}));
  connect(PBat.y,EBat. u)
    annotation (Line(points={{41,20},{58,20}},   color={0,0,127}));
  connect(PPum.y, EPum.u)
    annotation (Line(points={{-39,110},{-22,110}},
                                                 color={0,0,127}));
  connect(POth.y, EOth.u)
    annotation (Line(points={{-39,80},{-22,80}}, color={0,0,127}));
  connect(PBat.y, cha.u) annotation (Line(points={{41,20},{50,20},{50,-10},{58,
          -10}}, color={0,0,127}));
  connect(PBat.y, dis.u) annotation (Line(points={{41,20},{50,20},{50,-50},{58,
          -50}}, color={0,0,127}));
  connect(dis.y, dis1.u)
    annotation (Line(points={{81,-50},{90,-50}}, color={255,0,255}));
  connect(cha.y, cha1.u)
    annotation (Line(points={{81,-10},{90,-10}}, color={255,0,255}));
  connect(PBatDis.u2, dis1.y)
    annotation (Line(points={{118,-50},{113,-50}}, color={0,0,127}));
  connect(PBatCha.u2, cha1.y) annotation (Line(points={{118,-10},{114,-10},{114,
          -10},{113,-10}}, color={0,0,127}));
  connect(PBatCha.u1, cha.u) annotation (Line(points={{118,2},{50,2},{50,-10},{
          58,-10}}, color={0,0,127}));
  connect(PBatDis.u1, dis.u) annotation (Line(points={{118,-38},{50,-38},{50,
          -50},{58,-50}}, color={0,0,127}));
  connect(PBatDis.y, EBatDis.u)
    annotation (Line(points={{141,-44},{148,-44}}, color={0,0,127}));
  connect(PBatCha.y, EBatCha.u)
    annotation (Line(points={{141,-4},{146,-4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{180,140}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{180,
            140}})),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Experimental/COBEE/ElectricalAndFluid/Examples/SingleFamilyResidentialBuilding.mos"
      "Simulate and plot"),
    experiment(
      StartTime=86400,
      StopTime=172800,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end SingleFamilyResidentialBuilding;
