within BICEPS.Fluid.BuildingSystems.Examples;
model ThermoFluid4E "Test model for the thermofluid subsystem"
  extends Modelica.Icons.Example;
  package Medium=Buildings.Media.Water
    "Medium in the building distribution system";
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=ModelicaServices.ExternalReferences.loadResource(
        "modelica://BICEPS/Resources/weatherdata/DEU_BW_Mannheim_107290_TRY2010_12_Jahr_BBSR.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

  BICEPS.Fluid.BuildingSystems.ThermoFluidFourElements thermoFluid(
    redeclare package MediumWat = Medium,
    biomimeticControl=true,
    show_T=true,
    QHea_flow_nominal(displayUnit="kW") = 20000,
    COP_nominal=4,
    mLoaHea_flow_nominal=1,
    TMin=thermoFluid.T0 - 1.5,
    TMax=thermoFluid.T0 + 2.5)
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Blocks.Sources.Constant idealElecSig(k=0) "Ideal electrical signal"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
    T=285.15,
    nPorts=1) "Source"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium,
      nPorts=1) "Sink"
    annotation (Placement(transformation(extent={{62,0},{42,20}})));
  Modelica.Blocks.Continuous.Integrator EPum "Pump energy"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Continuous.Integrator EHeaPum "Heat Pump energy"
    annotation (Placement(transformation(extent={{60,30},{80,50}})));
  Modelica.Blocks.Sources.Trapezoid yEle(
    amplitude=1,
    rising(displayUnit="h") = 3600,
    width(displayUnit="h") = 10800,
    falling(displayUnit="h") = 3600,
    period(displayUnit="h") = 32400,
    offset=-1,
    startTime(displayUnit="h") = 3600)
    annotation (Placement(transformation(extent={{-80,26},{-60,46}})));
equation
  connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
      points={{-20,70},{10,70},{10,40}},
      color={255,204,51},
      thickness=0.5));
  connect(sou.ports[1], thermoFluid.port_a) annotation (Line(points={{-20,10},{-10,
          10},{-10,24},{0,24}}, color={0,127,255}));
  connect(thermoFluid.port_b, sin.ports[1]) annotation (Line(points={{20,24},{30,
          24},{30,10},{42,10}}, color={0,127,255}));
  connect(thermoFluid.PHeaPum, EHeaPum.u) annotation (Line(points={{21,35},{40,
          35},{40,40},{58,40}}, color={0,0,127}));
  connect(thermoFluid.PPum, EPum.u) annotation (Line(points={{21,38},{30,38},{
          30,70},{58,70}}, color={0,0,127}));
  connect(yEle.y, thermoFluid.yEle) annotation (Line(points={{-59,36},{-22,36},
          {-22,37},{-2,37}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Fluid/BuildingSystems/Examples/ThermoFluid4E.mos"
      "Simulate and plot"),
    experiment(
      StopTime=86400,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ThermoFluid4E;
