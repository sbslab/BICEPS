within BICEPS.Fluid.Building.Examples;
model ThermoFluid1E "Test model for the thermofluid subsystem"
  extends Modelica.Icons.Example;
  package Medium=Buildings.Media.Water
    "Medium in the building distribution system";
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

  BICEPS.Fluid.Building.ThermoFluidOneElement thermoFluid(
    redeclare package MediumWat = Medium,
    QHea_flow_nominal=261700,
    COP_nominal=4,
    mLoaHea_flow_nominal=10)
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Blocks.Sources.Constant idealElecSig(k=0) "Ideal electrical signal"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
    T=thermoFluid.TDisWatMin,
    nPorts=1) "Source"
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium,
      nPorts=1) "Sink"
    annotation (Placement(transformation(extent={{62,0},{42,20}})));
equation
  connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
      points={{-20,70},{10,70},{10,40}},
      color={255,204,51},
      thickness=0.5));
  connect(idealElecSig.y, thermoFluid.yEle) annotation (Line(points={{-59,70},{-50,
          70},{-50,37},{-2,37}}, color={0,0,127}));
  connect(sou.ports[1], thermoFluid.port_a) annotation (Line(points={{-20,10},{-10,
          10},{-10,24},{0,24}}, color={0,127,255}));
  connect(thermoFluid.port_b, sin.ports[1]) annotation (Line(points={{20,24},{30,
          24},{30,10},{42,10}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Fluid/Building/Examples/ThermoFluid1E.mos"
      "Simulate and plot"),
    experiment(
      StopTime=86400,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ThermoFluid1E;
