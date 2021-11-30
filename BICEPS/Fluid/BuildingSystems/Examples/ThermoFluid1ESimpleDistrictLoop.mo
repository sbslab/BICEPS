within BICEPS.Fluid.BuildingSystems.Examples;
model ThermoFluid1ESimpleDistrictLoop
  "Test model for the thermofluid subsystem"
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

  BICEPS.Fluid.BuildingSystems.ThermoFluidOneElement thermoFluid(
    redeclare package MediumWat = Medium,
    QHea_flow_nominal=261700,
    COP_nominal=4,
    mLoaHea_flow_nominal=10)
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  Modelica.Blocks.Sources.Constant ideal(k=0)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
      nPorts=1) "Source"
    annotation (Placement(transformation(extent={{80,-40},{60,-20}})));
  Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow
                                                     pumDis(
    redeclare package Medium = Medium,
    final m_flow_nominal=95,
    inputType=Buildings.Fluid.Types.InputType.Constant,
    addPowerToMedium=false,
    nominalValuesDefineDefaultPressureCurve=true)
    "Distribution pump"
    annotation (Placement(transformation(
      extent={{10,-10},{-10,10}},
      rotation=90,
      origin={50,-50})));
  Buildings.Fluid.HeatExchangers.Heater_T pla(
    redeclare package Medium = Medium,
    m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
    dp_nominal=0)                             "Ideal plant"
    annotation (Placement(transformation(extent={{20,-80},{0,-60}})));
  Buildings.Fluid.FixedResistances.Junction spl(
    redeclare package Medium = Medium,
    m_flow_nominal=thermoFluid.mHeaWat_flow_nominal*{1,-1,-1},
    dp_nominal={0,0,0}) "Splitter"
    annotation (Placement(transformation(extent={{-20,-10},{0,-30}})));
  Buildings.Fluid.FixedResistances.Junction jun(
    redeclare package Medium = Medium,
    m_flow_nominal=thermoFluid.mHeaWat_flow_nominal*{1,1,-1},
    dp_nominal={0,0,0}) "Junction"
    annotation (Placement(transformation(extent={{20,-10},{40,-30}})));
  Buildings.Fluid.FixedResistances.PressureDrop pipDis(
    redeclare package Medium = Medium,
    m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
    dp_nominal=60000) "District pipe" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-50})));
  Buildings.Fluid.FixedResistances.PressureDrop pipBld(
    redeclare package Medium = Medium,
    m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
    dp_nominal=6000) "Building pipe" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-10,6})));
  Modelica.Blocks.Sources.Constant TSetDis(k=thermoFluid.TDisWatMin)
    "District loop temperature"
    annotation (Placement(transformation(extent={{80,-90},{60,-70}})));
equation
  connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
      points={{-20,70},{10,70},{10,40}},
      color={255,204,51},
      thickness=0.5));
  connect(ideal.y, thermoFluid.yEle) annotation (Line(points={{-59,70},{-50,70},
          {-50,37},{-2,37}}, color={0,0,127}));
  connect(sou.ports[1], pumDis.port_a)
    annotation (Line(points={{60,-30},{50,-30},{50,-40}}, color={0,127,255}));
  connect(pumDis.port_b, pla.port_a)
    annotation (Line(points={{50,-60},{50,-70},{20,-70}}, color={0,127,255}));
  connect(pla.port_b, pipDis.port_a)
    annotation (Line(points={{0,-70},{-40,-70},{-40,-60}}, color={0,127,255}));
  connect(pipDis.port_b, spl.port_1) annotation (Line(points={{-40,-40},{-40,
          -20},{-20,-20}}, color={0,127,255}));
  connect(spl.port_2, jun.port_1)
    annotation (Line(points={{0,-20},{20,-20}}, color={0,127,255}));
  connect(pumDis.port_a, jun.port_2)
    annotation (Line(points={{50,-40},{50,-20},{40,-20}}, color={0,127,255}));
  connect(spl.port_3, pipBld.port_a)
    annotation (Line(points={{-10,-10},{-10,-4}}, color={0,127,255}));
  connect(pipBld.port_b, thermoFluid.port_a)
    annotation (Line(points={{-10,16},{-10,24},{0,24}},  color={0,127,255}));
  connect(thermoFluid.port_b, jun.port_3)
    annotation (Line(points={{20,24},{30,24},{30,-10}}, color={0,127,255}));
  connect(TSetDis.y, pla.TSet) annotation (Line(points={{59,-80},{30,-80},{30,
          -62},{22,-62}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=31536000, __Dymola_Algorithm="Dassl"));
end ThermoFluid1ESimpleDistrictLoop;
