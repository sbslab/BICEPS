within BICEPS.Combined.ElectricalAndFluid.Examples;
model SingleFamilyResidentialBuilding
  "Example model for the electrical and mechanical (thermofluid) coupled system"
  extends Modelica.Icons.Example;
  package Medium=Buildings.Media.Water
    "Medium in the building distribution system";
  BICEPS.Combined.ElectricalAndFluid.SingleFamilyResidentialBuilding bld(
      redeclare package MediumWat = Medium, lat=weaDat.lat) "Building"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Buildings.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    nPorts=1) "Sink"
    annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
  Buildings.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    T=bld.mec.TDisWatMin,
    nPorts=1) "Source"
    annotation (Placement(transformation(extent={{80,-20},{60,0}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
      "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(
    f=60,
    V=208,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
equation
  connect(gri.terminal, bld.terminal)
    annotation (Line(points={{-50,40},{-50,17},{-1,17}}, color={0,120,120}));
  connect(sin.ports[1], bld.port_b) annotation (Line(points={{-40,-10},{-20,-10},
          {-20,4},{0,4}}, color={0,127,255}));
  connect(bld.port_a, sou.ports[1]) annotation (Line(points={{20,4},{40,4},{40,
          -10},{60,-10}}, color={0,127,255}));
  connect(weaDat.weaBus, bld.weaBus) annotation (Line(
      points={{0,70},{10,70},{10,20}},
      color={255,204,51},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=86400, __Dymola_Algorithm="Dassl"));
end SingleFamilyResidentialBuilding;
