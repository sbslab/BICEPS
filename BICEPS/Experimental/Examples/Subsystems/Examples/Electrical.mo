within BICEPS.Experimental.Examples.Subsystems.Examples;
model Electrical "Example model for the electrical subsystem"
  extends Modelica.Icons.Example;
  BICEPS.Experimental.Examples.Subsystems.Electrical electrical(V_nominal=480,
      lat=weaDat.lat)
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(computeWetBulbTemperature=
        false, filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid
                                      gri(
    f=60,
    V=480,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Modelica.Blocks.Sources.Constant PHeaPum(k=5000)   "Heat pump power"
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
equation
  connect(weaDat.weaBus, electrical.weaBus) annotation (Line(
      points={{-20,70},{-10,70},{-10,20}},
      color={255,204,51},
      thickness=0.5));
  connect(gri.terminal, electrical.terminal)
    annotation (Line(points={{-70,60},{-70,17},{-21,17}}, color={0,120,120}));
  connect(PHeaPum.y, electrical.PHeaPum) annotation (Line(points={{-59,-10},{
          -30,-10},{-30,10},{-22,10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=86400, __Dymola_Algorithm="Dassl"));
end Electrical;
