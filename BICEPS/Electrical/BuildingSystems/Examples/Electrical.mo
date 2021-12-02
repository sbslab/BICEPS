within BICEPS.Electrical.BuildingSystems.Examples;
model Electrical
  "Example model to demonstrate the electrical subsystem"
  extends Modelica.Icons.Example;
  BICEPS.Electrical.BuildingSystems.Electrical ele(
    lat=weaDat.lat,
    PCon_nominal={PHeaPum.k},
    PPro_nominal=4000,
    PSto_nominal=1000)         "Electrical subsystem"
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  Modelica.Blocks.Continuous.Integrator EGri "Grid energy"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
    "Real grid power"
    annotation (Placement(transformation(extent={{20,60},{40,80}})));
  Modelica.Blocks.Continuous.Integrator Epv "PV energy"
    annotation (Placement(transformation(extent={{60,30},{80,50}})));
  Modelica.Blocks.Continuous.Integrator ELoa "Load energy"
    annotation (Placement(transformation(extent={{60,0},{80,20}})));
  Modelica.Blocks.Continuous.Integrator EBat "Battery energy"
    annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
  Modelica.Blocks.Sources.RealExpression Ppv(y=ele.dev.pv[1].pv.P)
                                                         "PV power"
    annotation (Placement(transformation(extent={{20,30},{40,50}})));
  Modelica.Blocks.Sources.RealExpression PLoa(y=ele.dev.con[1].loa.P)
                                                           "Load power"
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  Modelica.Blocks.Sources.RealExpression PBat(y=-1*ele.dev.bat[1].bat.P)
                                                           "Battery power"
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(computeWetBulbTemperature=
        false, filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid
                                      gri(
    f=60,
    V=208,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Modelica.Blocks.Sources.Constant PHeaPum(k=500)    "Heat pump power"
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
equation
  connect(PGriRea.y,EGri. u)
    annotation (Line(points={{41,70},{58,70}}, color={0,0,127}));
  connect(Ppv.y,Epv. u)
    annotation (Line(points={{41,40},{58,40}}, color={0,0,127}));
  connect(PLoa.y,ELoa. u)
    annotation (Line(points={{41,10},{58,10}},   color={0,0,127}));
  connect(PBat.y,EBat. u)
    annotation (Line(points={{41,-20},{58,-20}}, color={0,0,127}));
  connect(gri.terminal, ele.terminal)
    annotation (Line(points={{-70,40},{-70,17},{-21,17}}, color={0,120,120}));
  connect(weaDat.weaBus, ele.weaBus) annotation (Line(
      points={{-20,70},{-10,70},{-10,20}},
      color={255,204,51},
      thickness=0.5));
  connect(PHeaPum.y, ele.PCon[1]) annotation (Line(points={{-59,-10},{-40,-10},
          {-40,4},{-22,4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/BuildingSystems/Examples/Electrical.mos"
      "Simulate and plot"),
    experiment(
      StartTime=8640000,
      StopTime=8726400,
      Tolerance=1e-6,
      __Dymola_Algorithm="Dassl"));
end Electrical;
