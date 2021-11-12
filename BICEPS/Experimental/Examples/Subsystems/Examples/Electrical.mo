within BICEPS.Experimental.Examples.Subsystems.Examples;
model Electrical "Example model for the electrical subsystem"
  extends Modelica.Icons.Example;
  BICEPS.Experimental.Examples.Subsystems.Electrical ele(V_nominal=480, lat=
        weaDat.lat) "Electrical subsystem"
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(computeWetBulbTemperature=
        false, filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid
                                      gri(
    f=60,
    V=480,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Modelica.Blocks.Sources.Constant PHeaPum(k=1000)   "Heat pump power"
    annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
  Modelica.Blocks.Continuous.Integrator EGri "Grid energy"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
    "Real grid power"
    annotation (Placement(transformation(extent={{20,60},{40,80}})));
  Modelica.Blocks.Continuous.Integrator Epv "PV energy"
    annotation (Placement(transformation(extent={{60,30},{80,50}})));
  Modelica.Blocks.Continuous.Integrator EWin "Wind energy"
    annotation (Placement(transformation(extent={{60,0},{80,20}})));
  Modelica.Blocks.Continuous.Integrator ELoa "Load energy"
    annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
  Modelica.Blocks.Continuous.Integrator EBat "Battery energy"
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Blocks.Sources.RealExpression Ppv(y=ele.pv.P) "PV power"
    annotation (Placement(transformation(extent={{20,30},{40,50}})));
  Modelica.Blocks.Sources.RealExpression PWin(y=ele.winTur.P) "Wind power"
    annotation (Placement(transformation(extent={{20,0},{40,20}})));
  Modelica.Blocks.Sources.RealExpression PLoa(y=ele.loa.P) "Load power"
    annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
  Modelica.Blocks.Sources.RealExpression PBat(y=ele.bat.P) "Battery power"
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
equation
  connect(weaDat.weaBus, ele.weaBus) annotation (Line(
      points={{-20,50},{-10,50},{-10,20}},
      color={255,204,51},
      thickness=0.5));
  connect(gri.terminal, ele.terminal)
    annotation (Line(points={{-70,60},{-70,17},{-21,17}}, color={0,120,120}));
  connect(PHeaPum.y, ele.PHeaPum) annotation (Line(points={{-59,-10},{-30,-10},{
          -30,10},{-22,10}}, color={0,0,127}));
  connect(PGriRea.y, EGri.u)
    annotation (Line(points={{41,70},{58,70}}, color={0,0,127}));
  connect(Ppv.y, Epv.u)
    annotation (Line(points={{41,40},{58,40}}, color={0,0,127}));
  connect(PWin.y, EWin.u)
    annotation (Line(points={{41,10},{58,10}}, color={0,0,127}));
  connect(PLoa.y, ELoa.u)
    annotation (Line(points={{41,-20},{58,-20}}, color={0,0,127}));
  connect(PBat.y, EBat.u)
    annotation (Line(points={{41,-50},{58,-50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Experimental/Examples/Subsystems/Examples/Electrical.mos"
      "Simulate and plot"),
    experiment(StopTime=5184000, __Dymola_Algorithm="Radau"),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=2592000, __Dymola_Algorithm="Radau"));
end Electrical;
