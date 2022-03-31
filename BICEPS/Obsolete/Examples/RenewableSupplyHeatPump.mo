within BICEPS.Obsolete.Examples;
model RenewableSupplyHeatPump
  extends Modelica.Icons.Example;
  Subsystems.ThermoFluid hyd "Hydronic subsystem"
    annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
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
  Subsystems.ElectricalFlat ele(
    PLoa_nominal(displayUnit="kW") = 152000,
    PWin=1,
    PSun(displayUnit="kW") = 15000,
    lat=weaDat.lat) "Electrical subsystem"
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  Modelica.Blocks.Continuous.Integrator EGri "Grid energy"
    annotation (Placement(transformation(extent={{70,60},{90,80}})));
  Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
    "Real grid power"
    annotation (Placement(transformation(extent={{30,60},{50,80}})));
  Modelica.Blocks.Continuous.Integrator Epv "PV energy"
    annotation (Placement(transformation(extent={{70,30},{90,50}})));
  Modelica.Blocks.Continuous.Integrator EWin "Wind energy"
    annotation (Placement(transformation(extent={{70,0},{90,20}})));
  Modelica.Blocks.Continuous.Integrator ELoa "Load energy"
    annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  Modelica.Blocks.Continuous.Integrator EBat "Battery energy"
    annotation (Placement(transformation(extent={{70,-60},{90,-40}})));
  Modelica.Blocks.Sources.RealExpression Ppv(y=ele.pv.P) "PV power"
    annotation (Placement(transformation(extent={{30,30},{50,50}})));
  Modelica.Blocks.Sources.RealExpression PWin(y=ele.winTur.P) "Wind power"
    annotation (Placement(transformation(extent={{30,0},{50,20}})));
  Modelica.Blocks.Sources.RealExpression PLoa(y=ele.loa.P) "Load power"
    annotation (Placement(transformation(extent={{30,-30},{50,-10}})));
  Modelica.Blocks.Sources.RealExpression PBat(y=ele.bat.P) "Battery power"
    annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
equation
  connect(ele.yEle, hyd.yEle) annotation (Line(points={{1,10},{10,10},{10,-10},
          {-30,-10},{-30,-24},{-22,-24}}, color={0,0,127}));
  connect(hyd.PHeaPum, ele.PHeaPum) annotation (Line(points={{1,-24},{10,-24},{
          10,-50},{-40,-50},{-40,10},{-22,10}}, color={0,0,127}));
  connect(gri.terminal, ele.terminal)
    annotation (Line(points={{-70,40},{-70,17},{-21,17}}, color={0,120,120}));
  connect(weaDat.weaBus, ele.weaBus) annotation (Line(
      points={{-20,70},{-10,70},{-10,20}},
      color={255,204,51},
      thickness=0.5));
  connect(PGriRea.y,EGri. u)
    annotation (Line(points={{51,70},{68,70}}, color={0,0,127}));
  connect(Ppv.y,Epv. u)
    annotation (Line(points={{51,40},{68,40}}, color={0,0,127}));
  connect(PWin.y,EWin. u)
    annotation (Line(points={{51,10},{68,10}}, color={0,0,127}));
  connect(PLoa.y,ELoa. u)
    annotation (Line(points={{51,-20},{68,-20}}, color={0,0,127}));
  connect(PBat.y,EBat. u)
    annotation (Line(points={{51,-50},{68,-50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=345600, Tolerance=1e-6, __Dymola_Algorithm="Dassl"));
end RenewableSupplyHeatPump;
