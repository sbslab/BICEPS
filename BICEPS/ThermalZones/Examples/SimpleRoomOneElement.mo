within BICEPS.ThermalZones.Examples;
model SimpleRoomOneElement "Test model for the simple thermal zone"
  extends Modelica.Icons.Example;
  BICEPS.ThermalZones.SimpleRoomOneElement zon "Zone"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));

equation
  connect(weaDat.weaBus, zon.weaBus) annotation (Line(
      points={{-20,50},{10,50},{10,20}},
      color={255,204,51},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=12960000,
      StopTime=13824000,
      __Dymola_Algorithm="Dassl"));
end SimpleRoomOneElement;
