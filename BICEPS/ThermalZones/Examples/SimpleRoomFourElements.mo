within BICEPS.ThermalZones.Examples;
model SimpleRoomFourElements "Test model for the simple thermal zone"
  extends Modelica.Icons.Example;
  BICEPS.ThermalZones.SimpleRoomFourElements zon "Zone"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=ModelicaServices.ExternalReferences.loadResource(
        "modelica://BICEPS/Resources/weatherdata/DEU_BW_Mannheim_107290_TRY2010_12_Jahr_BBSR.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));

equation
  connect(weaDat.weaBus, zon.weaBus) annotation (Line(
      points={{-20,50},{10,50},{10,20}},
      color={255,204,51},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=2592000, __Dymola_Algorithm="Dassl"));
end SimpleRoomFourElements;
