within BICEPS.Experimental.Examples;
model RenewableSupplyHeatPump
  extends Modelica.Icons.Example;
  Subsystems.ThermoFluid thrFlu
    annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(computeWetBulbTemperature=
        false, filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data model"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid
                                      gri(
    f=f,
    V=V_nominal,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Subsystems.Electrical electrical
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RenewableSupplyHeatPump;
