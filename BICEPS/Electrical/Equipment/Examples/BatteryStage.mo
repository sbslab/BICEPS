within BICEPS.Electrical.Equipment.Examples;
model BatteryStage
  "Example model to test and demonstrate the battery staging"
  extends Modelica.Icons.Example;
  Modelica.Blocks.Sources.Ramp PNet(
    height=2500,
    duration(displayUnit="min") = 60,
    offset=-1225,
    startTime(displayUnit="min") = 60) "Net power"
    annotation (Placement(transformation(extent={{-80,26},{-60,46}})));
  Controls.BatteryStage sta(P_nominal=1000) "Stage"
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Modelica.Blocks.Sources.Ramp soc(
    height=1,
    duration(displayUnit="min") = 15,
    offset=0.25,
    startTime(displayUnit="min") = 210) "State of charge"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
equation
  connect(PNet.y, sta.PNetIn)
    annotation (Line(points={{-59,36},{-22,36}}, color={0,0,127}));
  connect(soc.y, sta.soc) annotation (Line(points={{-59,0},{-40,0},{-40,26},{-22,
          26}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/Equipment/Examples/BatteryStage.mos"
      "Simulate and plot"),
    experiment(StopTime=240, __Dymola_Algorithm="Dassl"));
end BatteryStage;
