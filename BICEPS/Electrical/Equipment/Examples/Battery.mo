within BICEPS.Electrical.Equipment.Examples;
model Battery "Example model to test and demonstrate the battery"
  extends Modelica.Icons.Example;

  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(
    f=60,
    V=208,
    phiSou=0)
    "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{0,40},{20,60}})));
  Modelica.Blocks.Sources.Ramp PNet(
    height=2500,
    duration(displayUnit="min") = 60,
    offset=-1250,
    startTime(displayUnit="min") = 60)
    "Net power"
    annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
  StorageBattery bat(PBat=1000) "Battery"
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  StorageBattery bat2(
    PBat=1000,
    redeclare BICEPS.Electrical.Equipment.Controls.Battery2 con)
    "Battery"
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
equation
  connect(gri.terminal, bat.terminal)
    annotation (Line(points={{10,40},{10,20.8}},   color={0,120,120}));
  connect(PNet.y, bat.PNetIn)
    annotation (Line(points={{-39,16},{-2,16}},  color={0,0,127}));
  connect(gri.terminal, bat2.terminal) annotation (Line(points={{10,40},{10,30},
          {30,30},{30,-19.2}}, color={0,120,120}));
  connect(PNet.y, bat2.PNetIn) annotation (Line(points={{-39,16},{-20,16},{-20,
          -24},{18,-24}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/Equipment/Examples/Battery.mos"
      "Simulate and plot"),
    experiment(StopTime=240, __Dymola_Algorithm="Dassl"));
end Battery;
