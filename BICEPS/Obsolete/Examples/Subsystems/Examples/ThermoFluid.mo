within BICEPS.Obsolete.Examples.Subsystems.Examples;
model ThermoFluid "Example model for the thermofluid subsystem"
  extends Modelica.Icons.Example;
  BICEPS.Experimental.Examples.Subsystems.ThermoFluid staTF(
    TMin=303.15,
    TMax=323.15,
    T0=313.15) annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Modelica.Blocks.Sources.Constant conSta(k=0) "Constant ideal state"
    annotation (Placement(transformation(extent={{-60,46},{-40,66}})));
  Modelica.Blocks.Sources.Sine staPul(freqHz=1) "Stable pulse"
    annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
  BICEPS.Experimental.Examples.Subsystems.ThermoFluid conTF(
    TMin=303.15,
    TMax=323.15,
    T0=313.15) annotation (Placement(transformation(extent={{0,40},{20,60}})));
  BICEPS.Experimental.Examples.Subsystems.ThermoFluid unStaTF(
    TMin=303.15,
    TMax=323.15,
    T0=313.15) annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Modelica.Blocks.Sources.Sine unStaPul(amplitude=2, freqHz=1) "Unstable pulse"
    annotation (Placement(transformation(extent={{-60,-34},{-40,-14}})));
equation
  connect(staPul.y, staTF.yEle)
    annotation (Line(points={{-39,16},{-2,16}}, color={0,0,127}));
  connect(conSta.y, conTF.yEle)
    annotation (Line(points={{-39,56},{-2,56}}, color={0,0,127}));
  connect(unStaPul.y, unStaTF.yEle) annotation (Line(points={{-39,-24},{-20,-24},
          {-20,-24},{-2,-24}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
   __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Experimental/Examples/Subsystems/Examples/ThermoFluid.mos"
      "Simulate and plot"),
    experiment(
      StopTime=1,
      Tolerance=1e-06),
    Diagram(coordinateSystem(preserveAspectRatio=false)));
end ThermoFluid;
