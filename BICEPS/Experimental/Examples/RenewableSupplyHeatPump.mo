within BICEPS.Experimental.Examples;
model RenewableSupplyHeatPump
  extends Modelica.Icons.Example;
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line line
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.PVSimple pv
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.WindTurbine winTur
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(linearized=
        false, mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Subsystems.ThermoFluid thrFlu
    annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RenewableSupplyHeatPump;
