within BICEPS.Electrical.Interfaces.Examples;
model Switch "Example model for the disconnect switch"
  extends Modelica.Icons.Example;
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Modelica.SIunits.Frequency f = 60 "Nominal grid frequency";
  parameter Modelica.SIunits.Power P_nominal = 5000
    "Nominal power of a load";
  BICEPS.Electrical.Interfaces.Switch swi
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid
                                      gri(
    f=f,
    V=V_nominal,
    phiSou=0) "Grid model that provides power to the system"
    annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(
    linearized=false,
    mode=Buildings.Electrical.Types.Load.FixedZ_steady_state,
    P_nominal=P_nominal)
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Blocks.Sources.BooleanStep ope(startTime=1) "Open"
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line lin(
    l=1500,
    P_nominal=P_nominal,
    V_nominal=V_nominal) "Power line"
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
equation
  connect(gri.terminal, swi.terminal_n)
    annotation (Line(points={{-50,20},{-50,0},{-20,0}}, color={0,120,120}));
  connect(ope.y, swi.on) annotation (Line(points={{-39,70},{-30,70},{-30,8},{
          -22,8}}, color={255,0,255}));
  connect(swi.terminal_p, lin.terminal_n)
    annotation (Line(points={{0,0},{20,0}}, color={0,120,120}));
  connect(lin.terminal_p, loa.terminal)
    annotation (Line(points={{40,0},{60,0}}, color={0,120,120}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Switch;
