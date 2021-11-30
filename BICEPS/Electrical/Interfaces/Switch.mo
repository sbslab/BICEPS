within BICEPS.Electrical.Interfaces;
model Switch "Ideal switch"
  extends Modelica.Icons.UnderConstruction;
  extends Buildings.BaseClasses.BaseIconLow;
  extends Buildings.Electrical.Interfaces.PartialConversion(
    redeclare Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terminal_n,
    redeclare Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal_p,
    redeclare package PhaseSystem_p =
        Buildings.Electrical.PhaseSystems.OnePhase,
    redeclare package PhaseSystem_n =
        Buildings.Electrical.PhaseSystems.OnePhase);
  extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
  parameter Modelica.SIunits.Temperature T_ref = 298.15 "Reference temperature";
  parameter Modelica.SIunits.Resistance R(final min=0) = 1e-5
      "Switch resistance when closed (on)";
  parameter Modelica.SIunits.Conductance G(final min=0) = 1e-5
    "Switch conductance when open (off)";
  Modelica.Blocks.Interfaces.BooleanInput on
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
protected
  Real s(final unit="1") "Auxiliary variable";
  constant Buildings.Electrical.PhaseSystems.PartialPhaseSystem.Voltage unitVoltage=1 annotation (HideResult=true);
  constant Buildings.Electrical.PhaseSystems.PartialPhaseSystem.Current unitCurrent=1 annotation (HideResult=true);
equation
  Connections.branch(terminal_p.theta, terminal_n.theta);
  terminal_p.theta = terminal_n.theta;
  0 = i_p + i_n;
  v_p = (s*unitCurrent)*(if on then R else 1);
  i_p = (s*unitVoltage)*(if on then 1 else G);
  LossPower = (v_p - v_n)*i_p;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-40,0},{30,50}}, color={0,0,0}),
        Ellipse(
          extent={{-50,10},{-30,-10}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{30,10},{50,-10}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-100,0},{-50,0}}, color={0,0,0}),
        Line(points={{50,0},{100,0}}, color={0,0,0})}),          Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Switch;
