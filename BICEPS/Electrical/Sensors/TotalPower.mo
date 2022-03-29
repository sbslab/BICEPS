within BICEPS.Electrical.Sensors;
model TotalPower "Sensor for total active power"
  extends Buildings.Electrical.Interfaces.PartialTwoPort(
    redeclare package PhaseSystem_p =
        Buildings.Electrical.PhaseSystems.OnePhase,
    redeclare package PhaseSystem_n =
        Buildings.Electrical.PhaseSystems.OnePhase,
    redeclare
      Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terminal_n(
        redeclare package PhaseSystem = PhaseSystem_n),
    redeclare
      Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal_p(
        redeclare package PhaseSystem = PhaseSystem_p));
  Modelica.Blocks.Interfaces.RealOutput P(
    each final quantity="Power",
    each final unit="W")=
    Buildings.Electrical.PhaseSystems.OnePhase.activePower(v=terminal_n.v, i=
    terminal_n.i) "Real power" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-50}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-90})));
equation
  connect(terminal_n, terminal_p) annotation (Line(
      points={{-100,0},{2,0},{2,0},{100,0}},
      color={0,120,120},
      smooth=Smooth.None));
  annotation (defaultComponentName="sen",
  Documentation(info="<html>
<p>
Ideal sensor that measures power, voltage and current.
The two components of the power <i>S</i> are the active and reactive power.
</p>
</html>", revisions="<html>
<ul>
<li>
September 24, 2014, by Michael Wetter:<br/>
Moved assignments outside of equation section to avoid mixing
textual and graphical modeling.
</li>
<li>
September 22, 2014, by Marco Bonvini:<br/>
Fixed bug. The model was referencing the wrong PhaseSystem.
</li>
<li>
August 25, 2014, by Marco Bonvini:<br/>
Revised documentation.
</li>
<li>
July 24, 2013, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}), graphics={
        Rectangle(
          extent={{-70,28},{70,-30}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-92,0},{-70,0}},
          color={0,0,0},
          smooth=Smooth.None),
        Line(
          points={{70,0},{92,0}},
          color={0,0,0},
          smooth=Smooth.None),Text(
          extent={{-140,110},{140,70}},
          lineColor={0,0,0},
          textString="%name"),
        Ellipse(
          extent={{-60,66},{60,-54}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-0.48,33.6},{18,28},{18,59.2},{-0.48,33.6}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-37.6,15.7},{-54,22}},     color={0,0,0}),
        Line(points={{-22.9,34.8},{-32,50}},     color={0,0,0}),
        Line(points={{0,60},{0,42}}, color={0,0,0}),
        Line(points={{22.9,34.8},{32,50}},     color={0,0,0}),
        Line(points={{37.6,15.7},{54,24}},     color={0,0,0}),
        Line(points={{0,2},{9.02,30.6}}, color={0,0,0}),
        Ellipse(
          extent={{-5,7},{5,-3}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-28,-48},{92,-88}},
          lineColor={0,0,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="P")}));
end TotalPower;
