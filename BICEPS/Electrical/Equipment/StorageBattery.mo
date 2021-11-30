within BICEPS.Electrical.Equipment;
model StorageBattery "Model for a chemical battery for the electrical system"
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-54,-74},{56,38}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Line(
          points={{-70,0}},
          color={0,140,72},
          thickness=0.5),
        Polygon(
          points={{-40,20},{0,74},{40,22},{-40,20}},
          lineColor={0,140,72},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-40,20},{0,74}},
          color={0,140,72},
          thickness=0.5),
        Line(
          points={{40,22},{0,74}},
          color={0,140,72},
          thickness=0.5)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end StorageBattery;
