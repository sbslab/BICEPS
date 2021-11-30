within BICEPS.Electrical.BuildingSystems;
model Electrical "Model of a building's electrical system"
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
                             Polygon(
        points={{-50,-76},{-34,-26},{0,-58},{-50,-76}},
        lineColor={0,0,0},
        smooth=Smooth.None,
        fillPattern=FillPattern.Solid,
        fillColor={0,0,0}),      Line(
        points={{40,78},{-28,10},{32,10},{-50,-76},{-50,-76}},
        color={0,0,0},
        smooth=Smooth.None)}), Diagram(coordinateSystem(preserveAspectRatio=
            false)));
end Electrical;
