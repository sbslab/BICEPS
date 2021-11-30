within BICEPS.Electrical.Equipment;
model ProducerPhotovoltaicArray "PV array subsystem"
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-6,40},{86,-60}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Rectangle(
          extent={{-10,40},{38,-60}},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-72,-10},{-12,-10},{8,20},{-52,20},{-72,-10}},
          lineThickness=0.5,
          lineColor={0,0,0}),
        Polygon(
          points={{-8,-10},{52,-10},{72,20},{12,20},{-8,-10}},
          lineThickness=0.5,
          lineColor={0,0,0}),
        Polygon(
          points={{-30,-44},{30,-44},{50,-14},{-10,-14},{-30,-44}},
          lineThickness=0.5,
          lineColor={0,0,0}),
        Polygon(
          points={{-80,-44},{-36,-44},{-16,-14},{-76,-14},{-79.3789,-19.0684},{
              -79.9957,-19.9936},{-80,-44}},
          lineThickness=0.5,
          lineColor={0,0,0}),
        Line(
          points={{40,40},{8,40},{-80,40},{-80,-60},{40,-60}},
          color={0,140,72},
          thickness=0.5)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end ProducerPhotovoltaicArray;
