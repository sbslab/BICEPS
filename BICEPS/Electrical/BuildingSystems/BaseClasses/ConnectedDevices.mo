within BICEPS.Electrical.BuildingSystems.BaseClasses;
model ConnectedDevices
  "Model of distributed electrically connected devices including producers, consumers, and storages."
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Polygon(points={{-10,-10},{10,-10},{22,-30},{10,-50},{-10,-50},{-22,-30},
              {-10,-10}}, lineColor={0,140,72}),
        Ellipse(extent={{66,10},{34,42}}, lineColor={0,140,72}),
        Polygon(
          points={{36,34},{50,50},{64,34},{36,34}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{36,34},{50,50}}, color={0,140,72}),
        Line(points={{50,50},{64,34}}, color={0,140,72}),
        Ellipse(
          extent={{-24,20},{-46,40}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-70,40},{-36,20}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-36,40},{-70,40},{-70,20},{-36,20}}, color={0,140,72}),
        Line(points={{-50,40},{-50,100}}, color={0,0,0}),
        Line(points={{0,-10},{0,100}}, color={0,0,0}),
        Line(points={{50,50},{50,100}}, color={0,0,0})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ConnectedDevices;
