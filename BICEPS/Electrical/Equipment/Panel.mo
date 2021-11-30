within BICEPS.Electrical.Equipment;
model Panel "Generic model for an electrical panel"
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-60,80},{60,-80}},
          lineColor={0,140,72},
          lineThickness=0.5),            Text(
          extent={{-56,76},{56,24}},
          lineColor={0,0,255},
          textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=
            false)));
end Panel;
