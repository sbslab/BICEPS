within BICEPS.Electrical.Equipment;
model ConsumerThreePhaseBalanced
  "Generic model for an electrical consumer with a three phase balanced load"
  extends Buildings.BaseClasses.BaseIcon;
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(linearized=false,
      mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
  Experimental.Examples.Sensors.RelativeElectricalExergyPotential senV(
    tol=tol,
    v0=V_nominal,
    k=k) if biomimeticControl
    "Control signal load"
    annotation (Placement(transformation(extent={{-10,42},{10,62}})));
  Modelica.Blocks.Math.Gain inv(k=-1) "Invert to be negative (consumption)"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Interfaces.RealInput P(
    final quantity="Power",
    final unit="W",
    min=0,
    displayUnit="kW") "Consumer power (positive value)"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terminal
    "Generalized electric terminal"
    annotation (Placement(transformation(extent={{-4,-112},{12,-96}}),
        iconTransformation(extent={{-8,-116},{8,-100}})));
  Modelica.Blocks.Interfaces.RealOutput y if biomimeticControl
    "Control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
equation
  connect(P, inv.u)
    annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
  connect(inv.y,loa. Pow)
    annotation (Line(points={{-59,0},{-40,0}}, color={0,0,127}));
  connect(loa.terminal, terminal) annotation (Line(points={{-20,0},{0,0},{0,-104},
          {4,-104}}, color={0,120,120}));
  connect(senV.terminal, terminal)
    annotation (Line(points={{0,42},{0,-104},{4,-104}}, color={0,120,120}));
  connect(senV.y, y) annotation (Line(points={{11,60},{110,60}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-54,50},{30,-20},{-54,50}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,20},{10,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={-30,-40},
          rotation=180),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={-30,0},
          rotation=360),
        Rectangle(
          extent={{-10,20},{10,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={0,-40},
          rotation=180),
        Rectangle(
          extent={{-10,20},{10,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          origin={30,-40},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-4.89859e-15,40}},
                                         color={0,0,0},
          origin={0,-60},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.46957e-15,12}},
                                         color={0,0,0},
          origin={-30,-60},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.46957e-15,12}},
                                         color={0,0,0},
          origin={30,-60},
          rotation=180),
        Polygon(
          points={{-40,68},{40,68},{80,0},{40,-72},{-40,-72},{-80,0},{-40,68}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={-30,20},
          rotation=360),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={-30,40},
          rotation=360),
        Rectangle(
          extent={{-30,52},{-18,-10}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                         color={0,0,0},
          origin={-30,-10},
          rotation=180),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={0,40},
          rotation=360),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={0,20},
          rotation=360),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={0,0},
          rotation=360),
        Rectangle(
          extent={{0,52},{12,-10}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={30,40},
          rotation=360),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={30,20},
          rotation=360),
        Ellipse(extent={{-10,-10},{10,10}},
          origin={30,0},
          rotation=360),
        Rectangle(
          extent={{30,52},{42,-10}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                         color={0,0,0},
          origin={0,-10},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                         color={0,0,0},
          origin={30,-10},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-2.20436e-15,18}},
                                         color={0,0,0},
          origin={0,68},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                         color={0,0,0},
          origin={30,60},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                         color={0,0,0},
          origin={-30,60},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{30,8}},
                                         color={0,0,0},
          origin={0,68},
          rotation=180),
          Line(points={{-6.85214e-44,-8.39117e-60},{-30,8}},
                                         color={0,0,0},
          origin={0,68},
          rotation=180),
        Line(points={{-100,0},{-80,0}}, color={0,0,127}),
        Line(points={{100,60},{98,60},{80,60},{56,42}}, color={0,0,127})}),
                           Diagram(coordinateSystem(preserveAspectRatio=false)));
end ConsumerThreePhaseBalanced;
