within BICEPS.Electrical.Device;
model ProducerWind "Wind subsystem"
  extends Buildings.BaseClasses.BaseIcon;
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.Power PWin
    "Nominal power of the wind turbine";
  parameter Modelica.SIunits.Angle lat "Latitude"
    annotation(Evaluate=true,Dialog(group="Orientation"));
  Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
    "Output control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Experimental.Examples.Sensors.RelativeElectricalExergyPotential senWin(
    tol=tol,
    v0=V_nominal,
    k=k) if biomimeticControl
    "Control signal wind"
    annotation (Placement(transformation(extent={{-10,42},{10,62}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    "Generalized electric terminal"
    annotation (Placement(transformation(extent={{-8,-116},{8,-100}}),
        iconTransformation(extent={{-8,-116},{8,-100}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus
    "Weather data bus"
    annotation (Placement(transformation(extent={{-100,-80},{-60,-120}}),
      iconTransformation(extent={{-90,-90},{-70,-110}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.WindTurbine winTur(
    V_nominal=V_nominal,
    h=15,
    hRef=10,
    pf=0.94,
    eta_DCAC=0.92,
    nWin=0.4,
    tableOnFile=false,
    scale=PWin) "Wind turbine model"
    annotation (Placement(transformation(extent={{-20,-20},{-40,0}})));
equation
  connect(senWin.y, yOut)
    annotation (Line(points={{11,60},{110,60}}, color={0,0,127}));
  connect(weaBus.winSpe, winTur.vWin) annotation (Line(
      points={{-80,-100},{-80,12},{-30,12},{-30,2}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(senWin.terminal, terminal)
    annotation (Line(points={{0,42},{0,-108},{0,-108}}, color={0,120,120}));
  connect(winTur.terminal, terminal) annotation (Line(points={{-20,-10},{0,-10},
          {0,-108},{0,-108}}, color={0,120,120}));
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
        Line(
          points={{40,40},{8,40},{-80,40},{-80,-60},{40,-60}},
          color={0,140,72},
          thickness=0.5),
        Line(points={{0,-60},{0,-76},{0,-100}}, color={0,0,0}),
        Polygon(
          points={{-40,-4},{-79.9844,19.3242},{-80,20},{-80.0273,20.627},{-36,2},
              {-40,-4}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{-36,-4},{10,30},{-40,2},{-36,-4}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{-42,-4},{-24,-56},{-36,0},{-42,-4}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-40,-2},{-36,-60}},
          fillColor={233,233,233},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{24,-14},{-20,22},{26,-8},{24,-14}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          origin={20,-2},
          rotation=90,
          lineColor={0,0,0}),
        Polygon(
          points={{-21,-17},{-7.03125,-7.10547},{-6.97266,-1.29297},{-25,-11},{
              -21,-17}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          origin={17,47},
          rotation=90,
          lineColor={0,0,0}),
        Polygon(
          points={{28,22},{77.6289,18.6914},{76.8984,19.9141},{30,28},{28,22}},
          smooth=Smooth.None,
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Rectangle(
          extent={{30,24},{34,-60}},
          fillColor={233,233,233},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-44,4},{-32,-8}},
          lineColor={0,0,0},
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{26,30},{38,18}},
          lineColor={0,0,0},
          fillColor={222,222,222},
          fillPattern=FillPattern.Solid)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false)));
end ProducerWind;
