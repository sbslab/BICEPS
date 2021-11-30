within BICEPS.Electrical.Equipment;
model ProducerPV "PV subsystem"
  extends Buildings.BaseClasses.BaseIcon;
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.Power PSun = 4000
    "Nominal power of the PV";
  parameter Modelica.SIunits.Angle lat "Latitude"
    annotation(Evaluate=true,Dialog(group="Orientation"));
  parameter Modelica.SIunits.DensityOfHeatFlowRate W_m2_nominal = 1000
    "Nominal solar power per unit area";
  parameter Real eff_PV = 0.12*0.85*0.9
    "Nominal solar power conversion efficiency (this should consider converion efficiency, area covered, AC/DC losses)";
  parameter Modelica.SIunits.Area A_PV = PSun/eff_PV/W_m2_nominal
    "Nominal area of a P installation";
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.PVSimpleOriented pv(
    eta_DCAC=0.89,
    A=A_PV,
    fAct=0.9,
    eta=0.12,
    linearized=false,
    V_nominal=V_nominal,
    pf=0.85,
    lat=lat,
    azi=Buildings.Types.Azimuth.S,
    til=0.5235987755983) "PV"
    annotation (Placement(transformation(extent={{-20,-30},{-40,-10}})));
  Experimental.Examples.Sensors.RelativeElectricalExergyPotential
                                            senPV(
    tol=tol,
    v0=V_nominal,
    k=k) "Control signal pv"
    annotation (Placement(transformation(extent={{-10,42},{10,62}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    "Generalized electric terminal"
    annotation (Placement(transformation(extent={{-12,-112},{4,-96}}),
        iconTransformation(extent={{-12,-112},{4,-96}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus
    "Weather data bus"
    annotation (Placement(transformation(extent={{-100,-80},{-60,-120}}),
      iconTransformation(extent={{-90,-90},{-70,-110}})));
equation
  connect(senPV.terminal, pv.terminal)
    annotation (Line(points={{0,42},{0,-20},{-20,-20}},    color={0,120,120}));
  connect(pv.terminal, terminal)
    annotation (Line(points={{-20,-20},{0,-20},{0,-104},{-4,-104}},
                                                         color={0,120,120}));
  connect(senPV.y, yOut) annotation (Line(points={{11,60},{110,60}},
                color={0,0,127}));
  connect(weaBus, pv.weaBus) annotation (Line(
      points={{-80,-100},{-80,10},{-30,10},{-30,-11}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
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
          points={{-80,-44},{-36,-44},{-16,-14},{-76,-14},{-79.3789,-19.0684},{-79.9957,
              -19.9936},{-80,-44}},
          lineThickness=0.5,
          lineColor={0,0,0}),
        Line(
          points={{40,40},{8,40},{-80,40},{-80,-60},{40,-60}},
          color={0,140,72},
          thickness=0.5),
        Line(points={{0,-60},{0,-76},{0,-100}}, color={0,0,0})}),
                            Diagram(coordinateSystem(preserveAspectRatio=false)));
end ProducerPV;
