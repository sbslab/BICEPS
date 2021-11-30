within BICEPS.Electrical.Equipment;
model StorageBattery
  "Model for a chemical battery for the electrical system"
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  replaceable package PhaseSystem = Buildings.Electrical.PhaseSystems.OnePhase
    annotation (__Dymola_choicesAllMatching=true);
  parameter Real SOC_start=0.5 "Initial charge";
  parameter Modelica.SIunits.Power PBat = 5000
    "Nominal power charge/discharge rate of the battery";
  // 50 kWh
  parameter Modelica.SIunits.Energy EBatMax = 180000000
    "Maximum energy capacity of the battery";
  Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Experimental.Examples.Sensors.RelativeElectricalExergyPotential senBat(
    tol=tol,
    v0=V_nominal,
    k=k)
    "Control signal battery"
    annotation (Placement(transformation(extent={{-10,42},{10,62}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat(
    redeclare package PhaseSystem = PhaseSystem,
    SOC_start=SOC_start,
    EMax(displayUnit="J") = EBatMax,
    V_nominal=V_nominal,
    initMode=Buildings.Electrical.Types.InitMode.zero_current)
    annotation (Placement(transformation(extent={{10,-20},{30,0}})));
  Experimental.Examples.Controls.Battery con(EMax=EBatMax, P_nominal=PBat)
    "Battery controller"
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    "Generalized electric terminal"
    annotation (Placement(transformation(extent={{-12,-112},{4,-96}}),
        iconTransformation(extent={{-8,100},{8,116}})));
  Modelica.Blocks.Interfaces.RealInput yIn
    "Input control signal"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));

equation
  connect(yIn, con.yEle) annotation (Line(points={{-120,60},{-70,60},{-70,26},{-62,
          26}}, color={0,0,127}));
  connect(senBat.terminal, terminal)
    annotation (Line(points={{0,42},{0,-104},{-4,-104}}, color={0,120,120}));
  connect(bat.terminal, terminal) annotation (Line(points={{10,-10},{0,-10},{0,-104},
          {-4,-104}}, color={0,120,120}));
  connect(senBat.y, yOut) annotation (Line(points={{11,60},{110,60}},
                color={0,0,127}));
  connect(con.P, bat.P)
    annotation (Line(points={{-39,20},{20,20},{20,0}}, color={0,0,127}));
  connect(bat.SOC, con.soc) annotation (Line(points={{31,-4},{40,-4},{40,-20},{-70,
          -20},{-70,14},{-62,14}}, color={0,0,127}));
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
          thickness=0.5),
        Rectangle(
          extent={{40,-46},{-30,4}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-30,-30},{-40,-10}},
          lineColor={215,215,215},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{20,-2},{36,-40}},
          lineColor={0,140,72},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          radius=1),
        Rectangle(
          extent={{-26,-2},{-10,-40}},
          lineColor={0,140,72},
          radius=1),
        Line(points={{100,60},{70,60},{34,30}}, color={0,0,127}),
        Line(points={{0,4},{0,100}},            color={0,0,0}),
        Line(points={{-100,60},{-84,60},{-70,60},{-32,30}}, color={0,0,127}),
        Rectangle(
          extent={{-4,-2},{12,-40}},
          lineColor={0,140,72},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid,
          radius=1)}),      Diagram(coordinateSystem(preserveAspectRatio=false)));
end StorageBattery;
