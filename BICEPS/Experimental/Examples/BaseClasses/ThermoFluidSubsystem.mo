within BICEPS.Experimental.Examples.BaseClasses;
model ThermoFluidSubsystem
  "Thermo-fluid subsystem (secondary) in the renewable supply heat pump example"
  extends Buildings.BaseClasses.BaseIconLow;
  Buildings.Fluid.HeatPumps.Carnot_y heaPum(
    redeclare package Medium1 = Medium1,
    redeclare package Medium2 = Medium2,
    P_nominal=P_nominal,
    dTEva_nominal=dTEva_nominal,
    dTCon_nominal=dTCon_nominal,
    dp1_nominal=6000,
    dp2_nominal=6000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    show_T=true,
    use_eta_Carnot_nominal=false,
    COP_nominal=COP_nominal,
    TCon_nominal=303.15,
    TEva_nominal=278.15) "Heat pump model"
    annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
  Buildings.Fluid.Sources.MassFlowSource_T souCon(
    nPorts=1,
    redeclare package Medium = Medium1,
    use_T_in=true,
    m_flow=m1_flow_nominal,
    T=298.15) "Condenser source"
    annotation (Placement(transformation(extent={{-48,10},{-28,30}})));
  Buildings.Fluid.Sources.MassFlowSource_T evaSou(
    nPorts=1,
    redeclare package Medium = Medium2,
    use_T_in=true,
    m_flow=m2_flow_nominal,
    T=291.15) "Evaporator source"
    annotation (Placement(transformation(extent={{50,-50},{30,-30}})));
  Buildings.Fluid.Sources.Boundary_pT conSin(nPorts=1, redeclare package Medium
      = Medium1) "Condenser sink" annotation (Placement(transformation(extent={
            {10,-10},{-10,10}}, origin={40,22})));
  Buildings.Fluid.Sources.Boundary_pT evaSin(
    use_T_in=true,
    nPorts=1,
    redeclare package Medium = Medium2) "Evaporator sink" annotation (Placement(
        transformation(extent={{-10,-10},{10,10}}, origin={-38,-40})));
  Modelica.Blocks.Interfaces.RealOutput PHeaPum "Heat pump power"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealInput yEle
    "Relative exergetic potential of electrical subsystem"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Sources.Constant TEvaIn(k=273.15 + 10)
    "Inlet temperature to the evaporator"
    annotation (Placement(transformation(extent={{80,-46},{60,-26}})));
  Modelica.Blocks.Math.Add TRet "Return temperature"
    annotation (Placement(transformation(extent={{-80,-46},{-60,-26}})));
equation
  connect(souCon.ports[1], heaPum.port_a1) annotation (Line(
      points={{-28,20},{-20,20},{-20,-4},{-10,-4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(evaSou.ports[1], heaPum.port_a2) annotation (Line(
      points={{30,-40},{20,-40},{20,-16},{10,-16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(heaPum.port_b1, conSin.ports[1]) annotation (Line(
      points={{10,-4},{20,-4},{20,22},{30,22}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(evaSin.ports[1], heaPum.port_b2) annotation (Line(
      points={{-28,-40},{-20,-40},{-20,-16},{-10,-16}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ThermoFluidSubsystem;
