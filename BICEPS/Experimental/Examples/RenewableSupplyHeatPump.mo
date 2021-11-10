within BICEPS.Experimental.Examples;
model RenewableSupplyHeatPump
  extends Modelica.Icons.Example;
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
    annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou1(
    nPorts=1,
    redeclare package Medium = Medium1,
    use_T_in=true,
    m_flow=m1_flow_nominal,
    T=298.15)
    annotation (Placement(transformation(extent={{-60,-34},{-40,-14}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou2(
    nPorts=1,
    redeclare package Medium = Medium2,
    use_T_in=true,
    m_flow=m2_flow_nominal,
    T=291.15)
    annotation (Placement(transformation(extent={{60,-46},{40,-26}})));
  Buildings.Fluid.Sources.Boundary_pT sin1(nPorts=1, redeclare package Medium
      = Medium1)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, origin={50,0})));
  Buildings.Fluid.Sources.Boundary_pT sin2(nPorts=1, redeclare package Medium
      = Medium2)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-50,-60})));
  Modelica.Blocks.Sources.Ramp uCom(
    height=-1,
    duration=60,
    offset=1,
    startTime=1800) "Compressor control signal"
    annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
  Modelica.Blocks.Sources.Ramp TCon_in(
    height=10,
    duration=60,
    offset=273.15 + 20,
    startTime=60) "Condenser inlet temperature"
    annotation (Placement(transformation(extent={{-90,-30},{-70,-10}})));
  Modelica.Blocks.Sources.Ramp TEva_in(
    height=10,
    duration=60,
    startTime=900,
    offset=273.15 + 15) "Evaporator inlet temperature"
    annotation (Placement(transformation(extent={{50,-80},{70,-60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line line
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.PVSimple pv
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.WindTurbine winTur
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(linearized=
        false, mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
equation
  connect(sou1.ports[1],heaPum. port_a1) annotation (Line(
      points={{-40,-24},{0,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sou2.ports[1],heaPum. port_a2) annotation (Line(
      points={{40,-36},{20,-36}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(heaPum.port_b1,sin1. ports[1]) annotation (Line(
      points={{20,-24},{30,-24},{30,0},{40,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sin2.ports[1],heaPum. port_b2) annotation (Line(
      points={{-40,-60},{-10,-60},{-10,-36},{0,-36}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TCon_in.y,sou1. T_in) annotation (Line(
      points={{-69,-20},{-62,-20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TEva_in.y,sou2. T_in) annotation (Line(
      points={{71,-70},{80,-70},{80,-32},{62,-32}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(uCom.y,heaPum. y) annotation (Line(
      points={{-39,20},{-10,20},{-10,-21},{-2,-21}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(heaPum.P, loa.Pow) annotation (Line(points={{21,-30},{26,-30},{26,10},
          {20,10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RenewableSupplyHeatPump;
