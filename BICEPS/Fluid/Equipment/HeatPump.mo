within BICEPS.Fluid.Equipment;
model HeatPump "Heat pump model"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package Medium1=Modelica.Media.Interfaces.PartialMedium
    "Medium model on condenser side"
    annotation (choices(choice(redeclare package Medium=Buildings.Media.Water "Water"),
    choice(redeclare package Medium =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=293.15,X_a=0.40)
    "Propylene glycol water, 40% mass fraction")));
  replaceable package Medium2=Modelica.Media.Interfaces.PartialMedium
    "Medium model on evaporator side"
    annotation (choices(choice(redeclare package Medium=Buildings.Media.Water "Water"),
    choice(redeclare package Medium =
      Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=293.15,X_a=0.40)
    "Propylene glycol water, 40% mass fraction")));
  parameter Real COP_nominal(final unit="1")
    "Heat pump COP"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature TCon_nominal
    "Condenser outlet temperature used to compute COP_nominal"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature TEva_nominal
    "Evaporator outlet temperature used to compute COP_nominal"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature THeaWatSup_nominal=313.15
    "Heating water supply temperature"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate Q1_flow_nominal(min=0)
    "Heating heat flow rate"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.TemperatureDifference dT1_nominal(
    final min=0) = 5 "Temperature difference condenser outlet-inlet"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.TemperatureDifference dT2_nominal(
    final max=0) = -5 "Temperature difference evaporator outlet-inlet"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Pressure dp1_nominal(displayUnit="Pa")
    "Pressure difference over condenser"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Pressure dp2_nominal(displayUnit="Pa")
    "Pressure difference over evaporator"
    annotation (Dialog(group="Nominal condition"));
  parameter Boolean allowFlowReversal1=false
    "Set to true to allow flow reversal on condenser side"
    annotation (Dialog(tab="Assumptions"), Evaluate=true);
  parameter Boolean allowFlowReversal2=false
    "Set to true to allow flow reversal on evaporator side"
    annotation (Dialog(tab="Assumptions"), Evaluate=true);
  final parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(min=0)=
    heaPum.m1_flow_nominal
    "Mass flow rate on condenser side"
    annotation (Dialog(group="Nominal condition"));
  final parameter Modelica.SIunits.MassFlowRate m2_flow_nominal(min=0)=
    heaPum.m2_flow_nominal
    "Mass flow rate on evaporator side"
    annotation (Dialog(group="Nominal condition"));
  constant Modelica.SIunits.SpecificHeatCapacity cp1_default=
    Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
      p = Medium1.p_default,
      T = Medium1.T_default))
    "Specific heat capacity of the fluid";
  Buildings.Fluid.HeatPumps.Carnot_TCon heaPum(
    redeclare final package Medium1 = Medium1,
    redeclare final package Medium2 = Medium2,
    final dTEva_nominal=dT2_nominal,
    final dTCon_nominal=dT1_nominal,
    final TCon_nominal=TCon_nominal,
    final TEva_nominal=TEva_nominal,
    final allowFlowReversal1=allowFlowReversal1,
    final allowFlowReversal2=allowFlowReversal2,
    final use_eta_Carnot_nominal=false,
    final COP_nominal=COP_nominal,
    final QCon_flow_nominal=Q1_flow_nominal,
    final dp1_nominal=dp1_nominal,
    final dp2_nominal=dp2_nominal) "Heat pump (index 1 for condenser side)"
    annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTConEnt(
    redeclare final package Medium = Medium1,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m1_flow_nominal)
    "Condenser water entering temperature" annotation (Placement(transformation(
        extent={{-80,10},{-60,30}})));
  Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow pumCon(
    redeclare final package Medium = Medium1,
    final m_flow_nominal=m1_flow_nominal,
    final allowFlowReversal=allowFlowReversal1)
    "Heat pump condenser water pump"
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow pumEva(
    redeclare final package Medium = Medium2,
    final m_flow_nominal=m2_flow_nominal,
    final allowFlowReversal=allowFlowReversal2)
    "Heat pump evaporator water pump"
    annotation (Placement(transformation(extent={{120,-110},{100,-90}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput PPum(final unit="W")
    "Pump power"
    annotation (Placement(transformation(extent={{140,60},{160,80}}),
    iconTransformation(extent={{100,40},{140,80}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput PHea(final unit="W")
    "Heat pump power"
    annotation (Placement(transformation(extent={{140,100},{160,120}}),
    iconTransformation(extent={{100,80},{140,120}})));
  Buildings.Controls.OBC.CDL.Continuous.Add addPum "Adder"
    annotation (Placement(transformation(extent={{110,60},{130,80}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(
    redeclare final package Medium = Medium2,
    m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
    h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
    "Fluid port for leaving evaporator water"
    annotation (Placement(
        transformation(extent={{-150,-110},{-130,-90}}),
        iconTransformation(extent={{-110,-110},{-90,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(
    redeclare final package Medium = Medium2,
    m_flow(min=if allowFlowReversal2 then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
    "Fluid port for entering evaporator water"
    annotation (Placement(
        transformation(extent={{130,-110},{150,-90}}),
        iconTransformation(extent={{90,-110},{110,-90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(
    redeclare final package Medium = Medium1,
    m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
    "Fluid port for entering condenser water"
    annotation (Placement(
        transformation(extent={{-150,10},{-130,30}}),
        iconTransformation(extent={{-110,10},{-90,30}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(
    redeclare final package Medium = Medium1,
    m_flow(max=if allowFlowReversal1 then +Modelica.Constants.inf else 0),
    h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
    "Fluid port for leaving condenser water"
    annotation (Placement(
        transformation(extent={{130,10},{150,30}}),
        iconTransformation(extent={{90,10},{110,30}})));
  Controls.HeatPump conHeaPum(
    TMin=273.15 + 28,
    TMax=273.15 + 48,
    T0=273.15 + 38)
    annotation (Placement(transformation(extent={{-80,-30},{-60,-50}})));
  Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold staPum[2](
    y(each start=false),
    t=1e-2 .* {m1_flow_nominal,m2_flow_nominal},
    h=0.5e-2 .* {m1_flow_nominal,m2_flow_nominal})
    "Pump return status"
    annotation (Placement(transformation(extent={{-20,-90},{-40,-70}})));
  Buildings.Controls.OBC.CDL.Logical.And ena
    "Enable heat pump if pump return status on"
    annotation (Placement(transformation(extent={{-62,-90},{-82,-70}})));
  Modelica.Blocks.Interfaces.BooleanInput u
    annotation (Placement(transformation(extent={{-180,100},{-140,140}}),
        iconTransformation(extent={{-120,90},{-100,110}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTConLea(
    redeclare final package Medium = Medium1,
    final allowFlowReversal=allowFlowReversal1,
    final m_flow_nominal=m1_flow_nominal) "Leaving condenser water temperature"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={72,20})));
  Buildings.Fluid.Sensors.MassFlowRate senMasFloHeaWat(
    redeclare final package Medium = Medium1,
    final allowFlowReversal=allowFlowReversal1)
    "Heating water mass flow rate"
    annotation (Placement(transformation(extent={{-120,10},{-100,30}})));
  Buildings.Controls.OBC.CDL.Continuous.Add dTHHW(k1=-1, k2=+1)
    "Heating hot water DeltaT"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain capFloHHW(final k=cp1_default)
    "Capacity flow rate"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Buildings.Controls.OBC.CDL.Continuous.Product loaHHW "Heating load"
    annotation (Placement(transformation(extent={{0,74},{20,94}})));
  Buildings.Controls.OBC.CDL.Continuous.Add heaFloEvaHHW
    "Heat flow rate at evaporator"
    annotation (Placement(transformation(extent={{62,-34},{82,-14}})));
  Controls.PrimaryVariableFlow conFloCon(Q_flow_nominal=Q1_flow_nominal,
      dT_nominal=dT1_nominal)
    annotation (Placement(transformation(extent={{60,74},{80,94}})));
  Controls.PrimaryVariableFlow conFloEva(Q_flow_nominal=-Q1_flow_nominal*(1 + 1
        /COP_nominal), dT_nominal=dT2_nominal)
    annotation (Placement(transformation(extent={{110,-34},{130,-14}})));
  Modelica.Blocks.Interfaces.RealInput yHeaPum "Control signal" annotation (
      Placement(transformation(extent={{-180,60},{-140,100}}),
        iconTransformation(extent={{-120,50},{-100,70}})));
  Buildings.Fluid.Sources.Boundary_pT pRef(redeclare package Medium = Medium1,
      nPorts=1) "Reference pressure"
    annotation (Placement(transformation(extent={{-20,-30},{-40,-10}})));
equation
  connect(port_a1, port_a1)
    annotation (Line(points={{-140,20},{-140,20}}, color={0,127,255}));
  connect(staPum[1].y,ena. u1)
    annotation (Line(points={{-42,-80},{-60,-80}},     color={255,0,255}));
  connect(staPum[2].y,ena. u2) annotation (Line(points={{-42,-80},{-52,-80},{-52,
          -88},{-60,-88}},          color={255,0,255}));
  connect(port_a2, pumEva.port_a)
    annotation (Line(points={{140,-100},{120,-100}},
                                                   color={0,127,255}));
  connect(port_b1, senTConLea.port_b)
    annotation (Line(points={{140,20},{82,20}},  color={0,127,255}));
  connect(senTConLea.port_a, heaPum.port_b1) annotation (Line(points={{62,20},{40,
          20},{40,-24},{30,-24}},  color={0,127,255}));
  connect(port_a1, senMasFloHeaWat.port_a)
    annotation (Line(points={{-140,20},{-120,20}}, color={0,127,255}));
  connect(senMasFloHeaWat.port_b, senTConEnt.port_a)
    annotation (Line(points={{-100,20},{-80,20}}, color={0,127,255}));
  connect(pumCon.port_b, heaPum.port_a1) annotation (Line(points={{-20,20},{0,20},
          {0,-24},{10,-24}},color={0,127,255}));
  connect(senTConLea.T, dTHHW.u2) annotation (Line(points={{72,31},{72,50},{-48,
          50},{-48,64},{-42,64}}, color={0,0,127}));
  connect(senTConEnt.T, dTHHW.u1)
    annotation (Line(points={{-70,31},{-70,76},{-42,76}}, color={0,0,127}));
  connect(capFloHHW.u, senMasFloHeaWat.m_flow) annotation (Line(points={{-102,90},
          {-110,90},{-110,31}},  color={0,0,127}));
  connect(senTConEnt.port_b, pumCon.port_a)
    annotation (Line(points={{-60,20},{-40,20}}, color={0,127,255}));
  connect(capFloHHW.y, loaHHW.u1)
    annotation (Line(points={{-78,90},{-2,90}},   color={0,0,127}));
  connect(loaHHW.u2, dTHHW.y) annotation (Line(points={{-2,78},{-14,78},{-14,70},
          {-18,70}}, color={0,0,127}));
  connect(heaPum.P, heaFloEvaHHW.u2)
    annotation (Line(points={{31,-30},{60,-30}}, color={0,0,127}));
  connect(loaHHW.y, heaFloEvaHHW.u1) annotation (Line(points={{22,84},{50,84},{50,
          -18},{60,-18}}, color={0,0,127}));
  connect(loaHHW.y, conFloCon.loa)
    annotation (Line(points={{22,84},{58,84}}, color={0,0,127}));
  connect(u, conFloCon.u) annotation (Line(points={{-160,120},{50,120},{50,92},{
          58,92}}, color={255,0,255}));
  connect(heaFloEvaHHW.y, conFloEva.loa)
    annotation (Line(points={{84,-24},{108,-24}}, color={0,0,127}));
  connect(u, conFloEva.u) annotation (Line(points={{-160,120},{90,120},{90,-16},
          {108,-16}}, color={255,0,255}));
  connect(conFloEva.m_flow, pumEva.m_flow_in) annotation (Line(points={{132,-24},
          {136,-24},{136,-70},{110,-70},{110,-88}}, color={0,0,127}));
  connect(conFloCon.m_flow, pumCon.m_flow_in) annotation (Line(points={{82,84},{
          86,84},{86,40},{-30,40},{-30,32}}, color={0,0,127}));
  connect(pumCon.m_flow_actual, staPum[1].u) annotation (Line(points={{-19,25},{
          -10,25},{-10,-80},{-18,-80}}, color={0,0,127}));
  connect(pumEva.m_flow_actual, staPum[2].u) annotation (Line(points={{99,-95},{
          99,-94},{-10,-94},{-10,-80},{-18,-80}}, color={0,0,127}));
  connect(ena.y, conHeaPum.uEna) annotation (Line(points={{-84,-80},{-90,-80},{
          -90,-40},{-82,-40}}, color={255,0,255}));
  connect(yHeaPum, conHeaPum.y) annotation (Line(points={{-160,80},{-130,80},{-130,
          -46},{-82,-46}}, color={0,0,127}));
  connect(senTConEnt.T, conHeaPum.TConEnt) annotation (Line(points={{-70,31},{-70,
          40},{-90,40},{-90,-34},{-82,-34}}, color={0,0,127}));
  connect(pumEva.P, addPum.u2) annotation (Line(points={{99,-91},{98,-91},{98,64},
          {108,64}}, color={0,0,127}));
  connect(pumCon.P, addPum.u1) annotation (Line(points={{-19,29},{94,29},{94,76},
          {108,76}}, color={0,0,127}));
  connect(conHeaPum.TSet, heaPum.TSet) annotation (Line(points={{-59,-40},{-6,-40},
          {-6,-22},{8,-22},{8,-21}}, color={0,0,127}));
  connect(heaPum.P, PHea) annotation (Line(points={{31,-30},{36,-30},{36,110},{150,
          110}}, color={0,0,127}));
  connect(addPum.y, PPum)
    annotation (Line(points={{132,70},{150,70}}, color={0,0,127}));
  connect(pRef.ports[1], pumCon.port_a) annotation (Line(points={{-40,-20},{-48,
          -20},{-48,20},{-40,20}}, color={0,127,255}));
  connect(port_b2, heaPum.port_b2) annotation (Line(points={{-140,-100},{0,-100},
          {0,-36},{10,-36}},color={0,127,255}));
  connect(heaPum.port_a2, pumEva.port_b) annotation (Line(points={{30,-36},{40,
          -36},{40,-100},{100,-100}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                         graphics={
        Rectangle(
          extent={{-60,60},{60,-60}},
          lineColor={27,0,55},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
end HeatPump;
