within BICEPS.Fluid.Device;
model FanCoilWithDistributionPump
  "Fan coil unit model with distribution pump"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package Medium1=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Medium in the building distribution system";
  replaceable package Medium2=Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Load side medium";
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  // Diagnostics
   parameter Boolean show_T1 = false
    "= true, if actual temperature at port is computed"
    annotation (
      Dialog(tab="Advanced", group="Diagnostics"),
      HideResult=true);
   parameter Boolean show_T2 = false
    "= true, if actual temperature at port is computed"
    annotation (
      Dialog(tab="Advanced", group="Diagnostics"),
      HideResult=true);
  parameter Boolean allowFlowReversal1=false
    "Set to true to allow flow reversal in building distribution system"
    annotation (Dialog(tab="Assumptions"),Evaluate=true);
  parameter Boolean allowFlowReversal2=true
    "Set to true to allow flow reversal on the load side"
    annotation (Dialog(tab="Assumptions"),Evaluate=true);
  parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(
    min=0)=0
    "Building distribution side mass flow rate at nominal conditions in heating mode"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.MassFlowRate m2_flow_nominal(
    min=0)=0
    "Load side mass flow rate at nominal conditions in heating mode"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp1_nominal(
    final min=0,
    displayUnit="Pa")
    "Pressure drop at nominal conditions on building distribution side"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp2_nominal(
    final min=0,
    displayUnit="Pa")
    "Pressure drop at nominal conditions on load side"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.HeatFlowRate QHea_flow_nominal(
    min=0)=0
    "Nominal heating capacity (>=0)"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature TMin=288.15
    "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=298.15
    "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=293.15
    "Nominal value for independent variable. Fixed setpoint if not biomimetic control.";
  // AHRI 440 Standard Heating
  parameter Modelica.SIunits.Temperature T_aHeaWat_nominal=273.15 + 60
    "Heating water inlet temperature at nominal conditions"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature T_bHeaWat_nominal(
    min=273.15,
    displayUnit="degC")=T_aHeaWat_nominal-22.2
    "Heating water outlet temperature at nominal conditions"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature T_aLoaHea_nominal=273.15 + 21.1
    "Load side inlet temperature at nominal conditions in heating mode"
    annotation (Dialog(group="Nominal condition"));
  Medium1.ThermodynamicState sta_a1=
      Medium1.setState_phX(port_a1.p,
                          noEvent(actualStream(port_a1.h_outflow)),
                          noEvent(actualStream(port_a1.Xi_outflow))) if
         show_T1 "Medium properties in port_a1";

  Medium1.ThermodynamicState sta_b1=
      Medium1.setState_phX(port_b1.p,
                          noEvent(actualStream(port_b1.h_outflow)),
                          noEvent(actualStream(port_b1.Xi_outflow))) if
          show_T1 "Medium properties in port_b1";
  Medium2.ThermodynamicState sta_a2=
      Medium2.setState_phX(port_a2.p,
                          noEvent(actualStream(port_a2.h_outflow)),
                          noEvent(actualStream(port_a2.Xi_outflow))) if
         show_T2 "Medium properties in port_a1";

  Medium2.ThermodynamicState sta_b2=
      Medium2.setState_phX(port_b1.p,
                          noEvent(actualStream(port_b2.h_outflow)),
                          noEvent(actualStream(port_b2.Xi_outflow))) if
          show_T2 "Medium properties in port_b2";
  replaceable parameter Buildings.Fluid.Movers.Data.Generic per(
    pressure(
      V_flow=m1_flow_nominal/rho1_default .* {0,1,2},
      dp=dp1_nominal .* {1.5,1,0.5}),
    motorCooledByFluid=false)
    constrainedby Buildings.Fluid.Movers.Data.Generic
    "Record with performance data"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{60,-40},{80,-20}})));
  Buildings.Fluid.FixedResistances.PressureDrop resDis(
    redeclare final package Medium = Medium1,
    final m_flow_nominal=m1_flow_nominal,
    final dp_nominal=dp1_nominal) "Distribution resistence pipe"
    annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(
    p(start=Medium1.p_default),
    redeclare final package Medium = Medium1,
    m_flow(max=if allowFlowReversal1 then +Modelica.Constants.inf else 0),
    h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{110,-70},{90,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(
    p(start=Medium1.p_default),
    redeclare final package Medium = Medium1,
    m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
  Buildings.Fluid.HeatExchangers.DryCoilEffectivenessNTU hex(
    redeclare final package Medium1 = Medium1,
    redeclare final package Medium2 = Medium2,
    final configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    final m1_flow_nominal=m1_flow_nominal,
    final m2_flow_nominal=m2_flow_nominal,
    final dp1_nominal=0,
    final dp2_nominal=0,
    final Q_flow_nominal=QHea_flow_nominal,
    final T_a1_nominal=T_aHeaWat_nominal,
    final T_a2_nominal=T_aLoaHea_nominal,
    final allowFlowReversal1=allowFlowReversal1,
    final allowFlowReversal2=allowFlowReversal2)
    "Heating coil"
    annotation (Placement(transformation(extent={{-20,0},{0,-20}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow fan(
    redeclare final package Medium = Medium2,
    final allowFlowReversal=allowFlowReversal2,
    final m_flow_nominal=m2_flow_nominal,
    redeclare final Buildings.Fluid.Movers.Data.Generic per,
    nominalValuesDefineDefaultPressureCurve=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    final dp_nominal=dp2_nominal)
    "Fan"
    annotation (Placement(transformation(extent={{30,30},{10,50}})));
  Buildings.Fluid.FixedResistances.PressureDrop resLoa(
    redeclare final package Medium = Medium2,
    final m_flow_nominal=m2_flow_nominal,
    final dp_nominal=dp2_nominal)
    "Load side pressure drop"
    annotation (Placement(transformation(extent={{60,30},{40,50}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(
    redeclare final package Medium = Medium2,
    p(start=Medium2.p_default),
    m_flow(min=if allowFlowReversal2 then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
    "Fluid stream inlet port on the load side"
    annotation (Placement(transformation(extent={{90,30},{110,50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(
    redeclare final package Medium = Medium2,
    p(start=Medium2.p_default),
    m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
    h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
    "Fluid stream outlet port on the load side"
    annotation (Placement(transformation(extent={{-90,30},{-110,50}})));
  Buildings.Fluid.Sources.Boundary_pT pRefFan(redeclare package Medium =
        Medium2, nPorts=1) "Reference pressure"
    annotation (Placement(transformation(extent={{60,0},{40,20}})));
  Modelica.Blocks.Interfaces.RealInput y if biomimeticControl "Control signal"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Math.Gain m2Set_flow(k=m2_flow_nominal)
    "Mass flow setpoint for the fan"
    annotation (Placement(transformation(extent={{-20,70},{0,90}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
        Medium2, m_flow_nominal=m2_flow_nominal) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={80,40})));
  Controls.Pump2 con(
    biomimeticControl=biomimeticControl,
    TMin=TMin,
    TMax=TMax,
    T0=T0) "Pump/fan control"
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealOutput PFan(final unit="W")
    "Fan power"
    annotation (Placement(transformation(extent={{100,90},{120,110}}),
    iconTransformation(extent={{100,40},{140,80}})));
  Modelica.Blocks.Sources.Constant TSetSta(k=T0) if not biomimeticControl
    "Static setpoint temperature if not biomimetic control"
    annotation (Placement(transformation(extent={{-100,90},{-80,110}})));
protected
  parameter Medium1.ThermodynamicState sta1_default=Medium1.setState_pTX(
    T=Medium1.T_default,
    p=Medium1.p_default,
    X=Medium1.X_default);
  parameter Modelica.SIunits.Density rho1_default=Medium1.density(
    sta1_default)
    "Density, used to compute fluid volume";
equation
  connect(port_b2, hex.port_b2) annotation (Line(points={{-100,40},{-24,40},{
          -24,-4},{-20,-4}},
                         color={0,127,255}));
  connect(resLoa.port_b, fan.port_a)
    annotation (Line(points={{40,40},{30,40}}, color={0,127,255}));
  connect(resDis.port_b, hex.port_a1) annotation (Line(points={{-30,-60},{-24,
          -60},{-24,-16},{-20,-16}},
                            color={0,127,255}));
  connect(hex.port_b1, port_b1) annotation (Line(points={{0,-16},{4,-16},{4,-60},
          {100,-60}},         color={0,127,255}));
  connect(pRefFan.ports[1], fan.port_a) annotation (Line(points={{40,10},{34,10},
          {34,40},{30,40}}, color={0,127,255}));
  connect(m2Set_flow.y, fan.m_flow_in)
    annotation (Line(points={{1,80},{20,80},{20,52}}, color={0,0,127}));
  connect(fan.port_b, hex.port_a2) annotation (Line(points={{10,40},{4,40},{4,
          -4},{0,-4}}, color={0,127,255}));
  connect(port_a2, senTem.port_a)
    annotation (Line(points={{100,40},{90,40}}, color={0,127,255}));
  connect(senTem.port_b, resLoa.port_a)
    annotation (Line(points={{70,40},{60,40}}, color={0,127,255}));
  connect(y, con.y)
    annotation (Line(points={{-120,80},{-62,80}}, color={0,0,127}));
  connect(con.yOut, m2Set_flow.u)
    annotation (Line(points={{-39,80},{-22,80}}, color={0,0,127}));
  connect(senTem.T, con.TMea) annotation (Line(points={{80,51},{80,60},{-70,60},
          {-70,74},{-62,74}}, color={0,0,127}));
  connect(port_a1, resDis.port_a)
    annotation (Line(points={{-100,-60},{-50,-60}}, color={0,127,255}));
  connect(TSetSta.y, con.TSetSta) annotation (Line(points={{-79,100},{-70,100},
          {-70,86},{-62,86}}, color={0,0,127}));
  connect(PFan, fan.P) annotation (Line(points={{110,100},{4,100},{4,49},{9,49}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,-66},{100,-54}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,34},{100,46}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-72,60},{72,-80}},
          lineColor={27,0,55},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,120}})));
end FanCoilWithDistributionPump;
